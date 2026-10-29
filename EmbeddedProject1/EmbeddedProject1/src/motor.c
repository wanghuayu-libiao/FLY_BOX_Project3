/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机 CAN 协议 V1.1 协议层 + 非阻塞初始化序列（见 motor.h）
  *
  *  与上位机 GyroHostCS/Motor.cs 的关系：
  *    - 报文格式、字节序（全部小端）、初始化顺序与已实机验证的 C# 版逐字节一致；
  *    - C# 用 Thread.Sleep 阻塞等待（一键初始化约 3 秒），这里改成由 Motor_Tick()
  *      推进的状态机，每步只执行"发一帧"，不会阻塞任何 FreeRTOS 任务。
  *
  *  单台电机的初始化顺序：
  *      绑定 SN(0x0313) -> 配置参数(0x0316) -> 配置轮径(0x0408 ×3)
  *      -> 设当前位置=0(0x0416 类型 0x0B) -> 使能(0x0413 0x5A)
  *      -> 设加速度(0x0414) -> 查询位置(0x0408/0x08)
  *  两台电机依次做一遍（先左后右），全部完成才置 MOTOR_INIT_READY。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "can.h"
#include "protocol.h"
#include "car_cfg.h"
#include "motor.h"

/* Private define ------------------------------------------------------------*/
#define MOTOR_DEV_SLOTS     16U     /* 反馈帧按设备号索引的槽位数（0x0420~0x042F）*/

/* Private types -------------------------------------------------------------*/

/* 单台电机的静态配置（全部来自 car_cfg.h） */
typedef struct
{
	uint8_t  dev;               /* 设备号，写进报文 Data[4] / Data[7] */
	uint8_t  use_main;          /* 1 = 用主电机帧组，0 = 用从电机帧组 */
	uint8_t  dir_reversed;      /* 1 = 正方向反装（0x0316 Byte6 bit7）*/
	uint16_t wheel_0p01mm;      /* 轮径，0.01mm */
	uint8_t  sn[7];             /* 绑定的 SN，全 0 = 不绑定 */
} MotorCfg_t;

/* Private variables ---------------------------------------------------------*/

/* 配置表：顺序必须与 motor.h 的 MOTOR_LEFT / MOTOR_RIGHT 一致
 *
 * ★★★ SN 不再写死 ★★★
 *   sn 全部填 0 = "不指定 SN"。绑定时会用「上电时电机自己广播的 0x0312」，
 *   见 Motor_GetBootSn()。这样换车/换电机都不用改代码。
 *   只有在上电广播收不到时，才建议在 car_cfg.h 里手写 SN 兜底（见那里的说明）。
 */
static const MotorCfg_t s_cfg[MOTOR_COUNT] =
{
	{
		(uint8_t)CAR_LEFT_DEV,
		(uint8_t)CAR_LEFT_USE_MAIN_FRAME,
		(uint8_t)CAR_LEFT_DIR_REVERSED,
		(uint16_t)CAR_LEFT_WHEEL_0P01MM,
		{ 0U, 0U, 0U, 0U, 0U, 0U, 0U }     /* 0 = 用上电广播的 SN */
	},
	{
		(uint8_t)CAR_RIGHT_DEV,
		(uint8_t)CAR_RIGHT_USE_MAIN_FRAME,
		(uint8_t)CAR_RIGHT_DIR_REVERSED,
		(uint16_t)CAR_RIGHT_WHEEL_0P01MM,
		{ 0U, 0U, 0U, 0U, 0U, 0U, 0U }     /* 0 = 用上电广播的 SN */
	}
};

/* 初始化状态机 */
static volatile MotorInitState_t s_state = MOTOR_INIT_IDLE;
static volatile uint8_t  s_step = MOTOR_STEP_IDLE;   /* 当前步骤（MOTOR_STEP_*）*/
static volatile uint8_t  s_idx  = 0U;                /* 当前初始化第几台电机 */
static volatile uint8_t  s_wheel_repeat = 0U;        /* 配轮径已发几次（要 3 次）*/
static volatile uint32_t s_deadline = 0U;            /* 下一步的时间点（HAL tick）*/
static volatile uint8_t  s_ready_mask = 0U;          /* 已初始化完成的电机位图 */
static volatile uint32_t s_tick_count = 0U;

/* 反馈缓存（按设备号索引） */
static volatile int32_t  s_remain_main[MOTOR_DEV_SLOTS];
static volatile int32_t  s_remain_slave[MOTOR_DEV_SLOTS];
static volatile uint8_t  s_has_remain[MOTOR_DEV_SLOTS];
static volatile int16_t  s_torque[MOTOR_DEV_SLOTS];
static volatile int32_t  s_query_pos[MOTOR_DEV_SLOTS];
static volatile uint8_t  s_has_query_pos[MOTOR_DEV_SLOTS];
static volatile int16_t  s_query_temp[MOTOR_DEV_SLOTS];
static volatile uint8_t  s_has_query_temp[MOTOR_DEV_SLOTS];
static volatile uint8_t  s_query_status[MOTOR_DEV_SLOTS];
static volatile uint8_t  s_has_query_status[MOTOR_DEV_SLOTS];
static volatile uint32_t s_query_version[MOTOR_DEV_SLOTS];
static volatile uint8_t  s_has_query_version[MOTOR_DEV_SLOTS];
static volatile uint32_t s_fb_count[MOTOR_DEV_SLOTS];

/* 总线级状态 */
static volatile uint32_t s_ack_count = 0U;
static volatile uint16_t s_last_ack_id = 0U;
static volatile uint32_t s_err_count = 0U;
static volatile uint16_t s_err_code = 0U;
static volatile uint16_t s_err_frame = 0U;
static volatile uint32_t s_send_fail = 0U;

/* SN 广播缓存 */
static volatile uint8_t  s_rx_sn[7];
static volatile uint32_t s_rx_sn_count = 0U;

/* ★ 上电广播的两台电机 SN（自动识别左右轮要用）
 *
 *   为什么不能只用上面那个 s_rx_sn[7]：
 *   上电时两台电机各广播一条 0x0312，但那个缓存只有一个槽位，
 *   后到的会把先到的覆盖掉 —— 结果只记得一台，另一台永远绑不上。
 *   这里按"首次出现的不同 SN"分别存两个槽位。
 */
static volatile uint8_t  s_boot_sn[2][7];
static volatile uint8_t  s_boot_sn_used = 0U;      /* 已占用几个槽位（0/1/2）*/
static volatile uint32_t s_boot_sn_total = 0U;     /* 累计收到多少条 0x0312（含重复）*/

/* 最近一次收到任何电机帧的时刻（判断电机是否还活着） */
static volatile uint8_t  s_fb_seen = 0U;
static volatile uint32_t s_last_fb_tick = 0U;

/* Watch 变量（加入调试器 Watch 观察）*/
volatile uint8_t  g_motor_state        = 0U;   /* MotorInitState_t */
volatile uint8_t  g_motor_step         = 0U;   /* MOTOR_STEP_* */
volatile uint8_t  g_motor_index        = 0U;   /* 正在初始化第几台 */
volatile uint8_t  g_motor_ready        = 0U;   /* 1 = 两台都初始化完成 */
volatile uint8_t  g_motor_sn_missing   = 0U;   /* 位图：哪台电机没 SN，跳过了绑定 */
volatile uint32_t g_motor_ack_count    = 0U;
volatile uint32_t g_motor_send_fail    = 0U;
volatile int32_t  g_motor_remain_left  = 0;    /* 左电机剩余位移 0.1mm */
volatile int32_t  g_motor_remain_right = 0;    /* 右电机剩余位移 0.1mm */
volatile int32_t  g_motor_pos_left     = 0;    /* 左电机自报绝对位置 0.1mm */
volatile int32_t  g_motor_pos_right    = 0;
volatile int16_t  g_motor_temp_left    = 0;    /* 左电机温度 0.1℃（250 = 25.0℃）*/
volatile int16_t  g_motor_temp_right   = 0;
volatile uint8_t  g_motor_status_left  = 0U;   /* 左电机状态位（MOTOR_STATUS_*）*/
volatile uint8_t  g_motor_status_right = 0U;
volatile uint32_t g_motor_ver_left     = 0U;   /* 左电机固件版本号 */
volatile uint32_t g_motor_ver_right    = 0U;
volatile uint16_t g_motor_error_code   = 0U;   /* 最近一次错误码 */
volatile uint32_t g_motor_rx_sn_count  = 0U;
volatile uint8_t  g_motor_rx_sn[7]     = {0};

/* Private function prototypes -----------------------------------------------*/
static void PutS32(uint8_t *b, int32_t v);
static int32_t GetS32(const uint8_t *b);
static int16_t GetS16(const uint8_t *b, uint8_t ofs);
static uint8_t SnIsZero(const uint8_t *sn);
static uint8_t Motor_PickSn(const MotorCfg_t *c, uint8_t dev_idx, uint8_t *out);

static uint8_t SendPosRaw(const MotorCfg_t *c, int32_t pos, uint8_t target_type);
static uint8_t SendSpdRaw(const MotorCfg_t *c, uint32_t spd);
static uint8_t SendAccRaw(const MotorCfg_t *c, uint32_t acc);
static uint8_t SendLockRaw(const MotorCfg_t *c, uint8_t code);
static uint8_t SendParam(const MotorCfg_t *c);
static uint8_t SendWheel(const MotorCfg_t *c);
static uint8_t SendBind(const MotorCfg_t *c, const uint8_t *sn);
static uint8_t SendQueryRaw(const MotorCfg_t *c, uint8_t type);

static void Motor_Schedule(uint8_t step);
static void Motor_Fail(void);
static void Motor_RefreshDebug(void);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  上电初始化：清状态、清反馈缓存。不发任何 CAN 报文。
  *         必须在 Can_Init() 之后、Motor_StartInit() 之前调用。
  */
void Motor_Init(void)
{
	uint8_t i;

	for (i = 0U; i < MOTOR_DEV_SLOTS; i++)
	{
		s_remain_main[i]    = 0;
		s_remain_slave[i]   = 0;
		s_has_remain[i]     = 0U;
		s_torque[i]         = 0;
		s_query_pos[i]      = 0;
		s_has_query_pos[i]  = 0U;
		s_query_temp[i]     = 0;
		s_has_query_temp[i] = 0U;
		s_query_status[i]   = 0U;
		s_has_query_status[i] = 0U;
		s_query_version[i]  = 0U;
		s_has_query_version[i] = 0U;
		s_fb_count[i]       = 0U;
	}

	for (i = 0U; i < 7U; i++)
	{
		s_rx_sn[i] = 0U;
		g_motor_rx_sn[i] = 0U;
	}

	s_state        = MOTOR_INIT_IDLE;
	s_step         = MOTOR_STEP_IDLE;
	s_idx          = 0U;
	s_wheel_repeat = 0U;
	s_deadline     = 0U;
	s_ready_mask   = 0U;
	s_tick_count   = 0U;
	s_ack_count    = 0U;
	s_last_ack_id  = 0U;
	s_err_count    = 0U;
	s_err_code     = 0U;
	s_err_frame    = 0U;
	s_send_fail    = 0U;
	s_rx_sn_count  = 0U;
	s_fb_seen      = 0U;
	s_last_fb_tick = 0U;

	g_motor_state        = (uint8_t)MOTOR_INIT_IDLE;
	g_motor_step         = MOTOR_STEP_IDLE;
	g_motor_index        = 0U;
	g_motor_ready        = 0U;
	g_motor_sn_missing   = 0U;
	g_motor_ack_count    = 0U;
	g_motor_send_fail    = 0U;
	g_motor_remain_left  = 0;
	g_motor_remain_right = 0;
	g_motor_pos_left     = 0;
	g_motor_pos_right    = 0;
	g_motor_temp_left    = 0;
	g_motor_temp_right   = 0;
	g_motor_status_left  = 0U;
	g_motor_status_right = 0U;
	g_motor_ver_left     = 0U;
	g_motor_ver_right    = 0U;
	g_motor_error_code   = 0U;
	g_motor_rx_sn_count  = 0U;
}

/**
  * @brief  启动初始化序列（从第一台电机的"绑定 SN"开始）
  *         可重复调用：会从头再来一遍，用于电机断电重启后重新配置
  */
void Motor_StartInit(void)
{
	s_idx          = 0U;
	s_wheel_repeat = 0U;
	s_ready_mask   = 0U;
	s_step         = MOTOR_STEP_BIND_SN;
	s_deadline     = HAL_GetTick();
	s_state        = MOTOR_INIT_RUNNING;

	g_motor_index      = 0U;
	g_motor_step       = MOTOR_STEP_BIND_SN;
	g_motor_state      = (uint8_t)MOTOR_INIT_RUNNING;
	g_motor_ready      = 0U;
	g_motor_sn_missing = 0U;
}

/**
  * @brief  推进初始化状态机：每 MOTOR_TICK_PERIOD_MS（10ms）调用一次
  *         每次最多下发一帧，步与步之间间隔 MOTOR_INIT_GAP_MS（300ms）
  * @retval 1 = 本次推进了一步，0 = 还没到时间 / 已经结束
  */
uint8_t Motor_Tick(void)
{
	const MotorCfg_t *c;

	if (s_state != MOTOR_INIT_RUNNING)
	{
		return 0U;
	}

	if ((int32_t)(HAL_GetTick() - s_deadline) < 0)
	{
		return 0U;                              /* 还在等上一步的间隔 */
	}

	c = &s_cfg[s_idx];
	s_tick_count++;
	g_motor_index = s_idx;

	switch (s_step)
	{
	case MOTOR_STEP_BIND_SN:
	{
		uint8_t sn[7];

		if (Motor_PickSn(c, s_idx, sn) != 0U)
		{
			if (SendBind(c, sn) != 0U) { Motor_Fail(); return 1U; }
		}
		else
		{
			/* 没有 SN：跳过绑定，但电机很可能不会转（实测行为），置位标志提醒 */
			g_motor_sn_missing |= (uint8_t)(1U << s_idx);
		}
		Motor_Schedule(MOTOR_STEP_CONFIG_PARAM);
		break;
	}

	case MOTOR_STEP_CONFIG_PARAM:
		if (SendParam(c) != 0U) { Motor_Fail(); return 1U; }
		s_wheel_repeat = 0U;
		Motor_Schedule(MOTOR_STEP_CONFIG_WHEEL);
		break;

	case MOTOR_STEP_CONFIG_WHEEL:
		/* 协议要求配轮径连发 3 次，每次之间也要留间隔 */
		if (SendWheel(c) != 0U) { Motor_Fail(); return 1U; }
		s_wheel_repeat++;
		Motor_Schedule((s_wheel_repeat >= 3U) ? MOTOR_STEP_SET_ZERO : MOTOR_STEP_CONFIG_WHEEL);
		break;

	case MOTOR_STEP_SET_ZERO:
		/* 目标类型 0x0B 只存在于主帧组（0x0416，DLC=6）；从帧组没有类型字节 */
		if (c->use_main != 0U)
		{
			if (SendPosRaw(c, 0, MOTOR_TARGET_SET_CURRENT) != 0U) { Motor_Fail(); return 1U; }
		}
		Motor_Schedule(MOTOR_STEP_ENABLE);
		break;

	case MOTOR_STEP_ENABLE:
		if (SendLockRaw(c, MOTOR_LOCK_ENABLE) != 0U) { Motor_Fail(); return 1U; }
		Motor_Schedule(MOTOR_STEP_SET_ACC);
		break;

	case MOTOR_STEP_SET_ACC:
		if (SendAccRaw(c, (uint32_t)CAR_DEFAULT_ACC_0P1MMPS2) != 0U) { Motor_Fail(); return 1U; }
		Motor_Schedule(MOTOR_STEP_QUERY_POS);
		break;

	case MOTOR_STEP_QUERY_POS:
		/* 查一次位置：既验证通信，又拿到一个绝对位置基准 */
		if (SendQueryRaw(c, MOTOR_QUERY_POS) != 0U) { Motor_Fail(); return 1U; }
		Motor_Schedule(MOTOR_STEP_DONE);
		break;

	case MOTOR_STEP_DONE:
		s_ready_mask |= (uint8_t)(1U << s_idx);
		s_idx++;
		if (s_idx >= MOTOR_COUNT)
		{
			s_state = MOTOR_INIT_READY;
			s_step  = MOTOR_STEP_IDLE;
			g_motor_state = (uint8_t)MOTOR_INIT_READY;
			g_motor_step  = MOTOR_STEP_DONE;
			g_motor_ready = 1U;
		}
		else
		{
			s_wheel_repeat = 0U;
			Motor_Schedule(MOTOR_STEP_BIND_SN);
		}
		break;

	default:
		s_step = MOTOR_STEP_IDLE;
		break;
	}

	return 1U;
}

MotorInitState_t Motor_GetInitState(void) { return s_state; }
uint8_t Motor_GetStep(void)               { return s_step; }
uint8_t Motor_GetInitingIndex(void)       { return s_idx; }
uint8_t Motor_IsReady(void)               { return (uint8_t)((s_state == MOTOR_INIT_READY) ? 1U : 0U); }

/**
  * @brief  解析一帧电机报文（在 CAN 接收任务里调用）
  *         非电机 ID 的帧直接忽略，所以可以和陀螺仪命令帧混在同一个循环里调用
  */
void Motor_OnFrame(const CanFrame_t *frame)
{
	uint16_t id;
	uint8_t  dev;

	if (frame == NULL)
	{
		return;
	}

	id = frame->id;

	/* --- 0x0420 + 设备号：剩余位移（主/从两个通道都在这一帧里）--- */
	if ((id >= CAN_ID_MOTOR_FB_POS_BASE) &&
	    (id < (uint16_t)(CAN_ID_MOTOR_FB_POS_BASE + MOTOR_DEV_SLOTS)))
	{
		dev = (uint8_t)(id - CAN_ID_MOTOR_FB_POS_BASE);
		if (frame->dlc < 8U) { return; }
		s_remain_main[dev]  = GetS32(&frame->data[0]);
		s_remain_slave[dev] = GetS32(&frame->data[4]);
		s_has_remain[dev]   = 1U;
		s_fb_count[dev]++;
		s_last_fb_tick = HAL_GetTick();
		s_fb_seen = 1U;
		Motor_RefreshDebug();
		return;
	}

	/* --- 0x0440 + 设备号：力矩（Data[2..3] 为 s16）--- */
	if ((id >= CAN_ID_MOTOR_FB_TORQUE_BASE) &&
	    (id < (uint16_t)(CAN_ID_MOTOR_FB_TORQUE_BASE + MOTOR_DEV_SLOTS)))
	{
		dev = (uint8_t)(id - CAN_ID_MOTOR_FB_TORQUE_BASE);
		if (frame->dlc < 4U) { return; }
		s_torque[dev] = GetS16(frame->data, 2U);
		s_last_fb_tick = HAL_GetTick();
		s_fb_seen = 1U;
		return;
	}

	/* --- 0x0380 + 设备号：心跳 --- */
	if ((id >= CAN_ID_MOTOR_FB_HEART_BASE) &&
	    (id < (uint16_t)(CAN_ID_MOTOR_FB_HEART_BASE + MOTOR_DEV_SLOTS)))
	{
		dev = (uint8_t)(id - CAN_ID_MOTOR_FB_HEART_BASE);
		s_fb_count[dev]++;
		s_last_fb_tick = HAL_GetTick();
		s_fb_seen = 1U;
		return;
	}

	/* --- 0x0409：查询应答。Data[0]=设备号，Data[1]=查询类型，Data[2..] 为结果 --- */
	if (id == CAN_ID_MOTOR_QUERY_RESP)
	{
		if (frame->dlc < 2U) { return; }
		dev = (uint8_t)(frame->data[0] & 0x0FU);

		switch (frame->data[1])
		{
		case MOTOR_QUERY_POS:                   /* 当前位置：Data[2..5] s32，0.1mm */
			if (frame->dlc < 6U) { return; }
			s_query_pos[dev]     = GetS32(&frame->data[2]);
			s_has_query_pos[dev] = 1U;
			break;

		case MOTOR_QUERY_TEMP:                  /* 温度：Data[2..3] s16，0.1℃ */
			if (frame->dlc < 4U) { return; }
			s_query_temp[dev]     = GetS16(frame->data, 2U);
			s_has_query_temp[dev] = 1U;
			break;

		case MOTOR_QUERY_STATUS:                /* 状态位：Data[2] */
			if (frame->dlc < 3U) { return; }
			s_query_status[dev]     = frame->data[2];
			s_has_query_status[dev] = 1U;
			break;

		case MOTOR_QUERY_VERSION:               /* 版本号：Data[2..5] u32 */
			if (frame->dlc < 6U) { return; }
			s_query_version[dev]     = (uint32_t)GetS32(&frame->data[2]);
			s_has_query_version[dev] = 1U;
			break;

		default:
			return;                             /* 未定义的查询类型，忽略 */
		}

		s_last_fb_tick = HAL_GetTick();
		s_fb_seen = 1U;
		Motor_RefreshDebug();
		return;
	}

	/* --- 0x041A：参数设置 ACK（Data[0..1] = 被应答命令帧 ID 的低 16 位）--- */
	if (id == CAN_ID_MOTOR_ACK)
	{
		if (frame->dlc < 2U) { return; }
		s_last_ack_id = (uint16_t)((uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8));
		s_ack_count++;
		g_motor_ack_count = s_ack_count;
		s_last_fb_tick = HAL_GetTick();
		s_fb_seen = 1U;
		return;
	}

	/* --- 0x0312：SN 应答 / 上电广播（7 字节）--- */
	if (id == CAN_ID_MOTOR_SN)
	{
		uint8_t i;
		uint8_t dup = 0U;

		if (frame->dlc < 7U) { return; }

		for (i = 0U; i < 7U; i++)
		{
			s_rx_sn[i] = frame->data[i];
			g_motor_rx_sn[i] = frame->data[i];
		}
		s_rx_sn_count++;
		g_motor_rx_sn_count = s_rx_sn_count;
		s_boot_sn_total++;

		/* ★ 记进「两个不同 SN」的槽位：重复广播不占新槽，超出的直接忽略。
		 *   这样上电时两台电机各广播一条，就能拿到两条不同的 SN，
		 *   自动识别左右轮时不用再依赖 car_cfg.h 里写死的那两行。 */
		for (i = 0U; i < s_boot_sn_used; i++)
		{
			uint8_t k;
			uint8_t same = 1U;

			for (k = 0U; k < 7U; k++)
			{
				if (s_boot_sn[i][k] != frame->data[k]) { same = 0U; break; }
			}
			if (same != 0U) { dup = 1U; break; }
		}
		if ((dup == 0U) && (s_boot_sn_used < 2U))
		{
			uint8_t k;
			for (k = 0U; k < 7U; k++)
			{
				s_boot_sn[s_boot_sn_used][k] = frame->data[k];
			}
			s_boot_sn_used++;
		}

		/* 收到新的 SN = 电机刚重新上电，参数和轮径都丢了，必须重新初始化 */
		if (s_state == MOTOR_INIT_READY)
		{
			s_state = MOTOR_INIT_IDLE;
			s_ready_mask = 0U;
			g_motor_state = (uint8_t)MOTOR_INIT_IDLE;
			g_motor_ready = 0U;
		}
		return;
	}

	/* --- 0x0100 ~ 0x017F：错误上报（Data[0..1] = 错误码）--- */
	if ((id >= CAN_ID_MOTOR_ERR_BASE) && (id <= CAN_ID_MOTOR_ERR_END))
	{
		if (frame->dlc < 2U) { return; }
		/* 小端解析（与协议里其它多字节字段一致）：Data[0]=低字节 */
		s_err_code  = (uint16_t)((uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8));
		s_err_frame = id;
		s_err_count++;
		g_motor_error_code = s_err_code;
		return;
	}
}

/* ------------------------------------------------------------------ 底层指令 */

uint8_t Motor_SendPosRel(uint8_t idx, int32_t dist_0p1mm)
{
	if (idx >= MOTOR_COUNT) { return 1U; }
	return SendPosRaw(&s_cfg[idx], dist_0p1mm, MOTOR_TARGET_RELATIVE);
}

uint8_t Motor_SendPosAbs(uint8_t idx, int32_t pos_0p1mm)
{
	if (idx >= MOTOR_COUNT) { return 1U; }
	return SendPosRaw(&s_cfg[idx], pos_0p1mm, MOTOR_TARGET_ABSOLUTE);
}

uint8_t Motor_SendSetZero(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 1U; }
	if (s_cfg[idx].use_main == 0U) { return 1U; }   /* 从帧组没有目标类型字节 */
	return SendPosRaw(&s_cfg[idx], 0, MOTOR_TARGET_SET_CURRENT);
}

uint8_t Motor_SendSpeed(uint8_t idx, uint32_t speed_0p1mmps)
{
	if ((idx >= MOTOR_COUNT) || (speed_0p1mmps == 0U)) { return 1U; }
	return SendSpdRaw(&s_cfg[idx], speed_0p1mmps);
}

uint8_t Motor_SendAcc(uint8_t idx, uint32_t acc_0p1mmps2)
{
	if ((idx >= MOTOR_COUNT) || (acc_0p1mmps2 == 0U)) { return 1U; }
	return SendAccRaw(&s_cfg[idx], acc_0p1mmps2);
}

uint8_t Motor_SendLock(uint8_t idx, uint8_t lock_code)
{
	if (idx >= MOTOR_COUNT) { return 1U; }
	if ((lock_code != MOTOR_LOCK_ENABLE) && (lock_code != MOTOR_LOCK_RELEASE)) { return 1U; }
	return SendLockRaw(&s_cfg[idx], lock_code);
}

uint8_t Motor_SendQuery(uint8_t idx, uint8_t query_type)
{
	if (idx >= MOTOR_COUNT) { return 1U; }
	return SendQueryRaw(&s_cfg[idx], query_type);
}

uint8_t Motor_SupportsSetZero(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_cfg[idx].use_main;
}

uint8_t Motor_QueryTemp(uint8_t idx)    { return Motor_SendQuery(idx, MOTOR_QUERY_TEMP); }
uint8_t Motor_QueryPos(uint8_t idx)     { return Motor_SendQuery(idx, MOTOR_QUERY_POS); }
uint8_t Motor_QueryStatus(uint8_t idx)  { return Motor_SendQuery(idx, MOTOR_QUERY_STATUS); }
uint8_t Motor_QueryVersion(uint8_t idx) { return Motor_SendQuery(idx, MOTOR_QUERY_VERSION); }

/**
  * @brief  BB 声提示：0x0408，DLC=6，[设备号, 0x07, 音量, 频率低, 频率中, 频率高]
  *         ★ 文档只给了"频率信息等"，没有明确编码；这里按 24 位小端发送，
  *           实机第一次使用时请确认音调是否符合预期。
  */
uint8_t Motor_SendBeep(uint8_t idx, uint8_t volume, uint32_t freq)
{
	uint8_t d[6];

	if (idx >= MOTOR_COUNT) { return 1U; }

	d[0] = s_cfg[idx].dev;
	d[1] = MOTOR_CFG_SUB_BEEP;
	d[2] = volume;
	d[3] = (uint8_t)(freq);
	d[4] = (uint8_t)(freq >> 8);
	d[5] = (uint8_t)(freq >> 16);
	return Can_SendStd(CAN_ID_MOTOR_CFG_WHEEL, d, 6U);
}

uint8_t Motor_RequestSn(uint8_t idx)
{
	uint8_t d[2];

	(void)idx;                                  /* SN 请求是广播的，与设备号无关 */
	d[0] = (uint8_t)CAR_MOTOR_SN_DEVTYPE;
	d[1] = 0U;
	return Can_SendStd(CAN_ID_MOTOR_GET_SN, d, 2U);
}

/* ------------------------------------------------------------------ 状态读取 */

uint8_t Motor_GetDev(uint8_t idx)           { return (idx < MOTOR_COUNT) ? s_cfg[idx].dev : 0xFFU; }
uint8_t Motor_GetUseMainFrame(uint8_t idx)  { return (idx < MOTOR_COUNT) ? s_cfg[idx].use_main : 0U; }

uint8_t Motor_HasRemain(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_has_remain[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

int32_t Motor_GetRemain(uint8_t idx)
{
	uint8_t dev;

	if (idx >= MOTOR_COUNT) { return 0; }
	dev = (uint8_t)(s_cfg[idx].dev & 0x0FU);
	return (s_cfg[idx].use_main != 0U) ? s_remain_main[dev] : s_remain_slave[dev];
}

int16_t Motor_GetTorque(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0; }
	return s_torque[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint8_t Motor_HasQueryPos(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_has_query_pos[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

int32_t Motor_GetQueryPos(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0; }
	return s_query_pos[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint32_t Motor_GetFeedbackSeq(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_fb_count[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint8_t Motor_HasQueryTemp(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_has_query_temp[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

int16_t Motor_GetQueryTemp(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0; }
	return s_query_temp[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint8_t Motor_HasQueryStatus(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_has_query_status[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint8_t Motor_GetQueryStatus(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_query_status[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint8_t Motor_HasQueryVersion(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_has_query_version[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint32_t Motor_GetQueryVersion(uint8_t idx)
{
	if (idx >= MOTOR_COUNT) { return 0U; }
	return s_query_version[(uint8_t)(s_cfg[idx].dev & 0x0FU)];
}

uint32_t Motor_GetAckCount(void)     { return s_ack_count; }
uint16_t Motor_GetLastAckId(void)    { return s_last_ack_id; }
uint16_t Motor_GetErrorCode(void)    { return s_err_code; }
uint16_t Motor_GetErrorFrameId(void) { return s_err_frame; }
uint32_t Motor_GetErrorCount(void)   { return s_err_count; }

const uint8_t *Motor_GetRxSn(void)   { return (const uint8_t *)s_rx_sn; }
uint32_t Motor_GetRxSnCount(void)    { return s_rx_sn_count; }

/**
  * @brief  取上电广播里第 idx 台的不同 SN（自动识别左右轮用）
  * @param  idx  0 或 1
  * @retval SN 指针（7 字节）；该槽位还没收到过则返回 0
  */
const uint8_t *Motor_GetBootSn(uint8_t idx)
{
	if (idx >= s_boot_sn_used)
	{
		return 0;
	}
	return (const uint8_t *)s_boot_sn[idx];
}

/** @brief 上电广播里已收集到几台不同 SN 的电机（0 / 1 / 2）*/
uint8_t Motor_GetBootSnUsed(void)    { return s_boot_sn_used; }

/** @brief 累计收到多少条 0x0312 广播（含重复）*/
uint32_t Motor_GetBootSnTotal(void)  { return s_boot_sn_total; }

/** @brief 清空上电广播的 SN 槽位（重新识别前调用）*/
void Motor_ClearBootSn(void)
{
	uint8_t i, k;

	for (i = 0U; i < 2U; i++)
	{
		for (k = 0U; k < 7U; k++)
		{
			s_boot_sn[i][k] = 0U;
		}
	}
	s_boot_sn_used  = 0U;
	s_boot_sn_total = 0U;
}

uint32_t Motor_GetMsSinceLastFrame(void)
{
	if (s_fb_seen == 0U)
	{
		return 0xFFFFFFFFU;                     /* 从未收到过任何电机帧 */
	}
	return HAL_GetTick() - s_last_fb_tick;
}

/* Private functions ---------------------------------------------------------*/

static void PutS32(uint8_t *b, int32_t v)
{
	uint32_t u = (uint32_t)v;

	b[0] = (uint8_t)u;
	b[1] = (uint8_t)(u >> 8);
	b[2] = (uint8_t)(u >> 16);
	b[3] = (uint8_t)(u >> 24);
}

static int32_t GetS32(const uint8_t *b)
{
	return (int32_t)((uint32_t)b[0] |
	                 ((uint32_t)b[1] << 8) |
	                 ((uint32_t)b[2] << 16) |
	                 ((uint32_t)b[3] << 24));
}

static int16_t GetS16(const uint8_t *b, uint8_t ofs)
{
	return (int16_t)((uint16_t)b[ofs] | ((uint16_t)b[ofs + 1U] << 8));
}

static uint8_t SnIsZero(const uint8_t *sn)
{
	uint8_t i;

	for (i = 0U; i < 7U; i++)
	{
		if (sn[i] != 0U) { return 0U; }
	}
	return 1U;
}

/**
  * @brief  取该电机要绑定的 SN
  *
  *         优先级：
  *           ① car_cfg.h 里手写的 SN（非全 0 时）—— 只在特殊情况下兜底用
  *           ② ★ 上电广播收集到的两个不同 SN，按【设备号索引】分配
  *              （设备 0 拿第 1 条、设备 1 拿第 2 条）
  *           ③ 最近一条 0x0312（最后手段）
  *
  *         ★ 为什么按设备号索引取而不是"随便拿一条"：
  *           两台电机各广播一条，必须给每个设备号分配不同的 SN，
  *           否则两台会被绑成同一个号 —— 那就是"车只会原地打转"的经典故障。
  *
  *         ★ 顺序（谁先广播）是不确定的，但这不影响功能：
  *           绑错的只是"哪台算左"，而左右由 Car_AutoDetectDirections()
  *           用推车法测出来，不依赖这里的顺序。
  *
  * @param  dev_idx  设备索引（0 或 1，对应 MOTOR_LEFT / MOTOR_RIGHT）
  * @retval 1 = out 里是有效 SN，0 = 没有任何 SN 可用
  */
static uint8_t Motor_PickSn(const MotorCfg_t *c, uint8_t dev_idx, uint8_t *out)
{
	uint8_t i;

	if (SnIsZero(c->sn) == 0U)
	{
		for (i = 0U; i < 7U; i++) { out[i] = c->sn[i]; }
		return 1U;
	}

	/* ② 上电广播里收集到的不同 SN，按设备索引分配 */
	if (dev_idx < s_boot_sn_used)
	{
		for (i = 0U; i < 7U; i++) { out[i] = s_boot_sn[dev_idx][i]; }
		if (SnIsZero(out) == 0U) { return 1U; }
	}

	/* ③ 最后手段：最近一条广播 */
	if (s_rx_sn_count == 0U)
	{
		return 0U;
	}

	for (i = 0U; i < 7U; i++) { out[i] = s_rx_sn[i]; }
	return 1U;
}

static uint8_t SendPosRaw(const MotorCfg_t *c, int32_t pos, uint8_t target_type)
{
	uint8_t d[6];

	PutS32(d, pos);
	d[4] = c->dev;

	if (c->use_main != 0U)
	{
		d[5] = target_type;                     /* 主帧组：DLC=6，最后一字节是目标类型 */
		return Can_SendStd(CAN_ID_MOTOR_SET_POS_MAIN, d, 6U);
	}

	/* 从帧组：DLC=5，Data[4] 就是设备号，没有目标类型字节 */
	return Can_SendStd(CAN_ID_MOTOR_SET_POS_SLAVE, d, 5U);
}

static uint8_t SendSpdRaw(const MotorCfg_t *c, uint32_t spd)
{
	uint8_t d[5];

	PutS32(d, (int32_t)spd);
	d[4] = c->dev;
	return Can_SendStd((c->use_main != 0U) ? CAN_ID_MOTOR_SET_SPD_MAIN : CAN_ID_MOTOR_SET_SPD_SLAVE,
	                   d, 5U);
}

static uint8_t SendAccRaw(const MotorCfg_t *c, uint32_t acc)
{
	uint8_t d[5];

	PutS32(d, (int32_t)acc);
	d[4] = c->dev;
	return Can_SendStd((c->use_main != 0U) ? CAN_ID_MOTOR_SET_ACC_MAIN : CAN_ID_MOTOR_SET_ACC_SLAVE,
	                   d, 5U);
}

static uint8_t SendLockRaw(const MotorCfg_t *c, uint8_t code)
{
	uint8_t d[5];

	d[0] = code;
	d[1] = 0U;
	d[2] = 0U;
	d[3] = 0U;
	d[4] = c->dev;
	return Can_SendStd(CAN_ID_MOTOR_LOCK, d, 5U);
}

/**
  * @brief  配置电机参数 0x0316（8 字节），位打包与上位机 MotorConfigParam.TryPack 一致
  */
static uint8_t SendParam(const MotorCfg_t *c)
{
	uint8_t d[8];

	d[0] = (uint8_t)(((uint8_t)CAR_MOTOR_BYTE0_HI << 4) | (uint8_t)CAR_MOTOR_POLE_PAIRS);
	d[1] = (uint8_t)(((uint8_t)CAR_MOTOR_SPEED_P << 4) | (uint8_t)CAR_MOTOR_SPEED_I);
	d[2] = (uint8_t)(((uint8_t)CAR_MOTOR_TORQUE_P << 4) | (uint8_t)CAR_MOTOR_TORQUE_I);
	d[3] = (uint8_t)(((uint8_t)CAR_MOTOR_CURRENT_CW << 4) | (uint8_t)CAR_MOTOR_CURRENT_CCW);
	d[4] = (uint8_t)((((uint8_t)CAR_MOTOR_BRAKE != 0U) ? 0x02U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_DAMPING != 0U) ? 0x01U : 0x00U));
	d[5] = (uint8_t)(((uint8_t)CAR_MOTOR_SPEED_D << 4) | (uint8_t)CAR_MOTOR_TORQUE_D);
	d[6] = (uint8_t)(((c->dir_reversed != 0U) ? 0x80U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_AUTO_UNLOCK != 0U) ? 0x40U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_RETURN_REMAIN != 0U) ? 0x00U : 0x20U) |
	                 (((uint8_t)CAR_MOTOR_RETURN_TORQUE != 0U) ? 0x00U : 0x10U) |
	                 ((uint8_t)c->dev & 0x0FU));          /* 马达号 = 设备号（本车 0/1）*/
	d[7] = c->dev;                              /* 设备号 */

	return Can_SendStd(CAN_ID_MOTOR_CFG_PARAM, d, 8U);
}

/**
  * @brief  配置轮径 0x0408：DLC=4，[设备号, 0x24, 轮径低字节, 轮径高字节]，单位 0.01mm
  *         协议要求连发 3 次，重复次数由调用方（状态机）控制
  */
static uint8_t SendWheel(const MotorCfg_t *c)
{
	uint8_t d[4];

	d[0] = c->dev;
	d[1] = MOTOR_CFG_SUB_WHEEL;
	d[2] = (uint8_t)(c->wheel_0p01mm & 0xFFU);
	d[3] = (uint8_t)(c->wheel_0p01mm >> 8);
	return Can_SendStd(CAN_ID_MOTOR_CFG_WHEEL, d, 4U);
}

static uint8_t SendBind(const MotorCfg_t *c, const uint8_t *sn)
{
	uint8_t d[8];
	uint8_t i;

	for (i = 0U; i < 7U; i++) { d[i] = sn[i]; }
	d[7] = c->dev;
	return Can_SendStd(CAN_ID_MOTOR_BIND_SN, d, 8U);
}

static uint8_t SendQueryRaw(const MotorCfg_t *c, uint8_t type)
{
	uint8_t d[2];

	d[0] = c->dev;
	d[1] = type;
	return Can_SendStd(CAN_ID_MOTOR_QUERY, d, 2U);
}

static void Motor_Schedule(uint8_t step)
{
	s_step     = step;
	s_deadline = HAL_GetTick() + MOTOR_INIT_GAP_MS;
	g_motor_step = step;
}

static void Motor_Fail(void)
{
	s_send_fail++;
	g_motor_send_fail = s_send_fail;
	s_state = MOTOR_INIT_FAILED;
	g_motor_state = (uint8_t)MOTOR_INIT_FAILED;
	g_motor_step  = s_step;
}

/* 把"按设备号"缓存的反馈映射成"按左/右"的 Watch 变量 */
static void Motor_RefreshDebug(void)
{
	uint8_t i;

	for (i = 0U; i < MOTOR_COUNT; i++)
	{
		uint8_t dev = (uint8_t)(s_cfg[i].dev & 0x0FU);
		int32_t remain = (s_cfg[i].use_main != 0U) ? s_remain_main[dev] : s_remain_slave[dev];
		int32_t pos    = s_query_pos[dev];

		if (i == MOTOR_LEFT)
		{
			g_motor_remain_left  = remain;
			g_motor_pos_left     = pos;
			g_motor_temp_left    = s_query_temp[dev];
			g_motor_status_left  = s_query_status[dev];
			g_motor_ver_left     = s_query_version[dev];
		}
		else
		{
			g_motor_remain_right  = remain;
			g_motor_pos_right     = pos;
			g_motor_temp_right    = s_query_temp[dev];
			g_motor_status_right  = s_query_status[dev];
			g_motor_ver_right     = s_query_version[dev];
		}
	}
}
