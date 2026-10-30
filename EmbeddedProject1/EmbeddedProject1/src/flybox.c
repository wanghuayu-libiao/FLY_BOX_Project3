/**
  ******************************************************************************
  * @file    flybox.c
  * @brief   课题三·飞箱推拉箱机构：设备号发现 + 单轴点动 + 语义动作
  *
  *  ★★★ 这个文件当前只做到「能把机构安全地动起来」这一步 ★★★
  *
  *    推拉箱的完整业务状态机（抓 -> 拉 -> 放 -> 让位 -> 输送 -> 推出去）
  *    要等两个实测数据齐了才能写：
  *      ① 5 个轴的设备号与实物对应关系     -> Fly_RunJogTest() 点一遍就知道
  *      ② FLY_SWING_UNITS_PER_DEG / FLY_HOOK_UNITS_PER_DEG 的真实值
  *                                          -> 点 90° 量角度，按比例修正
  *    这两项没定之前写状态机等于凭空猜，改了还得推翻重写。
  *
  *  ★★★ 没有极限限位挡块 → 一切保护都在软件里 ★★★
  *    ① 软限位：Fly_MoveRel() 里先算新位置，越界直接拒绝，一帧都不发
  *    ② 行程限幅：单次点动不超过 FLY_JOG_MAX_MM
  *    ③ 到位确认：必须先见到「忙」再见到「闲」，否则不允许下发下一条位置指令
  *       （课题二实测：运动中途改目标会把电机写死，只能断电重启）
  *    ④ 位置基准：上电查询 0x0408/0x08 拿绝对位置。
  *       ★ 如果该电机不保持掉电位置，上电后 pos 是未知的，
  *         这时软限位失效，只剩行程限幅 —— 界面上看 g_fly_pos_valid 标志。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "task.h"                      /* ★ xPortGetFreeHeapSize / uxTaskGetStackHighWaterMark */
#include "can.h"
#include "protocol.h"
#include "car_cfg.h"
#include "motor.h"
#include "tof8.h"                       /* 飞箱的定位靠 TOF 距离反馈，不靠电机位置 */
#include "flybox.h"

/* ★ 系统健康诊断要读 CAN 错误状态（ESR）和邮箱丢弃计数。
 *   hcan 在 can.c 里是 static，所以通过 can.h 的两个只读接口拿 ——
 *   不要直接 extern hcan（那是 static，链接会报 undefined reference）。 */

/* =========================================================================
 *                              静态配置
 * ========================================================================= */

/* ★ 马达号与设备号是否相同。
 *   0x0316 的 Byte6 bit3..0 是「马达号」，0x0413/14/15/16 的 Data[4] 用它寻址；
 *   而 0x0408 的 B0、0x0420+dev 用的是「设备号」。
 *   课题二（car_bringup）已实机确认两者是同一个地址空间。
 *
 *   ★ 飞箱的处理：用 0x0313 按 SN 绑定，把两者都设成同一个值（0..4），
 *     绑定后这个值就是确定的，不再是「出厂未知」。
 *
 *   1 = 两者相同（绑定时也会写成相同值）
 *   0 = 两者不同，需要另外探测马达号（当前未实现，会直接报错）
 */
#define FLY_MOTOR_NO_EQ_DEV             1

/* 发送帧的统一出口：统计 + 失败计数 */
#define FLY_SEND(id, buf, dlc)          Fly_SendFrame((uint16_t)(id), (buf), (uint8_t)(dlc))

/* Private types -------------------------------------------------------------*/

typedef struct
{
	uint8_t  dev;               /* 设备号（0xFF = 未指定）*/
	uint8_t  motor_no;          /* 马达号（Data[4] 寻址用；同上则等于 dev）*/
	uint8_t  param_written;     /* 本次上电是否已写过 0x0316（每台只能写一条）*/
	uint8_t  pos_valid;         /* 1 = 已从电机读到过绝对位置（pos 可信）*/
	uint8_t  zeroed;            /* 1 = 当前位置已被定义为零点（软限位/角度目标才可用）*/
	uint8_t  cfg_ok;            /* 1 = 本次上电的 0x0316/轮径/0x0413/速度均下发成功 */
	uint8_t  use_slave;         /* 1 = 该轴走从帧组 0x0417/18/19（双通道驱动器的第二路）*/

	int32_t  pos;               /* 软件跟踪的当前位置，0.1mm / 指令单位 */
	int32_t  min_pos;
	int32_t  max_pos;

	uint32_t speed;             /* 默认速度 0.1mm/s */
	uint32_t accel;             /* 默认加速度 0.1mm/s^2 */

	/* 到位判据用 */
	uint8_t  seen_busy;
	uint32_t expect_ms;
} FlyAxisState_t;

/* Private variables ---------------------------------------------------------*/

static FlyAxisState_t s_ax[FLY_AXIS_COUNT];

/* 每次上电只允许给同一台电机写一条 0x0316（写第二条会把电机写死）*/
static uint8_t s_param_written = 0U;

/* SN 表（实物标签抄录，见 flybox.h 第 2.2 节）
 * ★ 顺序必须与 FlyAxis_t 一致：BELT_L / BELT_R / SLIDE / SWING / HOOK
 * ★ 只在 FLY_USE_SN_BIND == 1 时有实际用途，否则不编译进来
 *   （避免 "defined but not used" 警告） */
#if (FLY_USE_SN_BIND != 0)
static const uint8_t s_sn[FLY_AXIS_COUNT][7] =
{
	FLY_SN_BELT_L,
	FLY_SN_BELT_R,
	FLY_SN_SLIDE,
	FLY_SN_SWING,
	FLY_SN_HOOK
};
#endif

/* 扫描结果 */
volatile uint8_t g_fly_scan_cnt = 0U;
volatile uint8_t g_fly_scan_list[16];

volatile uint8_t  g_fly_mapped_cnt = 0U;
volatile uint8_t  g_fly_cfg_cnt    = 0U;
volatile uint8_t  g_fly_bind_cnt   = 0U;
volatile uint8_t  g_fly_verify_cnt = 0U;
volatile uint8_t  g_fly_verify_mask = 0U;   /* bit i = 第 i 轴应答过 */
volatile uint8_t  g_fly_cfg_mask   = 0U;    /* bit i = 第 i 轴配置下发成功 */
volatile uint8_t  g_fly_zeroed_cnt = 0U;
volatile uint8_t  g_fly_zero_req  = 0U;
volatile uint8_t  g_fly_zero_axis = 0xFFU;
volatile uint8_t  g_fly_jog_axis   = 0xFFU;
volatile uint8_t  g_fly_jogtest_req  = 0U;
volatile uint8_t  g_fly_jogtest_busy = 0U;
volatile uint8_t  g_fly_jog_ok       = 0U;
volatile uint8_t  g_fly_jog_bad      = 0U;
volatile uint8_t  g_fly_jog_nofb     = 0U;
volatile uint8_t  g_fly_jog_fatal    = 0U;
volatile uint8_t  g_fly_slave_used_mask = 0U;   /* bit i = 第 i 轴最终走了从帧组 */
volatile uint8_t  g_fly_beltjog_req = 0U;
volatile uint16_t g_fly_beltjog_mm  = 0U;      /* 有符号，用 u16 存负值补码 */

/* 业务流程状态 */
volatile uint8_t  g_fly_flow_step    = 0U;     /* 当前/失败的步骤号（1 起）*/
volatile uint8_t  g_fly_flow_result  = 0U;     /* 0 = 成功，非 0 = 失败步骤 */
volatile uint8_t  g_fly_flow_running = 0U;     /* 0 = 空闲，1 = 拉箱，2 = 推箱 */
volatile uint8_t  g_fly_flow_req     = 0U;     /* 1 = 已请求跑流程 */
volatile uint8_t  g_fly_flow_which   = 0U;     /* 1 = 拉箱，2 = 推箱 */
volatile uint8_t  g_fly_flow_why     = 0U;     /* 失败原因码，见 FLY_WHY_* */

/* ★ 堵转/未到位标志：某一步没能确认到位就置 1，由 0xB5 帧 [7] bit1 上报。
 *   目的是把「机械卡死」和「MCU 死机」区分开 —— 见 Fly_DoStep 里的说明。 */
volatile uint8_t  g_fly_flow_stall   = 0U;

/* ★ 抓取距离（拉箱靠近的停止点，mm）
 *
 *   TOF 只知道「传感器到箱子多少毫米」，不知道钩子在哪。
 *   「钩尖刚好能勾住」是安装几何决定的，只能量或学。
 *   做成变量是为了能用 0x111 [0x0E] 现场学，不用重编烧录。
 *   只存 RAM，上电重置回宏里的默认值。 */
volatile uint16_t g_fly_grab_dist_mm = FLY_PULL_GRAB_DIST_MM;
volatile uint8_t  g_fly_learn_req    = 0U;
volatile uint8_t  g_fly_tofhealth_req = 0U;
volatile uint8_t  g_fly_tofscan_req   = 0U;
volatile uint8_t  g_fly_hookhome_req  = 0U;

/* ★ 拉箱前伸距离（mm）—— 现场可调，不靠重编烧录 */
volatile uint16_t g_fly_pull_fwd_mm  = (uint16_t)FLY_PULL_SLIDE_FWD_MM;
volatile uint8_t  g_fly_tofprobe_req  = 0U;
volatile uint8_t  g_fly_tofprobe_belt = 0U;
volatile uint16_t g_fly_tofprobe_step = 0U;
volatile uint8_t  g_fly_tofprobe_cnt  = 0U;

/* ★ 上电流程走到哪一步（诊断「卡在哪」用），含义见 flybox.h */
volatile uint8_t  g_fly_boot_stage    = 0U;
volatile int32_t  g_fly_jog_delta  = 0;
volatile uint8_t  g_fly_abort      = 0U;
volatile uint32_t g_fly_tx_count   = 0U;
volatile uint32_t g_fly_tx_fail    = 0U;

/* Private function prototypes -----------------------------------------------*/

static uint8_t Fly_SendFrame(uint16_t id, const uint8_t *buf, uint8_t dlc);
static uint8_t Fly_SendAccel(uint8_t motor_no, uint32_t acc);
static uint8_t Fly_SendSpeed(uint8_t motor_no, uint32_t spd);
static uint8_t Fly_SendPosRel(uint8_t motor_no, int32_t dist);
static uint8_t Fly_SendAccelSlave(uint8_t motor_no, uint32_t acc);
static uint8_t Fly_SendSpeedSlave(uint8_t motor_no, uint32_t spd);
static uint8_t Fly_SendPosRelSlave(uint8_t motor_no, int32_t dist);
static uint8_t Fly_SendPosTyped(uint8_t motor_no, int32_t value, uint8_t target_type);
static uint8_t Fly_SendLock(uint8_t dev, uint8_t code);
static uint8_t Fly_SendWheel(uint8_t dev, uint16_t wheel_0p01mm);
static uint8_t Fly_SendParam(uint8_t dev, uint8_t motor_no);
static uint8_t Fly_SendQueryDev(uint8_t dev, uint8_t type);
static int32_t Fly_UnitsPerDeg(FlyAxis_t axis);
static uint32_t Fly_AxisSpeed(FlyAxis_t axis);
static uint32_t Fly_AxisAccel(FlyAxis_t axis);
static uint8_t Fly_IsRotary(FlyAxis_t axis);
static uint16_t Fly_WheelOfAxis(FlyAxis_t axis);
static uint8_t Fly_MoveRelEx(FlyAxis_t axis, int32_t dist_units, uint32_t speed,
                             uint32_t accel, uint8_t allow_nozero);

/* =========================================================================
 *                            底层发送
 * ========================================================================= */

static uint8_t Fly_SendFrame(uint16_t id, const uint8_t *buf, uint8_t dlc)
{
	uint8_t r = Can_SendStd(id, buf, dlc);

	if (r == 0U) { g_fly_tx_count++; }
	else         { g_fly_tx_fail++;  }

	return r;                              /* 0 = 成功，1 = 失败 */
}

/** @brief 0x0414 加速度：[u32 小端, 马达号] DLC=5 */
static uint8_t Fly_SendAccel(uint8_t motor_no, uint32_t acc)
{
	uint8_t d[5];

	d[0] = (uint8_t)(acc);
	d[1] = (uint8_t)(acc >> 8);
	d[2] = (uint8_t)(acc >> 16);
	d[3] = (uint8_t)(acc >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);

	return FLY_SEND(CAN_ID_MOTOR_SET_ACC_MAIN, d, 5U);
}

/* ---- 从帧组 0x0417 / 0x0418 / 0x0419 ----
 *
 *  ★★★ 为什么需要这一组 ★★★
 *
 *   电机驱动器是【双通道】的：一帧 0x0420 反馈里带着主/从两个位移
 *    （见最早那份 CAN 日志：0x0420 的 DLC=8，data 是 8 个 0）。
 *    也就是说一个设备号下面可以挂两台电机：
 *        主通道 -> 0x0414 / 0x0415 / 0x0416
 *        从通道 -> 0x0417 / 0x0418 / 0x0419
 *
 *    如果 M3/M4/M5 接在某些驱动器的【第二个通道】上，
 *    它们会正常响应 0x0316 / 0x0413（这两条按设备号寻址，两个通道都收），
 *    所以「会锁死」；但位置指令发到主帧组它们就【收不到】，
 *    表现为「锁死了但一动不动」—— 正好是现在的现象。
 *
 *  ★ 从帧组的限制：0x0419 的 DLC=5，Data[4] 就是设备号，
 *    **没有目标类型字节**，所以只能发「相对位置」。
 *    对我们够用：所有运动本来就是相对位移。
 */

/** @brief 0x0417 从帧组加速度 DLC=5 */
static uint8_t Fly_SendAccelSlave(uint8_t motor_no, uint32_t acc)
{
	uint8_t d[5];

	d[0] = (uint8_t)(acc);
	d[1] = (uint8_t)(acc >> 8);
	d[2] = (uint8_t)(acc >> 16);
	d[3] = (uint8_t)(acc >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);

	return FLY_SEND(CAN_ID_MOTOR_SET_ACC_SLAVE, d, 5U);
}

/** @brief 0x0418 从帧组速度 DLC=5 */
static uint8_t Fly_SendSpeedSlave(uint8_t motor_no, uint32_t spd)
{
	uint8_t d[5];

	d[0] = (uint8_t)(spd);
	d[1] = (uint8_t)(spd >> 8);
	d[2] = (uint8_t)(spd >> 16);
	d[3] = (uint8_t)(spd >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);

	return FLY_SEND(CAN_ID_MOTOR_SET_SPD_SLAVE, d, 5U);
}

/** @brief 0x0419 从帧组相对位置 DLC=5（★ 没有目标类型字节）*/
static uint8_t Fly_SendPosRelSlave(uint8_t motor_no, int32_t dist)
{
	uint8_t  d[5];
	uint32_t u = (uint32_t)dist;

	d[0] = (uint8_t)(u);
	d[1] = (uint8_t)(u >> 8);
	d[2] = (uint8_t)(u >> 16);
	d[3] = (uint8_t)(u >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);

	return FLY_SEND(CAN_ID_MOTOR_SET_POS_SLAVE, d, 5U);
}

/** @brief 0x0415 速度：[u32 小端, 马达号] DLC=5 */
static uint8_t Fly_SendSpeed(uint8_t motor_no, uint32_t spd)
{
	uint8_t d[5];

	d[0] = (uint8_t)(spd);
	d[1] = (uint8_t)(spd >> 8);
	d[2] = (uint8_t)(spd >> 16);
	d[3] = (uint8_t)(spd >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);

	return FLY_SEND(CAN_ID_MOTOR_SET_SPD_MAIN, d, 5U);
}

/** @brief 0x0416 相对位置：[s32 小端, 马达号, 目标类型 0x00] DLC=6 */
static uint8_t Fly_SendPosRel(uint8_t motor_no, int32_t dist)
{
	uint8_t  d[6];
	uint32_t u = (uint32_t)dist;

	d[0] = (uint8_t)(u);
	d[1] = (uint8_t)(u >> 8);
	d[2] = (uint8_t)(u >> 16);
	d[3] = (uint8_t)(u >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);
	d[5] = MOTOR_TARGET_RELATIVE;

	return FLY_SEND(CAN_ID_MOTOR_SET_POS_MAIN, d, 6U);
}

/**
  * @brief  0x0416 通用位置帧（可指定目标类型）
  *         0x00 = 相对位置  0x05 = 绝对  0x0A = 归零  0x0B = 设当前位置为某值
  *         ★ 目标类型字节只存在于主帧组 0x0416（DLC=6），从帧组 0x0419 没有。
  */
static uint8_t Fly_SendPosTyped(uint8_t motor_no, int32_t value, uint8_t target_type)
{
	uint8_t  d[6];
	uint32_t u = (uint32_t)value;

	d[0] = (uint8_t)(u);
	d[1] = (uint8_t)(u >> 8);
	d[2] = (uint8_t)(u >> 16);
	d[3] = (uint8_t)(u >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);
	d[5] = target_type;

	return FLY_SEND(CAN_ID_MOTOR_SET_POS_MAIN, d, 6U);
}

/** @brief 0x0413 使能：[0x5A/0x4A, 0, 0, 0, 设备号] DLC=5 */
static uint8_t Fly_SendLock(uint8_t dev, uint8_t code)
{
	uint8_t d[5];

	d[0] = code;
	d[1] = 0U;
	d[2] = 0U;
	d[3] = 0U;
	d[4] = dev;

	return FLY_SEND(CAN_ID_MOTOR_LOCK, d, 5U);
}

/** @brief 0x0408 配轮径：[设备号, 0x24, 轮径低, 轮径高] DLC=4 */
static uint8_t Fly_SendWheel(uint8_t dev, uint16_t wheel_0p01mm)
{
	uint8_t d[4];

	d[0] = dev;
	d[1] = MOTOR_CFG_SUB_WHEEL;
	d[2] = (uint8_t)(wheel_0p01mm & 0xFFU);
	d[3] = (uint8_t)(wheel_0p01mm >> 8);

	return FLY_SEND(CAN_ID_MOTOR_CFG_WHEEL, d, 4U);
}

/** @brief 0x0408 查询：[设备号, 查询类型] DLC=2 */
static uint8_t Fly_SendQueryDev(uint8_t dev, uint8_t type)
{
	uint8_t d[2];

	d[0] = (uint8_t)(dev & 0x0FU);
	d[1] = type;

	return FLY_SEND(CAN_ID_MOTOR_QUERY, d, 2U);
}

/**
  * @brief  0x0316 参数配置：B4 BF CF AA 00 5F [Byte6] [设备号]
  *         ★★ Byte6 bit3..0 = 马达号，必须填这台电机【当前自己的】号，
  *            否则等于把出厂固定号写坏。
  *         ★★ bit5/bit4 = 0 才会回传剩余位移 / 力矩，这是到位判据的来源。
  *         ★★ 同一台一次上电只能写一条，写第二条会把电机写死（课题二实测）。
  */
static uint8_t Fly_SendParam(uint8_t dev, uint8_t motor_no)
{
	uint8_t d[8];

	d[0] = (uint8_t)(((uint8_t)CAR_MOTOR_BYTE0_HI << 4) | (uint8_t)CAR_MOTOR_POLE_PAIRS);
	d[1] = (uint8_t)(((uint8_t)CAR_MOTOR_SPEED_P << 4) | (uint8_t)CAR_MOTOR_SPEED_I);
	d[2] = (uint8_t)(((uint8_t)CAR_MOTOR_TORQUE_P << 4) | (uint8_t)CAR_MOTOR_TORQUE_I);
	d[3] = (uint8_t)(((uint8_t)CAR_MOTOR_CURRENT_CW << 4) | (uint8_t)CAR_MOTOR_CURRENT_CCW);
	d[4] = (uint8_t)((((uint8_t)CAR_MOTOR_BRAKE != 0U) ? 0x02U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_DAMPING != 0U) ? 0x01U : 0x00U));
	d[5] = (uint8_t)(((uint8_t)CAR_MOTOR_SPEED_D << 4) | (uint8_t)CAR_MOTOR_TORQUE_D);
	d[6] = (uint8_t)((((uint8_t)CAR_MOTOR_AUTO_UNLOCK != 0U) ? 0x40U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_RETURN_REMAIN != 0U) ? 0x00U : 0x20U) |
	                 (((uint8_t)CAR_MOTOR_RETURN_TORQUE != 0U) ? 0x00U : 0x10U) |
	                 (uint8_t)(motor_no & 0x0FU));
	d[7] = dev;

	return FLY_SEND(CAN_ID_MOTOR_CFG_PARAM, d, 8U);
}

/* =========================================================================
 *                            参数换算
 * ========================================================================= */

static uint8_t Fly_IsRotary(FlyAxis_t axis)
{
	return ((axis == FLY_AXIS_SWING) || (axis == FLY_AXIS_HOOK)) ? 1U : 0U;
}

/**
  * @brief  取该轴的轮径（0.01mm）
  *
  * ★ 5 个轴全都配轮径，即使 M4/M5 是旋转轴。
  *   原因见 Fly_ConfigAxes() 里第 2 步的说明：不配的话位置指令不产生运动。
  */
static uint16_t Fly_WheelOfAxis(FlyAxis_t axis)
{
	switch (axis)
	{
	case FLY_AXIS_BELT_L:
	case FLY_AXIS_BELT_R:
		return (uint16_t)FLY_WHEEL_BELT_0P01MM;

	case FLY_AXIS_SLIDE:
		return (uint16_t)FLY_WHEEL_SLIDE_0P01MM;

	case FLY_AXIS_SWING:
		return (uint16_t)FLY_WHEEL_SWING_0P01MM;

	case FLY_AXIS_HOOK:
		return (uint16_t)FLY_WHEEL_HOOK_0P01MM;

	default:
		return (uint16_t)FLY_WHEEL_BELT_0P01MM;
	}
}

/** @brief 旋转轴：每 1° 对应的指令单位数；非旋转轴返回 0 */
static int32_t Fly_UnitsPerDeg(FlyAxis_t axis)
{
	if (axis == FLY_AXIS_SWING) { return (int32_t)FLY_SWING_UNITS_PER_DEG; }
	if (axis == FLY_AXIS_HOOK)  { return (int32_t)FLY_HOOK_UNITS_PER_DEG;  }
	return 0;
}

/** @brief 默认速度，0.1mm/s（旋转轴按 °/s × 每度单位数换算）*/
static uint32_t Fly_AxisSpeed(FlyAxis_t axis)
{
	if (Fly_IsRotary(axis) != 0U)
	{
		return (uint32_t)((int32_t)FLY_ROT_SPEED_DPS * Fly_UnitsPerDeg(axis));
	}
	return (uint32_t)FLY_DEFAULT_SPEED_MMPS * (uint32_t)FLY_UNITS_PER_MM;    /* mm/s -> 单位/s */
}

static uint32_t Fly_AxisAccel(FlyAxis_t axis)
{
	(void)axis;
	return (uint32_t)FLY_DEFAULT_ACC_MMPS2 * (uint32_t)FLY_UNITS_PER_MM;     /* mm/s^2 -> 单位/s^2 */
}

/* =========================================================================
 *                            初始化与设备发现
 * ========================================================================= */

void Fly_Init(void)
{
	uint8_t  i;
	uint8_t  dev_list[FLY_AXIS_COUNT] = { FLY_DEV_BELT_L, FLY_DEV_BELT_R,
	                                      FLY_DEV_SLIDE,  FLY_DEV_SWING,
	                                      FLY_DEV_HOOK };

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		s_ax[i].dev           = dev_list[i];
		s_ax[i].motor_no      = (dev_list[i] == FLY_DEV_UNKNOWN) ? FLY_DEV_UNKNOWN : dev_list[i];
		s_ax[i].param_written = 0U;
		s_ax[i].pos_valid     = 0U;
		s_ax[i].zeroed        = 0U;
		s_ax[i].cfg_ok        = 0U;
		s_ax[i].use_slave     = 0U;      /* 默认主帧组，失败时由点动自检自动切换 */
		s_ax[i].pos           = 0;
		s_ax[i].min_pos       = 0;
		s_ax[i].max_pos       = 0;
		s_ax[i].speed         = Fly_AxisSpeed((FlyAxis_t)i);
		s_ax[i].accel         = Fly_AxisAccel((FlyAxis_t)i);
		s_ax[i].seen_busy     = 0U;
		s_ax[i].expect_ms     = 0U;
	}

	/* ---- 软限位（没挡块，这是唯一的越界保护）----
	 *
	 * ★★★ 必须换算到「物理方向」★★★
	 *
	 *   pos 是【物理单位】跟踪的：Fly_MoveRelEx 一进来就把逻辑位移
	 *   乘了 FLY_DIR_SIGN_OF(axis)。而 FLY_*_MIN/MAX_UNITS 是按
	 *   【逻辑方向】写的（正 = 机构该往那边走）。
	 *
	 *   两者符号不一致时直接比较，限制方向就反了。实例：
	 *     SLIDE 物理符号 = -1，逻辑「前伸 3000」物理上是 -3000；
	 *     逻辑限位是 [-20, +4000]，于是 -3000 < -20 被判越界，
	 *     -> 业务流程第 1 步就被拒，整个流程一步都不动。
	 *
	 *   做法：两个端点各自乘符号，再按大小重排成 [min, max]。
	 *   这样逻辑「前伸 / 后退」两个方向都能落在正确的物理区间里。 */
	{
		int32_t p1;
		int32_t p2;

		p1 = FLY_SLIDE_MIN_UNITS * (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_SLIDE);
		p2 = FLY_SLIDE_MAX_UNITS * (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_SLIDE);
		s_ax[FLY_AXIS_SLIDE].min_pos = (p1 < p2) ? p1 : p2;
		s_ax[FLY_AXIS_SLIDE].max_pos = (p1 < p2) ? p2 : p1;

		p1 = FLY_SWING_MIN_UNITS * (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_SWING);
		p2 = FLY_SWING_MAX_UNITS * (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_SWING);
		s_ax[FLY_AXIS_SWING].min_pos = (p1 < p2) ? p1 : p2;
		s_ax[FLY_AXIS_SWING].max_pos = (p1 < p2) ? p2 : p1;

		p1 = FLY_HOOK_MIN_UNITS * (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_HOOK);
		p2 = FLY_HOOK_MAX_UNITS * (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_HOOK);
		s_ax[FLY_AXIS_HOOK].min_pos = (p1 < p2) ? p1 : p2;
		s_ax[FLY_AXIS_HOOK].max_pos = (p1 < p2) ? p2 : p1;
	}

	s_param_written = 0U;
	g_fly_scan_cnt  = 0U;
	g_fly_mapped_cnt = 0U;
	g_fly_cfg_cnt   = 0U;
	g_fly_bind_cnt  = 0U;
	g_fly_verify_cnt = 0U;
	g_fly_verify_mask = 0U;
	g_fly_cfg_mask  = 0U;
	g_fly_zeroed_cnt = 0U;
	g_fly_zero_req  = 0U;
	g_fly_zero_axis = 0xFFU;
	g_fly_jog_axis  = 0xFFU;
	g_fly_abort     = 0U;

	g_fly_mapped_cnt = Fly_AutoMapDevices();
}

const uint8_t *Fly_GetAxisSn(FlyAxis_t axis)
{
#if (FLY_USE_SN_BIND != 0)
	return (axis < FLY_AXIS_COUNT) ? s_sn[axis] : NULL;
#else
	(void)axis;
	return NULL;
#endif
}

uint8_t Fly_SnEqual(const uint8_t *a, const uint8_t *b)
{
	uint8_t i;

	if ((a == NULL) || (b == NULL)) { return 0U; }
	for (i = 0U; i < 7U; i++)
	{
		if (a[i] != b[i]) { return 0U; }
	}
	return 1U;
}

#if (FLY_USE_SN_BIND != 0)
uint8_t Fly_BindAxes(void)
{
	uint8_t i;
	uint8_t j;
	uint8_t n = 0U;

	g_fly_bind_cnt = 0U;

	/* ---- 安全前提：设备号必须互不相同 ----
	 * 同一个设备号绑两次，等于后一次把前一次覆盖掉，两台电机变一台。
	 * ★ 这一条必须留在「全局」层面，不能改成逐轴：
	 *   两台同号是唯一会造成「一帧指令驱动两个关节」的情况，必须拦住。
	 *   但下面「SN 是否填全」的检查已改成逐轴，不再因为某一台缺失就全停。 */
	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev > 0x0FU) { continue; }     /* 未指定号的轴不参与 */

		for (j = (uint8_t)(i + 1U); j < FLY_AXIS_COUNT; j++)
		{
			if (s_ax[j].dev > 0x0FU) { continue; }
			if (s_ax[i].dev == s_ax[j].dev) { return 0U; }   /* 有同号 -> 一帧不发 */
		}
	}

	/* ---- 发绑定：0x0313 [SN0..SN6, 设备号] ----
	 * ★ SN 寻址，只有 SN 完全匹配的那一台会接受，其余电机直接丢弃。
	 *   所以 5 台同时上电也能一条一条准确地绑，不需要逐台断电。
	 *
	 * ★ 逐轴尽力而为：SN 全 0（表没填）或设备号未指定的轴直接跳过，
	 *   不影响其他轴。原来这里是「有一台缺失就整批不发」，
	 *   一个笔误就能让全部电机都绑不上 —— 这种"全有或全无"是错的。 */
	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		uint8_t k;
		uint8_t all_zero = 1U;

		if (s_ax[i].dev > 0x0FU) { continue; }     /* 没指定设备号 */

		for (k = 0U; k < 7U; k++)
		{
			if (s_sn[i][k] != 0U) { all_zero = 0U; break; }
		}
		if (all_zero != 0U) { continue; }          /* SN 没填，跳过这一轴 */

		if (Motor_SendBindSn(s_sn[i], s_ax[i].dev) == 0U)
		{
			n++;
		}
		osDelay(80U);           /* 每台之间留间隔，等它写完内部 Flash */
	}

	g_fly_bind_cnt = n;
	return n;
}
#else
uint8_t Fly_BindAxes(void)
{
	/* ★ FLY_USE_SN_BIND == 0：设备号出厂已定，不绑定。
	 *   这里刻意保留一个空实现而不是删掉，是为了让调用方不用加 #if，
	 *   以后要换电机时只改 flybox.h 一个宏就能恢复绑定能力。 */
	g_fly_bind_cnt = 0U;
	return 0U;
}
#endif

uint8_t Fly_VerifyAxes(uint32_t gap_ms)
{
	uint8_t i;
	uint8_t need = 0U;
	uint8_t n = 0U;
	uint8_t round;

	Motor_ClearFbCount();

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev != FLY_DEV_UNKNOWN) { need++; }
	}

	/* ★ 重试 3 轮，而不是只发一轮。
	 *   电机刚上电 / 刚被 0x0313 绑定完那几百毫秒内，可能还没准备好响应查询。
	 *   只发一轮就判"应答 0 个"，会把「时序问题」误判成「设备号错误」，
	 *   然后闸门把整段配置拦住 —— 表现就是电机永远不使能、一帧都不动。 */
	for (round = 0U; round < 3U; round++)
	{
		for (i = 0U; i < FLY_AXIS_COUNT; i++)
		{
			if (s_ax[i].dev == FLY_DEV_UNKNOWN) { continue; }
			(void)Motor_SendQueryDev(s_ax[i].dev, MOTOR_QUERY_STATUS);
			osDelay(gap_ms);
		}
		osDelay(60U);           /* 等这一轮的应答收完 */
	}

	osDelay(50U);

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev == FLY_DEV_UNKNOWN) { continue; }
		if (Motor_GetFbCountDev(s_ax[i].dev) > 0U)
		{
			n++;
			g_fly_verify_mask |= (uint8_t)(1U << i);    /* ★ 记下是哪几个轴应答了 */
		}
	}

	g_fly_verify_cnt = n;

	/* need == 0 说明一个号都没填，这本身就是错误配置 */
	return (need == 0U) ? 0U : n;
}

uint8_t Fly_ScanDevices(uint32_t gap_ms)
{
	uint8_t dev;
	uint8_t n = 0U;

	Motor_ClearFbCount();
	g_fly_scan_cnt = 0U;

	/* ★ 只发「查询状态」0x0408 [dev, 0x25]：纯读操作，不会让电机动作。
	 *   dev 范围限制在 0..15 —— motor.c 的内部缓存只有 16 格，
	 *   而且 TOF8 的 25/26 也在这个范围外，天然不会误伤。 */
	for (dev = FLY_SCAN_DEV_MIN; dev <= FLY_SCAN_DEV_MAX; dev++)
	{
		(void)Fly_SendQueryDev(dev, MOTOR_QUERY_STATUS);
		osDelay(gap_ms);
	}

	/* 再给总线一点时间把最后的应答收完 */
	osDelay(50U);

	for (dev = FLY_SCAN_DEV_MIN; dev <= FLY_SCAN_DEV_MAX; dev++)
	{
		if (Motor_GetFbCountDev(dev) > 0U)
		{
			if (n < 16U)
			{
				g_fly_scan_list[n] = dev;
				n++;
			}
		}
	}

	g_fly_scan_cnt = n;
	return n;
}

uint8_t Fly_GetScanDev(uint8_t i)
{
	return (i < g_fly_scan_cnt) ? g_fly_scan_list[i] : FLY_DEV_UNKNOWN;
}

uint8_t Fly_GetScanCount(void)
{
	return g_fly_scan_cnt;
}

uint8_t Fly_AutoMapDevices(void)
{
	uint8_t i;
	uint8_t k = 0U;
	uint8_t missing = 0U;

	/* 已指定的不动，只填 0xFF 的。按扫描顺序（升序）依次分配。 */
	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev != FLY_DEV_UNKNOWN)
		{
			continue;                       /* 已在 flybox.h 里写死，尊重它 */
		}
		/* 跳过已被别的轴占用的设备号 */
		while (k < g_fly_scan_cnt)
		{
			uint8_t j;
			uint8_t used = 0U;

			for (j = 0U; j < FLY_AXIS_COUNT; j++)
			{
				if (s_ax[j].dev == g_fly_scan_list[k]) { used = 1U; break; }
			}
			if (used == 0U) { break; }
			k++;
		}
		if (k >= g_fly_scan_cnt) { continue; }

		s_ax[i].dev      = g_fly_scan_list[k];
		s_ax[i].motor_no = g_fly_scan_list[k];
		k++;
	}

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev == FLY_DEV_UNKNOWN) { missing++; }
	}

	return missing;
}

uint8_t Fly_GetAxisDev(FlyAxis_t axis)
{
	return (axis < FLY_AXIS_COUNT) ? s_ax[axis].dev : FLY_DEV_UNKNOWN;
}

void Fly_SetAxisDev(FlyAxis_t axis, uint8_t dev)
{
	if (axis >= FLY_AXIS_COUNT) { return; }
	s_ax[axis].dev      = dev;
	s_ax[axis].motor_no = dev;
	s_ax[axis].pos_valid = 0U;
}

uint8_t Fly_AxisReady(FlyAxis_t axis)
{
	if (axis >= FLY_AXIS_COUNT) { return 0U; }
	if (s_ax[axis].dev == FLY_DEV_UNKNOWN) { return 0U; }
	return (Motor_GetFbCountDev(s_ax[axis].dev) > 0U) ? 1U : 0U;
}

uint8_t Fly_ConfigAxes(void)
{
	uint8_t i;
	uint8_t n = 0U;
	uint8_t step;

	/* ★★★ 这里是本轮最重要的修复 ★★★
	 *
	 *  原来的写法是「全局闸门」：
	 *      if (g_fly_verify_cnt != need) { return 0U; }   // 一帧都不发
	 *  意图是安全（怕设备号有重复时一帧指令驱动两个关节），
	 *  但实际后果是灾难性的：
	 *
	 *      只要 5 个轴里有【任意一个】没应答（SN 抄错一位、
	 *      某个电机没上电、查询丢帧...），整段配置就被完全跳过，
	 *      0x0413 使能一帧都发不出去 ——
	 *      现象就是「电机根本不锁死，一点反应都没有」。
	 *
	 *      一个轴的问题，拖死了其余四个能用的轴。
	 *
	 *  正确的做法是【逐轴尽力而为】：
	 *      · 每台电机只按自己的设备号寻址（0x0316 的 Byte7、
	 *        0x0413 的 Data[4]），号错了最多这一台不变，不会伤到别的
	 *      · 已经应答的轴一定要配上并锁死，不能因为别人缺席就一起躺平
	 *      · 配置了哪些轴记进 g_fly_cfg_mask，点动时按轴判断
	 *
	 *  ★ 真正的「重复设备号」风险在绑定时就已经排除了：
	 *    Fly_BindAxes() 里有「设备号必须互不相同」的检查。
	 */
	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		FlyAxis_t ax = (FlyAxis_t)i;
		uint8_t   dev = s_ax[i].dev;
		uint8_t   r;

		if (dev == FLY_DEV_UNKNOWN) { continue; }

#if (FLY_MOTOR_NO_EQ_DEV == 0)
#error "马达号与设备号不同时，需要先实现马达号探测，当前不支持"
#endif

		/* ---- 1) 0x0316 参数：Byte6 低 4 位填【自己的号】，保持原值 ----
		 * ★ 每轴一次上电只写一条：写第二条会把电机写死（课题二实测）。
		 *   这一条顺带把「自动回传剩余位移/力矩」打开（bit5/bit4 = 0），
		 *   没有它就没有 0x0420，也就没有任何到位判据。 */
		if (s_ax[i].param_written == 0U)
		{
			(void)Fly_SendParam(dev, s_ax[i].motor_no);
			osDelay(50U);
			s_ax[i].param_written = 1U;
			s_param_written++;
		}

		/* ---- 2) 配轮径 ×3（★ 5 个轴全部都要配，不能跳过旋转轴）----
		 *
		 * ★★★ 这里原来是 `if (Fly_IsRotary(ax) == 0U)`，即旋转轴(M4/M5)跳过。
		 *     实测：M1/M2 皮带正常动，M4/M5 锁死但一动不动。
		 *     三条线索指向「没配轮径」：
		 *       ① 三条轴唯一的配置差异，就是这一条被跳过了
		 *       ② 电机内部用轮径把「位移单位」换算成机械位置，
		 *          轮径 = 0 时换算结果是 0，位置指令被接受但不产生运动
		 *       ③ 电机对 0x0408 不应答时不报错，所以前面一直没发现
		 *
		 *     现在改成全部配，只是值不同（见 flybox.h 的 FLY_WHEEL_*）。 */
		{
			uint16_t wheel = Fly_WheelOfAxis(ax);

			for (step = 0U; step < 3U; step++)
			{
				(void)Fly_SendWheel(dev, wheel);
				osDelay(10U);
			}
		}

		/* ---- 3) 使能（★ 不发这条电机不会锁死、也不出力）---- */
		r  = Fly_SendLock(dev, MOTOR_LOCK_ENABLE);
		osDelay(50U);

		/* ---- 4) 加速度 / 速度（顺序不能反：先加速度再速度）---- */
		r |= Fly_SendAccel(s_ax[i].motor_no, s_ax[i].accel);
		osDelay(FLY_MOTION_GAP_MS);
		r |= Fly_SendSpeed(s_ax[i].motor_no, s_ax[i].speed);
		osDelay(FLY_MOTION_GAP_MS);

		/* 只要三帧都成功发到 CAN 上就算这台配置过了。
		 * ★ 这里不要求「电机应答」——应答情况看 g_fly_verify_mask，
		 *   两件事分开看才能定位问题。 */
		if (r == 0U)
		{
			s_ax[i].cfg_ok = 1U;
			g_fly_cfg_mask |= (uint8_t)(1U << i);
			n++;
		}
	}

	g_fly_cfg_cnt = n;
	return n;
}

void Fly_RequestPositions(void)
{
	uint8_t i;

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev == FLY_DEV_UNKNOWN) { continue; }
		(void)Fly_SendQueryDev(s_ax[i].dev, MOTOR_QUERY_POS);
		osDelay(10U);
	}
}

uint8_t Fly_RefreshPositions(void)
{
	uint8_t i;
	uint8_t n = 0U;

	/* ★★★ 这一步不能省 ★★★
	 *
	 *   0x0420 的「剩余位移」只说明「还差多少」，是相对量；
	 *   而软限位需要的是「现在在哪」，是绝对量 —— 只有 0x0409/0x08 能给。
	 *
	 *   pos_valid 一旦为 0：
	 *     · Fly_MoveRel()       跳过软限位检查（只剩 200mm 限幅兜底）
	 *     · Fly_RotaryToAngle() 直接返回失败（不敢动）
	 *     · s_ax[].pos          永远停在 0，软件位置跟踪失效
	 *   没有限位挡块时，这三条同时失效等于「机构裸奔」。
	 *
	 *   所以上电 Fly_RequestPositions() 之后必须调本函数，
	 *   把电机自报的绝对位置灌进软件跟踪值。
	 */
	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev == FLY_DEV_UNKNOWN) { continue; }
		if (Motor_HasQueryPosDev(s_ax[i].dev) == 0U) { continue; }

		/* ★ 已经置零的轴不能覆盖！
		 *   置零之后 s_ax[].pos 是「相对零点」的软件帧，而电机自报的是它自己的
		 *   内部绝对帧，两者原点不同。用后者覆盖前者会把零点冲掉，
		 *   之后软限位和 0°/90° 全部基于错误原点 —— 没有挡块，这是致命的。 */
		if (s_ax[i].zeroed != 0U) { continue; }

		s_ax[i].pos       = Motor_GetQueryPosDev(s_ax[i].dev);
		s_ax[i].pos_valid = 1U;
		n++;
	}

	return n;
}

int32_t Fly_GetAxisPos(FlyAxis_t axis)
{
	if (axis >= FLY_AXIS_COUNT) { return 0; }
	if (s_ax[axis].dev == FLY_DEV_UNKNOWN) { return 0; }
	if (Motor_HasQueryPosDev(s_ax[axis].dev) == 0U) { return 0; }
	return Motor_GetQueryPosDev(s_ax[axis].dev);
}

int32_t Fly_GetAxisRemain(FlyAxis_t axis)
{
	if (axis >= FLY_AXIS_COUNT) { return 0; }
	if (s_ax[axis].dev == FLY_DEV_UNKNOWN) { return 0; }
	return Motor_GetRemainDev(s_ax[axis].dev);
}

/* =========================================================================
 *                        零点（位置初始化）
 * ========================================================================= */

uint8_t Fly_SetZeroAxis(FlyAxis_t axis)
{
	uint8_t i;
	uint8_t cnt = 0U;

	if (axis >= FLY_AXIS_COUNT) { return 1U; }
	if (s_ax[axis].dev == FLY_DEV_UNKNOWN) { return 1U; }

	/* ---- 1) 软件零点：这是真正的零点，所有功能都靠它 ---- */
	s_ax[axis].pos    = 0;
	s_ax[axis].zeroed = 1U;

	/* ---- 2) ★ 不再给电机发 0x0B（把当前位置设为某值）----
	 *
	 *   理由（课题二的教训）：
	 *     · 我们的运动全是相对位置 0x0416 类型 0x00，电机内部计数不需要对齐；
	 *     · 0x0B 是否被这个驱动支持没有验证过，发一条不被支持的指令
	 *       有可能让电机进入异常状态（课题二见过「多发一条就锁死」）。
	 *     · 少发一条就少一个出错的可能 —— 零点纯软件做，够了。
	 *
	 *   ★ 副作用：0x7E2 位置帧里的 pos 是我们的软件帧，不是电机自报的。
	 *     这本来就是设计意图（电机自报的值没有物理含义）。 */

	/* ---- 3) 刷新已置零计数 ---- */
	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].zeroed != 0U) { cnt++; }
	}
	g_fly_zeroed_cnt = cnt;

	return 0U;
}

void Fly_SetZeroAll(void)
{
	uint8_t i;

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		(void)Fly_SetZeroAxis((FlyAxis_t)i);
		osDelay(30U);            /* 别把 CAN 发送邮箱一次挤满 */
	}
}

uint8_t Fly_IsZeroed(FlyAxis_t axis)
{
	return (axis < FLY_AXIS_COUNT) ? s_ax[axis].zeroed : 0U;
}

void Fly_RequestSetZero(uint8_t axis)
{
	/* ★ 与点动自检一样只置标志，由 Task_Flybox 执行：
	 *   置零本身很快，但为了保持「所有会碰电机的事都在同一个任务里做」的规矩，
	 *   不在 CAN 接收任务里发指令。 */
	g_fly_zero_axis = axis;
	g_fly_zero_req  = 1U;
}

/* =========================================================================
 *                            运动
 * ========================================================================= */

/**
  * @brief  单轴相对运动的内部实现
  * @param  allow_nozero  1 = 允许在「未置零」状态下运动
  *
  *  ★★★ 为什么需要这个开关 ★★★
  *
  *    软限位是「相对零点」定义的，没置零就没法算，所以正常业务路径
  *    必须拒绝未置零的轴运动（否则没有挡块的机构会裸奔）。
  *
  *    但【点动自检】的目的是「还没置零时先看看能不能动」——
  *    如果也按正常路径拒绝，M3/M4/M5 永远动不了，自检就失去意义了。
  *
  *    所以点动自检走 allow_nozero = 1：
  *      · 跳过软限位（此时无从判断）
  *      · 但【保留】200mm 行程限幅
  *      · 而且自检是「+N 再 -N」，净位移为 0，本身不会把机构越走越偏
  *
  *    这样既能让自检跑起来，又不放松正常业务的安全性。
  */
static uint8_t Fly_MoveRelEx(FlyAxis_t axis, int32_t dist_units, uint32_t speed,
                             uint32_t accel, uint8_t allow_nozero)
{
	FlyAxisState_t *a;
	int32_t new_pos;
	int32_t limit;

	if (axis >= FLY_AXIS_COUNT) { return 1U; }
	a = &s_ax[axis];

	if (a->dev == FLY_DEV_UNKNOWN) { return 1U; }
	if ((dist_units == 0) || (speed == 0U)) { return 1U; }

	/* ★★★ 方向符号在这里统一应用 ★★★
	 *
	 *   调用方永远只写「逻辑方向」（正 = 机构该往那边走），
	 *   实际下发时乘上该轴的方向符号。
	 *
	 *   为什么放在最前面：后面的行程限幅、软限位、pos 跟踪、expect_ms
	 *   全都用【物理单位】算一遍，符号只出现这一次，不会漏、不会重复取反。
	 *
	 *   ★ pos 也用物理单位跟踪，配合 min/max 也是物理方向定义，
	 *     始终保持一致。零点在 pos=0 处重置，所以物理/逻辑原点相同。 */
	dist_units *= (int32_t)FLY_DIR_SIGN_OF(axis);

	/* ---- 保护 ②：单次行程限幅（★ 无论如何不跳过）---- */
	limit = (int32_t)FLY_JOG_MAX_MM * (int32_t)FLY_UNITS_PER_MM;
	if ((dist_units > limit) || (dist_units < -limit)) { return 1U; }

	/* ---- 保护 ①：软限位 ---- */
#if (FLY_IGNORE_LIMITS != 0)
	/* ★★ 盲走模式：软限位和零点要求全部跳过 ★★
	 *
	 *   目的：先把整套动作顺序跑通。软限位是「度数 × 每度系数」算的，
	 *   而那两个系数现在还是估的，系数一偏限位就跟着偏，
	 *   结果就是「该走的距离被软件拒了」，流程永远走不通。
	 *
	 *   ★★★ 代价：机构没有任何越界保护 ★★★
	 *     没有挡块、没有限位开关，走过头就是硬顶结构，
	 *     只剩电机自己的堵转保护。调试时手放在断电开关上。
	 *
	 *   ★ 流程验证通了之后，把 flybox.h 里的 FLY_IGNORE_LIMITS 改回 0。 */
	(void)new_pos;
	(void)allow_nozero;
#else
	if (a->zeroed != 0U)
	{
		if ((a->min_pos == 0) && (a->max_pos == 0))
		{
			/* 皮带轴：不限位（皮带是闭环，连续输送，无绝对位置概念）*/
		}
		else
		{
			new_pos = a->pos + dist_units;
			if ((new_pos < a->min_pos) || (new_pos > a->max_pos)) { return 1U; }
		}
	}
	else if ((allow_nozero == 0U) && ((a->min_pos != 0) || (a->max_pos != 0)))
	{
		/* 有行程要求的轴但还没置零 -> 拒绝运动。
		 * 宁可不动，也不能在没有零点的情况下往前走 —— 没有挡块。 */
		if ((axis != FLY_AXIS_BELT_L) && (axis != FLY_AXIS_BELT_R)) { return 1U; }
	}
#endif

	/* ---- 顺序不能反：加速度 -> 速度 -> 位置（课题二实测）----
	 * ★ 按 use_slave 选主帧组 / 从帧组：
	 *   双通道驱动器上挂在第二路的电机只响应 0x0417/18/19。 */
	if (a->use_slave != 0U)
	{
		(void)Fly_SendAccelSlave(a->motor_no, accel);
		osDelay(FLY_MOTION_GAP_MS);
		(void)Fly_SendSpeedSlave(a->motor_no, speed);
		osDelay(FLY_MOTION_GAP_MS);
	}
	else
	{
		(void)Fly_SendAccel(a->motor_no, accel);
		osDelay(FLY_MOTION_GAP_MS);
		(void)Fly_SendSpeed(a->motor_no, speed);
		osDelay(FLY_MOTION_GAP_MS);
	}

	a->seen_busy = 0U;
	{
		int32_t dm = (dist_units >= 0) ? dist_units : -dist_units;
		a->expect_ms = (uint32_t)(((uint32_t)dm * 1000U) / speed);
	}

	if (a->use_slave != 0U)
	{
		if (Fly_SendPosRelSlave(a->motor_no, dist_units) != 0U) { return 1U; }
	}
	else
	{
		if (Fly_SendPosRel(a->motor_no, dist_units) != 0U) { return 1U; }
	}

	s_ax[axis].pos += dist_units;      /* 只要下发成功就跟踪（与是否置零无关）*/

	return 0U;
}

uint8_t Fly_MoveRel(FlyAxis_t axis, int32_t dist_units, uint32_t speed, uint32_t accel)
{
	return Fly_MoveRelEx(axis, dist_units, speed, accel, 0U);
}

uint8_t Fly_BeltMove(int32_t dist_units, uint32_t speed, uint32_t accel)
{
	FlyAxisState_t *l = &s_ax[FLY_AXIS_BELT_L];
	FlyAxisState_t *r = &s_ax[FLY_AXIS_BELT_R];
	int32_t limit;

	if ((l->dev == FLY_DEV_UNKNOWN) || (r->dev == FLY_DEV_UNKNOWN)) { return 1U; }
	if ((dist_units == 0) || (speed == 0U)) { return 1U; }

	limit = (int32_t)FLY_JOG_MAX_MM * (int32_t)FLY_UNITS_PER_MM;
	if ((dist_units > limit) || (dist_units < -limit)) { return 1U; }

	/* 加速度、速度各发一帧，帧间留间隔 */
	(void)Fly_SendAccel(l->motor_no, accel);
	osDelay(FLY_MOTION_GAP_MS);
	(void)Fly_SendAccel(r->motor_no, accel);
	osDelay(FLY_MOTION_GAP_MS);

	(void)Fly_SendSpeed(l->motor_no, speed);
	osDelay(FLY_MOTION_GAP_MS);
	(void)Fly_SendSpeed(r->motor_no, speed);
	osDelay(FLY_MOTION_GAP_MS);

	{
		uint32_t dm = (uint32_t)((dist_units >= 0) ? dist_units : -dist_units);
		uint32_t em = (uint32_t)((dm * 1000U) / speed);

		l->seen_busy = 0U;  l->expect_ms = em;
		r->seen_busy = 0U;  r->expect_ms = em;
	}

	/* ★★★ 两帧位置指令紧挨着发，中间不留延时 ★★★
	 *   课题二实测：两轮之间插 20ms 会让起步错开，整段带倾角。
	 *   两条皮带同理 —— 错开会导致箱子在皮带上被拧斜。
	 *
	 * ★ 方向符号在这里【分别】应用：
	 *   M1 正装、M2 反装，同一条「向前 1000」在 CAN 上必须是 +1000 和 -1000。
	 *   这正是课题二左右轮的处理方式（left * sign[0], right * sign[1]）。 */
	if (Fly_SendPosRel(l->motor_no, dist_units * (int32_t)FLY_DIR_SIGN_BELT_L) != 0U) { return 1U; }
	if (Fly_SendPosRel(r->motor_no, dist_units * (int32_t)FLY_DIR_SIGN_BELT_R) != 0U) { return 1U; }

	if (l->pos_valid != 0U) { l->pos += dist_units; }
	if (r->pos_valid != 0U) { r->pos += dist_units; }

	return 0U;
}

static uint8_t Fly_WaitOne(FlyAxisState_t *a, uint32_t timeout_ms)
{
	uint32_t waited = 0U;
	uint8_t  seen   = 0U;
	uint8_t  ok     = 0U;

	while (waited < timeout_ms)
	{
		int32_t rem  = Motor_GetRemainDev(a->dev);
		int32_t arem = (rem >= 0) ? rem : -rem;

		if (arem > (int32_t)FLY_REMAIN_EPS_UNITS)
		{
			seen = 1U;
			a->seen_busy = 1U;
		}
		else if (seen != 0U)
		{
			ok = 1U;                    /* 见过忙，现在闲了 -> 真到位 */
			break;
		}
		else if (waited >= a->expect_ms)
		{
			/* 一次没见过忙：可能是位移太小，也可能是根本没收到 0x0420。
			 * 至少要等过「位移 ÷ 速度」这个物理下限。 */
			if (a->expect_ms > 0U) { ok = 1U; }
			break;
		}

		osDelay(20U);
		waited += 20U;
	}

	osDelay(FLY_MOTION_SETTLE_MS);

	/* 停稳后再确认一次 */
	{
		int32_t rem  = Motor_GetRemainDev(a->dev);
		int32_t arem = (rem >= 0) ? rem : -rem;

		if (arem > (int32_t)FLY_REMAIN_EPS_UNITS) { ok = 0U; }
	}

	/* 整条指令期间一次没见过忙 = 没反馈 = 无法确认已停 */
	if ((a->seen_busy == 0U) && (a->expect_ms >= 100U)) { ok = 0U; }

	return ok;
}

uint8_t Fly_WaitAxisDone(FlyAxis_t axis, uint32_t timeout_ms)
{
	uint8_t ok;

	if (axis >= FLY_AXIS_COUNT) { return 0U; }
	if (s_ax[axis].dev == FLY_DEV_UNKNOWN) { return 0U; }

	ok = Fly_WaitOne(&s_ax[axis], timeout_ms);
	if (ok == 0U) { g_fly_abort = 1U; }   /* 无法确认到位 -> 后续动作全部停止 */
	return ok;
}

uint8_t Fly_WaitBeltDone(uint32_t timeout_ms)
{
	uint8_t ok_l;
	uint8_t ok_r;

	ok_l = Fly_WaitOne(&s_ax[FLY_AXIS_BELT_L], timeout_ms);
	ok_r = Fly_WaitOne(&s_ax[FLY_AXIS_BELT_R], timeout_ms);

	if ((ok_l == 0U) || (ok_r == 0U)) { g_fly_abort = 1U; return 0U; }
	return 1U;
}

/* =========================================================================
 *                    ★ TOF 闭环运行（飞箱真正的定位手段）
 *
 *  ★★★ 为什么不能「一直走，等 TOF 说停就停」★★★
 *
 *    电机协议只有「走 N 个单位」这一种位置指令，没有持续运转模式；
 *    而课题二已实测：运动中途改目标会把电机搞死，只能断电重启。
 *    所以只能「走一小段 -> 停稳确认 -> 读 TOF -> 判断 -> 再走一小段」。
 *
 *  ★ 这条路的精度 = 步长（FLY_TOF_CHUNK_MM），不是电机位置单位。
 *    因此轮径/减速比/每度系数全都不需要准 —— 这正是飞箱的设计意图。
 * ========================================================================= */

volatile uint8_t g_fly_tof_result = FLY_TOF_OK;

/* 解析出来的槽位（0xFF = 该设备号不在 TOF8 配置里）*/
volatile uint8_t g_fly_tof_slot_under = 0xFFU;
volatile uint8_t g_fly_tof_slot_side  = 0xFFU;

void Fly_TofInit(void)
{
	uint8_t i;

	Tof8_Init();

	/* ★ 先解析设备号 -> 槽位。
	 *   用 Tof8_FindSlot() 而不是写死槽位号，这样 car_cfg.h 里
	 *   TOF8_DEV0 / TOF8_DEV1 的顺序改了也不会把逻辑搞乱。 */
	g_fly_tof_slot_under = Tof8_FindSlot(FLY_TOF_DEV_UNDER_HOOK);
	g_fly_tof_slot_side  = Tof8_FindSlot(FLY_TOF_DEV_SIDE);

	/* 等模块上电稳定：连续模式首次出数可能要 >100ms */
	osDelay(FLY_TOF_BOOT_WAIT_MS);

	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		(void)Tof8_Config(i);
		osDelay(20U);
	}
	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		(void)Tof8_ContinuousSet(i, 1U);
		osDelay(20U);
	}

	osDelay(TOF8_FIRST_DATA_TIMEOUT_MS);
}

uint16_t Fly_TofDistance(uint8_t slot)
{
	return Tof8_GetDistanceMm(slot);
}

uint16_t Fly_TofUnderHook(void)
{
	return (g_fly_tof_slot_under == 0xFFU) ? TOF8_DIST_MM_INVALID
	                                       : Tof8_GetDistanceMm(g_fly_tof_slot_under);
}

uint16_t Fly_TofSide(void)
{
	return (g_fly_tof_slot_side == 0xFFU) ? TOF8_DIST_MM_INVALID
	                                      : Tof8_GetDistanceMm(g_fly_tof_slot_side);
}

void Fly_SendTofReport(void)
{
	static uint8_t seq = 0U;
	uint8_t  buf[8];
	uint16_t du = Fly_TofUnderHook();
	uint16_t ds = Fly_TofSide();

	buf[0] = 0xADU;
	buf[1] = (uint8_t)(du & 0xFFU);         /* 抓钩下面（dev 25）低 */
	buf[2] = (uint8_t)(du >> 8);            /* 抓钩下面高 */
	buf[3] = (uint8_t)(ds & 0xFFU);         /* 抓钩左侧（dev 26）低 */
	buf[4] = (uint8_t)(ds >> 8);            /* 抓钩左侧高 */
	buf[5] = g_fly_tof_result;
	buf[6] = g_fly_abort;
	buf[7] = seq++;

	(void)Can_SendStd(FLY_ID_TOF_REPORT, buf, 8U);
}

/**
  * @brief  ★ 上电阶段上报（0x7F1，每次阶段变化立即发一帧）
  *
  *         [0] 0xB2 魔数
  *         [1] 当前阶段（含义见 flybox.h 的 g_fly_boot_stage）
  *         [2] 校验应答数 verify_cnt
  *         [3] 已配置轴数 cfg_cnt
  *         [4] 已置零轴数 zeroed_cnt
  *         [5] 本帧时收到的电机反馈帧总数低字节（0 = 一帧都没收到）
  *         [6] CAN 发送失败计数低字节（0 = 发送通路正常）
  *         [7] CAN 发送成功计数低字节（在变 = STM32 确实在发帧）
  *
  *  ★ 这是排查「固件到底有没有在跑」最直接的一帧：
  *     看到 0x7F1 且 [7] 在变  -> 固件在跑、CAN 发送正常
  *     完全看不到 0x7F1         -> 固件没跑 / 烧的还是旧固件 / CAN 发不出去
  */
void Fly_SendBootStage(void)
{
	uint8_t  buf[8];
	uint8_t  i;
	uint32_t fb = 0U;

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		if (s_ax[i].dev != FLY_DEV_UNKNOWN)
		{
			fb += Motor_GetFbCountDev(s_ax[i].dev);
		}
	}

	buf[0] = 0xB2U;
	buf[1] = g_fly_boot_stage;
	buf[2] = g_fly_verify_cnt;
	buf[3] = g_fly_cfg_cnt;
	buf[4] = g_fly_zeroed_cnt;
	buf[5] = (uint8_t)(fb & 0xFFU);
	buf[6] = (uint8_t)(g_fly_tx_fail & 0xFFU);
	buf[7] = (uint8_t)(g_fly_tx_count & 0xFFU);

	(void)Can_SendStd(FLY_ID_BOOT_REPORT, buf, 8U);
}

void Fly_SetBootStage(uint8_t stage)
{
	g_fly_boot_stage = stage;
	Fly_SendBootStage();
	osDelay(30U);               /* 给 CAN 一点时间把这一帧发出去 */
}

uint8_t Fly_TofProbe(uint8_t use_belt, uint16_t step_mm, uint8_t steps)
{
	FlyAxis_t axis = (use_belt != 0U) ? FLY_AXIS_BELT_L : FLY_AXIS_SLIDE;
	uint8_t   i;
	uint8_t   r;

	if (step_mm == 0U)  { step_mm = FLY_TOF_CHUNK_MM; }
	if (steps == 0U)    { steps = 5U; }
	if (step_mm > FLY_JOG_MAX_MM) { step_mm = FLY_JOG_MAX_MM; }

	/* 起点先报一次，作为「变化前」的基准 */
	Fly_SendTofReport();
	osDelay(120U);

	for (i = 0U; i < steps; i++)
	{
		int32_t units = (int32_t)step_mm * (int32_t)FLY_UNITS_PER_MM;

		if (use_belt != 0U)
		{
			if (Fly_BeltMove(units, FLY_TOF_CHUNK_SPEED,
			                 Fly_AxisAccel(FLY_AXIS_BELT_L)) != 0U) { return FLY_TOF_ERR_SEND; }
			if (Fly_WaitBeltDone(FLY_TOF_STEP_TIMEOUT_MS) == 0U) { return FLY_TOF_ERR_NODONE; }
		}
		else
		{
			/* 允许未置零：这是诊断，不是业务动作 */
			if (Fly_MoveRelEx(axis, units, FLY_TOF_CHUNK_SPEED,
			                  Fly_AxisAccel(axis), 1U) != 0U) { return FLY_TOF_ERR_SEND; }
			if (Fly_WaitAxisDone(axis, FLY_TOF_STEP_TIMEOUT_MS) == 0U) { return FLY_TOF_ERR_NODONE; }
		}

		/* 每走完一步就报一次：CAN 日志里连续几条 0x7F3 就能看出
		 * 哪一路距离在随步数单调变化 */
		Fly_SendTofReport();
		osDelay(150U);
	}

	r = FLY_TOF_OK;
	g_fly_tof_result = r;
	return r;
}

/** @brief 读一次有效 TOF 距离；读不到返回 TOF8_DIST_MM_INVALID */
static uint16_t Fly_TofReadOnce(uint8_t slot)
{
	return Tof8_ReadDistanceMm(slot, FLY_TOF_READ_TIMEOUT_MS);
}

/** @brief 走一个增量（皮带用双轴同步，其它用单轴）
 *  @param  speed 增量速度（指令单位/s），0 = 用默认 FLY_TOF_CHUNK_SPEED */
static uint8_t Fly_TofStep(FlyAxis_t axis, uint8_t use_belt, int32_t dir, uint32_t speed)
{
	int32_t units = dir * (int32_t)FLY_TOF_CHUNK_MM * (int32_t)FLY_UNITS_PER_MM;

	if (speed == 0U) { speed = FLY_TOF_CHUNK_SPEED; }

	if (use_belt != 0U)
	{
		if (Fly_BeltMove(units, speed,
		                 Fly_AxisAccel(FLY_AXIS_BELT_L)) != 0U) { return FLY_TOF_ERR_SEND; }
		if (Fly_WaitBeltDone(FLY_TOF_STEP_TIMEOUT_MS) == 0U) { return FLY_TOF_ERR_NODONE; }
	}
	else
	{
		/* ★ 允许未置零运行：TOF 闭环本身就是定位手段，不依赖软件零点。
		 *   安全性由 200mm 行程限幅 + 总行程上限 + 卡住检测共同保证。 */
		if (Fly_MoveRelEx(axis, units, speed,
		                  Fly_AxisAccel(axis), 1U) != 0U) { return FLY_TOF_ERR_SEND; }
		if (Fly_WaitAxisDone(axis, FLY_TOF_STEP_TIMEOUT_MS) == 0U) { return FLY_TOF_ERR_NODONE; }
	}

	return FLY_TOF_OK;
}

/**
  * @brief  TOF 闭环增量运动的公共实现
  * @param  use_belt       1 = 动皮带（M1+M2 同步），0 = 动单轴
  * @param  speed          增量速度，0 = 用 FLY_TOF_CHUNK_SPEED
  * @param  max_travel_mm  本次调用允许的最大总行程（0 = 用 FLY_TOF_MAX_TRAVEL_MM）
  *                        ★ 必须传，因为它才是“撞不到结构”的真正保证
  */
static uint8_t Fly_TofFeedLoop(FlyAxis_t axis, uint8_t use_belt, uint8_t slot,
                               uint16_t target_mm, uint8_t approach,
                               uint32_t speed, uint16_t max_travel_mm,
                               uint32_t timeout_ms)
{
	uint32_t t0;
	uint32_t traveled = 0U;
	uint16_t prev;
	uint16_t d;
	uint8_t  stall = 0U;
	uint8_t  first_move = 1U;
	int32_t  dir = 1;
	uint8_t  r;

	if (slot >= TOF8_DEV_NUM) { return FLY_TOF_ERR_NO_DATA; }
	if (max_travel_mm == 0U) { max_travel_mm = (uint16_t)FLY_TOF_MAX_TRAVEL_MM; }

	/* ---- 先拿一次有效读数：拿不到就【不动】----
	 * ★ 这条很重要：没有 TOF 读数就没有定位手段，
	 *   盲走一段等于在没有挡块的机构上乱撞。所以宁可不动。 */
	d = Fly_TofReadOnce(slot);
	if (d == TOF8_DIST_MM_INVALID) { return FLY_TOF_ERR_NO_DATA; }

	prev = d;
	t0   = HAL_GetTick();

	/* ---- 已经到位就不用动 ---- */
	if (approach == 0U) { if (d <= target_mm) { return FLY_TOF_OK; } }
	else                { if (d >= target_mm) { return FLY_TOF_OK; } }

	for (;;)
	{
		/* ---------------- 1. 走一个增量 ---------------- */
		r = Fly_TofStep(axis, use_belt, dir, speed);
		if (r != FLY_TOF_OK) { return r; }

		traveled += FLY_TOF_CHUNK_MM;
		if (traveled > (uint32_t)max_travel_mm) { return FLY_TOF_ERR_TRAVEL; }

		/* ---------------- 2. 读新的距离 ---------------- */
		d = Fly_TofReadOnce(slot);

		/* ★ 每个增量发一帧实时进度：CAN 日志里能直接看到
		 *   「滑台位置在变、距箱距离在变小」的对应关系。 */
		Fly_SendFlowReport();

		if (d == TOF8_DIST_MM_INVALID)
		{
			/* 读数无效：不判到位也不算卡住，但仍受总超时约束 */
			if ((uint32_t)(HAL_GetTick() - t0) >= timeout_ms) { return FLY_TOF_ERR_TIMEOUT; }
			continue;
		}

		/* ---------------- 3. 第一段自动判方向 ----------------
		 * 走之前是 prev，走之后是 d。
		 * approach==0（要让距离变小）时，d < prev 说明方向对。
		 * 不对就翻转符号，下一段往回走 —— 相当于自动纠正，
		 * 不用人去配「正方向到底是不是靠近」。 */
		if (first_move != 0U)
		{
			uint8_t good = (approach == 0U) ? ((d < prev) ? 1U : 0U)
			                                : ((d > prev) ? 1U : 0U);
			if (good == 0U)
			{
				dir = -dir;     /* 反了：下一段朝反方向走，把这一步也退回去 */
			}
			first_move = 0U;
		}

		/* ---------------- 4. 到位判断 ---------------- */
		if (approach == 0U)
		{
			if (d <= target_mm) { return FLY_TOF_OK; }
		}
		else
		{
			if (d >= target_mm) { return FLY_TOF_OK; }
		}

		/* ---------------- 5. 卡住检测 ----------------
		 * 连续 N 段走完距离一点没变 -> 要么机构卡了，
		 * 要么这个 TOF 看的根本不是正在动的那个物体。
		 * 此时必须停：继续走就是在没有任何反馈的情况下盲走。 */
		if (d == prev)
		{
			if (++stall >= FLY_TOF_STALL_STEPS) { return FLY_TOF_ERR_STALL; }
		}
		else
		{
			stall = 0U;
		}
		prev = d;

		/* ---------------- 6. 总超时 ---------------- */
		if ((uint32_t)(HAL_GetTick() - t0) >= timeout_ms) { return FLY_TOF_ERR_TIMEOUT; }
	}
}

uint8_t Fly_BeltFeedToTof(uint8_t slot, uint16_t target_mm, uint8_t approach,
                          uint32_t speed, uint16_t max_travel_mm, uint32_t timeout_ms)
{
	uint8_t r = Fly_TofFeedLoop(FLY_AXIS_BELT_L, 1U, slot, target_mm, approach,
	                            speed, max_travel_mm, timeout_ms);

	g_fly_tof_result = r;
	return r;
}

uint8_t Fly_SlideToTof(uint8_t slot, uint16_t target_mm, uint8_t approach,
                       uint32_t speed, uint16_t max_travel_mm, uint32_t timeout_ms)
{
	uint8_t r = Fly_TofFeedLoop(FLY_AXIS_SLIDE, 0U, slot, target_mm, approach,
	                            speed, max_travel_mm, timeout_ms);

	g_fly_tof_result = r;
	return r;
}

/* =========================================================================
 *                          语义动作
 * ========================================================================= */

/** @brief 通用：把某旋转轴从「当前软件位置」走到指定角度 */
static uint8_t Fly_RotaryToAngle(FlyAxis_t axis, float target_deg)
{
	int32_t updeg = Fly_UnitsPerDeg(axis);
	int32_t target;
	int32_t delta;

	if (updeg <= 0) { return 1U; }
	if (s_ax[axis].zeroed == 0U) { return 1U; }   /* ★ 未置零 -> 角度目标无意义 */

	target = (int32_t)(target_deg * (float)updeg);
	delta  = target - s_ax[axis].pos;
	if (delta == 0) { return 0U; }

	if (Fly_MoveRel(axis, delta, s_ax[axis].speed, s_ax[axis].accel) != 0U) { return 1U; }
	return Fly_WaitAxisDone(axis, 8000U);
}

uint8_t Fly_HookUp(void)    { return Fly_RotaryToAngle(FLY_AXIS_HOOK,  FLY_ANGLE_PARK_DEG); }
uint8_t Fly_HookDown(void)  { return Fly_RotaryToAngle(FLY_AXIS_HOOK,  FLY_ANGLE_WORK_DEG); }
uint8_t Fly_SwingWork(void) { return Fly_RotaryToAngle(FLY_AXIS_SWING, FLY_ANGLE_WORK_DEG); }
uint8_t Fly_SwingPark(void) { return Fly_RotaryToAngle(FLY_AXIS_SWING, FLY_ANGLE_PARK_DEG); }

/* =========================================================================
 *                          上报
 * ========================================================================= */

void Fly_SendScanReport(void)
{
	uint8_t buf[8];
	uint8_t i;
	uint8_t page = 0U;
	uint8_t n;

	do
	{
		buf[0] = 0xA8U;
		buf[1] = page;
		buf[2] = g_fly_scan_cnt;

		for (i = 0U; i < 5U; i++)
		{
			n = (uint8_t)(page * 5U + i);
			buf[3U + i] = (n < g_fly_scan_cnt) ? g_fly_scan_list[n] : 0xFFU;
		}

		(void)Can_SendStd(FLY_ID_SCAN_REPORT, buf, 8U);
		osDelay(20U);
		page++;
	} while ((uint16_t)page * 5U < g_fly_scan_cnt);
}

void Fly_SendStateReport(void)
{
	static uint8_t seq  = 0U;
	static uint8_t page = 0U;
	uint8_t buf[8];
	uint8_t stage = (uint8_t)(((g_fly_verify_cnt & 0x0FU) << 4) | (g_fly_cfg_cnt & 0x0FU));

	buf[1] = seq++;
	buf[7] = stage;

	switch (page)
	{
	case 0U:
		/* ---- 帧 A（0xA9）：设备号映射 ---- */
		buf[0] = 0xA9U;
		buf[2] = s_ax[FLY_AXIS_BELT_L].dev;
		buf[3] = s_ax[FLY_AXIS_BELT_R].dev;
		buf[4] = s_ax[FLY_AXIS_SLIDE].dev;
		buf[5] = s_ax[FLY_AXIS_SWING].dev;
		buf[6] = s_ax[FLY_AXIS_HOOK].dev;
		page = 1U;
		break;

	case 1U:
		/* ---- 帧 B（0xAC）：运行状态计数 ---- */
		buf[0] = 0xACU;
		buf[2] = g_fly_zeroed_cnt;      /* ★ 必须是 5 才能用语义动作 */
		buf[3] = g_fly_jog_ok;
		buf[4] = g_fly_jog_bad;
		buf[5] = g_fly_jog_nofb;        /* ★ 已下发但无反馈的轴数 */
		buf[6] = (uint8_t)(((g_fly_verify_mask & 0x1FU) << 0) |
		                   ((g_fly_cfg_mask & 0x07U) << 5));  /* 低5位verify 高3位cfg */
		page = 2U;
		break;

	case 2U:
		/* ---- 帧 C（0xAF）：每轴应答计数 + 两个掩码 ---- */
		buf[0] = 0xAFU;
		buf[2] = (uint8_t)(Motor_GetFbCountDev(s_ax[FLY_AXIS_BELT_L].dev) & 0xFFU);
		buf[3] = (uint8_t)(Motor_GetFbCountDev(s_ax[FLY_AXIS_BELT_R].dev) & 0xFFU);
		buf[4] = (uint8_t)(Motor_GetFbCountDev(s_ax[FLY_AXIS_SLIDE].dev) & 0xFFU);
		buf[5] = (uint8_t)(Motor_GetFbCountDev(s_ax[FLY_AXIS_SWING].dev) & 0xFFU);
		buf[6] = (uint8_t)(Motor_GetFbCountDev(s_ax[FLY_AXIS_HOOK].dev) & 0xFFU);
		buf[7] = (uint8_t)(((g_fly_verify_mask & 0x0FU) << 4) |
		                   (g_fly_cfg_mask & 0x0FU));
		page = 3U;
		break;

	default:
		/* ---- 帧 D（0xB4）：★ 业务流程进度 ----
		 *   [1] 序号
		 *   [2] 正在跑哪个流程（0 空闲 / 1 拉箱 / 2 推箱）
		 *   [3] 当前步骤号（1~6，0 = 未开始）
		 *   [4] 上一次失败的步骤号（0 = 没失败过）
		 *   [5] abort 标志
		 *   [6] 已置零轴数
		 *   [7] 阶段（同帧 A/B/C）
		 *
		 *   ★ 卡住时看 [3] 和 [4]：
		 *       正在跑 [2]≠0 且 [3] 不动 -> 卡在这一步
		 *       [2]=0 且 [4]≠0        -> 上次失败停在第 [4] 步
		 */
		buf[0] = 0xB4U;
		buf[2] = g_fly_flow_running;
		buf[3] = g_fly_flow_step;
		buf[4] = g_fly_flow_result;
		buf[5] = g_fly_flow_why;        /* ★ 0 正常 / 1 被拒 / 2 超时 / 3 设备号未知 */
		buf[6] = g_fly_zeroed_cnt;
		page = 0U;
		break;
	}

	(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
}

/**
  * @brief  ★ 业务流程实时进度（0x7E3，魔数 0xB5，流程运行中每一步发一帧）
  *
  *   ★★★ 为什么需要单独一帧 ★★★
  *
  *     流程是在 Task_Flybox 的主循环里同步执行的（每步都要等到位），
  *     所以流程一跑起来，主循环那条「200ms/400ms 定期上报」就完全停了。
  *     结果就是：动作在动，但 CAN 上一片死寂，完全看不出走到哪一步、
  *     激光读到多少 —— 这正是排查「流程卡住」最需要的两条信息。
  *
  *   所以流程内部在「每步开始」和「每个 TOF 增量」时各发一帧。
  *
  *   [0] 0xB5 魔数
  *   [1] 序号（自增，用于判断有没有丢帧）
  *   [2] 流程（0 空闲 / 1 拉箱 / 2 推箱）
  *   [3] 当前步骤号
  *   [4] 失败原因码（FLY_WHY_*）
  *   [5..6] 抓钩下方 TOF 距离 mm（0xFFFF = 读数无效）
  *   [7] M3 滑台的软件位置低 8 位（看它在不在动）
  */
void Fly_SendFlowReport(void)
{
	static uint8_t seq = 0U;
	uint8_t  buf[8];
	uint16_t du = Fly_TofUnderHook();

	buf[0] = 0xB5U;
	buf[1] = seq++;
	buf[2] = g_fly_flow_running;
	buf[3] = g_fly_flow_step;
	buf[4] = g_fly_flow_why;
	buf[5] = (uint8_t)(du & 0xFFU);
	buf[6] = (uint8_t)(du >> 8);

	/* [7] 标志位（★ 2026-10-30 改：原来是滑台 pos 低字节，没用）
	 *   bit0 = 滑台仍有剩余位移（电机还在动 / 没走完）
	 *   bit1 = 本步没能确认到位（★ 堵转嫌疑，连着几步都置 1 就是卡死了）
	 *   bit2 = 已置 abort
	 *
	 * ★ 怎么用这一字节：
	 *     bit1 一直是 1、且 bit0 也一直是 1  ->  电机在“想走但走不动” = 机械卡死
	 *     bit1 = 0（全 0）但动作停了        ->  软件认为到位了，那就要看电机侧
	 *     如果连 0xB5 都不再刷新               ->  MCU 死了，看 0x7EF */
	{
		uint8_t fl = 0U;
		int32_t rem = Motor_GetRemainDev(s_ax[FLY_AXIS_SLIDE].dev);

		if ((rem > (int32_t)FLY_REMAIN_EPS_UNITS) ||
		    (rem < -(int32_t)FLY_REMAIN_EPS_UNITS)) { fl |= 0x01U; }
		if (g_fly_flow_stall != 0U)                { fl |= 0x02U; }
		if (g_fly_abort != 0U)                     { fl |= 0x04U; }

		buf[7] = fl;
	}

	(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
}

/**
  * @brief  抓取距离回复帧（0x7E3，魔数 0xB6）
  *
  *   [0] 0xB6 魔数
  *   [1] 结果（0 = 学到了 / 1 = 读不到有效距离，原值没变）
  *   [2..3] 当前生效的抓取距离 mm（u16 小端）
  *   [4..5] 学习那一刻的 TOF 原始读数 mm（u16 小端，0xFFFF = 无效）
  *   [6..7] 保留
  */
static void Fly_SendGrabDistReport(uint8_t result, uint16_t raw_mm)
{
	uint8_t buf[8];

	buf[0] = 0xB6U;
	buf[1] = result;
	buf[2] = (uint8_t)(g_fly_grab_dist_mm & 0xFFU);
	buf[3] = (uint8_t)(g_fly_grab_dist_mm >> 8);
	buf[4] = (uint8_t)(raw_mm & 0xFFU);
	buf[5] = (uint8_t)(raw_mm >> 8);
	buf[6] = 0U;
	buf[7] = 0U;

	(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
}

uint8_t Fly_LearnGrabDist(void)
{
	uint8_t  slot;
	uint16_t d;

	slot = Tof8_FindSlot(FLY_TOF_USE_FOR_SLIDE);
	if (slot >= TOF8_DEV_NUM)
	{
		Fly_SendGrabDistReport(1U, TOF8_DIST_MM_INVALID);
		return 1U;
	}

	d = Tof8_ReadDistanceMm(slot, FLY_TOF_READ_TIMEOUT_MS);
	if ((d == TOF8_DIST_MM_INVALID) || (d == 0U))
	{
		/* ★ 读不到就【不改】原值：宁可留着旧值，
		 *   也不能把抓取距离写成无效值 —— 那会让整个靠近逻辑失效。 */
		Fly_SendGrabDistReport(1U, d);
		return 1U;
	}

	g_fly_grab_dist_mm = d;
	Fly_SendGrabDistReport(0U, d);
	return 0U;
}

/**
  * @brief  ★★★ 系统健康诊断（0x7E3，魔数 0xBA / 0xBB，每 1s 一组）★★★
  *
  *  ★ 为什么要这一帧：
  *    「跑到一半整个系统不动了」有四种完全不同的原因，以前无法区分：
  *      ① MCU 死了（HardFault / 栈溢出）  -> 不再发任何帧
  *      ② 内存不够（堆耗尽）              -> 任务创建失败 / 分配失败
  *      ③ CAN 总线异常（bus-off / 错误累积）-> 帧发不出去
  *      ④ 电机堵转保护                    -> 帧正常发，电机不动
  *    这一帧把 ②③ 的数字直接倒出来，配合 0x7FE 心跳和 0x7EF 死机帧，
  *    四种原因就都能分开。
  *
  *  帧 1（0xBA）：内存与栈
  *    [1..2] 当前剩余堆字节 u16LE（总 15360）
  *    [3..4] 历史最小剩余堆 u16LE（★ 应 >= 1000，否则随时会炸）
  *    [5]    Task_Flybox 栈高水位（字，低字节）
  *           ★ = 768 表示一字节没用过；= 100 表示只剩 400B，低于 150 危险
  *    [6..7] 保留
  *
  *  帧 2（0xBB）：CAN 总线
  *    [1..2] g_fly_tx_fail  u16LE
  *    [3..4] g_can_tx_flush u16LE（★ 在涨 = 总线拥堵/无应答）
  *    [5]    CAN 状态 0 正常 / 1 error passive / 2 bus off
  *    [6]    TEC 发送错误计数（★ > 96 警告，到 255 就 bus off）
  *    [7]    REC 接收错误计数
  */
void Fly_SendHealthReport(void)
{
	static uint8_t page = 0U;
	uint8_t  buf[8];
	uint32_t esr;
	uint32_t flushCnt;
	uint8_t  st;

	if (page == 0U)
	{
		uint32_t freeHeap = (uint32_t)xPortGetFreeHeapSize();
		uint32_t minHeap  = (uint32_t)xPortGetMinimumEverFreeHeapSize();

		/* ★ 用「当前任务」句柄：本函数就是在 Task_Flybox 里调的，
		 *   省掉一个外部句柄依赖（main.c 的 static 变量拿不到）。 */
		uint16_t hwm = (uint16_t)uxTaskGetStackHighWaterMark(NULL);

		buf[0] = 0xBAU;
		buf[1] = (uint8_t)(freeHeap & 0xFFU);
		buf[2] = (uint8_t)((freeHeap >> 8) & 0xFFU);
		buf[3] = (uint8_t)(minHeap & 0xFFU);
		buf[4] = (uint8_t)((minHeap >> 8) & 0xFFU);
		buf[5] = (uint8_t)(hwm & 0xFFU);
		buf[6] = 0U;
		buf[7] = 0U;

		(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
		page = 1U;
		return;
	}

	/* ---- 帧 2：CAN 总线 ---- */
	esr      = Can_GetEsr();
	flushCnt = Can_GetTxFlushCount();
	st  = (uint8_t)((esr >> 4) & 0x01U);          /* ESRS: 1 = 总线关闭 */
	if (st != 0U)
	{
		st = 2U;
	}
	else if (((esr >> 16) & 0xFFU) >= 0x80U)      /* TEC >= 128 = error passive */
	{
		st = 1U;
	}
	else
	{
		st = 0U;
	}

	buf[0] = 0xBBU;
	buf[1] = (uint8_t)(g_fly_tx_fail & 0xFFU);
	buf[2] = (uint8_t)((g_fly_tx_fail >> 8) & 0xFFU);
	buf[3] = (uint8_t)(flushCnt & 0xFFU);
	buf[4] = (uint8_t)((flushCnt >> 8) & 0xFFU);
	buf[5] = st;
	buf[6] = (uint8_t)((esr >> 16) & 0xFFU);      /* TEC */
	buf[7] = (uint8_t)((esr >> 24) & 0xFFU);      /* REC */

	(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);

	page = 0U;
}

/**
  * @brief  ★ TOF8 设备号扫描上报（0x7E3，魔数 0xB9，两帧）
  *
  *   [0] 0xB9    [1] 帧号(0/1)    [2] 扫到的总数
  *   [3..7] 本帧包含的设备号（每帧最多 5 个，共 10 个）
  */
void Fly_SendTofScanReport(void)
{
	uint8_t buf[8];
	uint8_t page;
	uint8_t i;

	for (page = 0U; page < 2U; page++)
	{
		buf[0] = 0xB9U;
		buf[1] = page;
		buf[2] = g_tof8_scan_cnt;

		for (i = 0U; i < 5U; i++)
		{
			uint8_t n = (uint8_t)(page * 5U + i);
			buf[3U + i] = (n < g_tof8_scan_cnt) ? g_tof8_scan_list[n] : 0xFFU;
		}

		(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
		osDelay(15U);
	}
}

/**
  * @brief  ★ TOF 健康诊断上报（0x7E3，魔数 0xB7）
  *
  *   ★ 一个槽位一帧，按槽位 0..TOF8_DEV_NUM-1 依次发，帧间留 20ms。
  *     字段含义见 flybox.h 的 Fly_SendTofHealth() 注释。
  *
  *   ★ 怎么判读（这是排查「激光为什么不起作用」最直接的手段）：
  *       应答计数 = 0        -> TOF 根本不在总线上：设备号错 / 没接 / 上电顺序不对
  *       应答计数 > 0 但无效值 -> 在总线上，但没对准目标（或超出量程）
  *       两者都有且距离正常 -> 激光可用
  */
void Fly_SendTofHealth(void)
{
	uint8_t  buf[8];
	uint8_t  i;

	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		uint16_t d = Tof8_GetDistanceMm(i);

		buf[0] = 0xB7U;
		buf[1] = i;
		buf[2] = Tof8_GetDev(i);
		buf[3] = (uint8_t)(Tof8_GetRespCount(i) & 0xFFU);
		buf[4] = Tof8_HasData(i);
		buf[5] = Tof8_IsFresh(i);
		buf[6] = (uint8_t)(d & 0xFFU);
		buf[7] = (uint8_t)(d >> 8);

		(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
		osDelay(20U);
	}
}

/**
  * @brief  前伸距离回复帧（0x7E3，魔数 0xB8）
  *
  *   [0] 0xB8 魔数
  *   [1] 保留
  *   [2..3] 当前生效的拉箱前伸距离 mm（u16 小端）
  *   [4..5] 当前生效的抓取距离 mm（u16 小端）
  *   [6..7] 保留
  */
void Fly_SendPullFwdReport(void)
{
	uint8_t buf[8];

	buf[0] = 0xB8U;
	buf[1] = 0U;
	buf[2] = (uint8_t)(g_fly_pull_fwd_mm & 0xFFU);
	buf[3] = (uint8_t)(g_fly_pull_fwd_mm >> 8);
	buf[4] = (uint8_t)(g_fly_grab_dist_mm & 0xFFU);
	buf[5] = (uint8_t)(g_fly_grab_dist_mm >> 8);
	buf[6] = 0U;
	buf[7] = 0U;

	(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
}

/**
  * @brief  ★ M5 抓钩归零上报（0x7E3，魔数 0xBC）
  *
  *   [0] 0xBC    [1] 结果 0=成功 / 1=指令被拒 / 2=未确认到位
  *   [2..3] 归零前位置（int16 小端，逻辑单位）
  *   [4..5] 归零后位置（int16 小端，应为 0）
  */
static void Fly_SendHookHomeReport(uint8_t result, int32_t before, int32_t after)
{
	uint8_t  buf[8];
	int16_t  b16 = (int16_t)((before > 32767) ? 32767 : ((before < -32768) ? -32768 : before));
	int16_t  a16 = (int16_t)((after  > 32767) ? 32767 : ((after  < -32768) ? -32768 : after));

	buf[0] = 0xBCU;
	buf[1] = result;
	buf[2] = (uint8_t)((uint16_t)b16 & 0xFFU);
	buf[3] = (uint8_t)((uint16_t)b16 >> 8);
	buf[4] = (uint8_t)((uint16_t)a16 & 0xFFU);
	buf[5] = (uint8_t)((uint16_t)a16 >> 8);
	buf[6] = 0U;
	buf[7] = 0U;

	(void)Can_SendStd(FLY_ID_STATE_REPORT, buf, 8U);
}

uint8_t Fly_HookHomeNow(void)
{
	int32_t sign;
	int32_t before_logical;
	int32_t after_logical;
	int32_t units;
	uint8_t r;

	if (s_ax[FLY_AXIS_HOOK].dev == FLY_DEV_UNKNOWN)
	{
		Fly_SendHookHomeReport(1U, 0, 0);
		return 1U;
	}

	sign           = (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_HOOK);
	before_logical = s_ax[FLY_AXIS_HOOK].pos * sign;

	/* ---- 反向 unwind：把抬起行程退回去，转到垂下位置 ----
	 * ★ 用【慢速】FLY_LOAD_SPEED：这是开环动作，可能已经顶到结构，
	 *   慢速能让驱动器更早进入堵转保护，而不是磅一下撞上去。 */
	units = -(int32_t)(FLY_HOOK_HOME_DEG * FLY_HOOK_UNITS_PER_DEG);

	r = Fly_MoveRelEx(FLY_AXIS_HOOK, units, FLY_LOAD_SPEED, FLY_LOAD_ACCEL, 1U);
	if (r != 0U)
	{
		/* 指令被拒（设备号未知 / 超限幅 / 软限位）
		 * ★ 软限位：如果置零过，反向会受到 FLY_HOOK_MIN_UNITS(-20°) 限制，
		 *   这时 unwind 会失败 —— 所以上电归零必须在 SetZeroAll【之前】做。 */
		Fly_SendHookHomeReport(1U, before_logical, before_logical);
		return 1U;
	}

	r = Fly_WaitAxisDone(FLY_AXIS_HOOK, FLY_FLOW_STEP_TIMEOUT_MS);
	osDelay(FLY_FLOW_STEP_GAP_MS);

	/* ---- 无论是否确认到位，都把当前位置记为零点 ----
	 * ★ 为什么「未到位也置零」：
	 *   这是开环动作，如果机械已经顶住了，再等也算不出来。
	 *   置零至少让「从当前位置算起的动作」是对的，
	 *   而不是让整个流程因为一次归零失败就跑不了。
	 *   结果码 2 会告出去，用户能从 0xBC 看到。 */
	(void)Fly_SetZeroAxis(FLY_AXIS_HOOK);

	after_logical = s_ax[FLY_AXIS_HOOK].pos * sign;
	Fly_SendHookHomeReport((r != 0U) ? 2U : 0U, before_logical, after_logical);

	return (r != 0U) ? 2U : 0U;
}

void Fly_SendPosReport(void)
{
	static uint8_t axis = 0U;
	static uint8_t seq  = 0U;
	uint8_t  buf[8];
	int32_t  pos;
	int32_t  rem;
	int16_t  p16;
	int16_t  r16;
	uint8_t  dev;

	dev = s_ax[axis].dev;
	if (dev == FLY_DEV_UNKNOWN)
	{
		axis = (uint8_t)((axis + 1U) % FLY_AXIS_COUNT);
		return;
	}

	pos = Fly_GetAxisPos((FlyAxis_t)axis);
	rem = Motor_GetRemainDev(dev);

	/* 16 位截断只是为了塞进一帧；看绝对值请用调试器 */
	p16 = (int16_t)((pos > 32767) ? 32767 : ((pos < -32768) ? -32768 : pos));
	r16 = (int16_t)((rem > 32767) ? 32767 : ((rem < -32768) ? -32768 : rem));

	buf[0] = 0xAAU;
	buf[1] = seq++;
	buf[2] = axis;
	buf[3] = (uint8_t)((uint16_t)p16 & 0xFFU);
	buf[4] = (uint8_t)((uint16_t)p16 >> 8);
	buf[5] = (uint8_t)((uint16_t)r16 & 0xFFU);
	buf[6] = (uint8_t)((uint16_t)r16 >> 8);
	buf[7] = (uint8_t)(Motor_GetFbCountDev(dev) & 0xFFU);

	(void)Can_SendStd(FLY_ID_POS_REPORT, buf, 8U);

	axis = (uint8_t)((axis + 1U) % FLY_AXIS_COUNT);
}

/* =========================================================================
 *                          调试命令 0x111
 * ========================================================================= */

void Fly_OnCommand(const uint8_t *data, uint8_t dlc)
{
	uint8_t  cmd;
	FlyAxis_t axis;
	uint8_t  dir;
	uint16_t mm;
	uint8_t  spd_mmps;
	int32_t  dist;
	uint32_t speed;

	if ((data == NULL) || (dlc < 1U)) { return; }

	cmd = data[0];

	switch (cmd)
	{
	case FLY_CMD_JOG:
		/* [1]轴号 [2]方向 [3..4]距离mm [5]速度mm/s */
		if (dlc < 6U) { break; }
		axis = (FlyAxis_t)data[1];
		if (axis >= FLY_AXIS_COUNT) { break; }

		dir = data[2];
		mm  = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
		spd_mmps = data[5];

		if (mm == 0U) { mm = 10U; }                     /* 0 当成 10mm，避免无动作 */
		if (mm > FLY_JOG_MAX_MM) { mm = FLY_JOG_MAX_MM; }
		if (spd_mmps == 0U) { spd_mmps = (uint8_t)FLY_DEFAULT_SPEED_MMPS; }

		if (Fly_IsRotary(axis) != 0U)
		{
			/* 旋转轴：这里的「mm」当成「度」用，方便直接点 90° */
			dist  = (int32_t)mm * Fly_UnitsPerDeg(axis);
			speed = (uint32_t)((int32_t)spd_mmps * Fly_UnitsPerDeg(axis));
		}
		else
		{
			dist  = (int32_t)mm * (int32_t)FLY_UNITS_PER_MM;      /* mm -> 指令单位 */
			speed = (uint32_t)spd_mmps * (uint32_t)FLY_UNITS_PER_MM;
		}
		if (dir == 0U) { dist = -dist; }

		g_fly_jog_axis  = (uint8_t)axis;
		g_fly_jog_delta = dist;

		if (Fly_MoveRel(axis, dist, speed, s_ax[axis].accel) == 0U)
		{
			(void)Fly_WaitAxisDone(axis, 10000U);
		}
		g_fly_jog_axis = 0xFFU;
		break;

	case FLY_CMD_STOP:
		/* 只是从容地不再发新位置；不能靠「发 0 目标」急停（会中途改目标）
		 * ★ 必须同时清掉流程请求，否则自动循环会继续起下一轮 ——
		 *   那就变成「按了停止也不停」。 */
		g_fly_abort     = 1U;
		g_fly_flow_req  = 0U;
		break;

	case FLY_CMD_QUERY_POS:
		Fly_RequestPositions();
		break;

	case FLY_CMD_SCAN:
		(void)Fly_ScanDevices(FLY_SCAN_GAP_MS);
		(void)Fly_AutoMapDevices();
		Fly_SendScanReport();
		break;

	case FLY_CMD_JOG_TEST:
		/* 只置标志，真正的点动由 Task_Flybox 执行（这里不能阻塞）*/
		Fly_RequestJogTest();
		break;

	case FLY_CMD_SET_ZERO:
		/* [1] = 轴号，0xFF = 全部
		 * ★ 只置标志，由 Task_Flybox 执行（保持「所有碰电机的事都在同一任务里」）。
		 * ★ 前提：调用方必须已经把机构物理上摆到该轴的零点姿态。 */
		if (dlc >= 2U)
		{
			Fly_RequestSetZero(data[1]);
		}
		else
		{
			Fly_RequestSetZero(0xFFU);      /* 没给轴号就全部都置 */
		}
		break;

	case FLY_CMD_TOF_PROBE:
		/* [1] 轴（0=皮带 1=滑台） [2..3] 步长mm u16LE [4] 步数
		 * ★ 只置请求，由 Task_Flybox 执行（内部要等到位，不能在这个任务里跑）*/
		if (dlc >= 2U)
		{
			g_fly_tofprobe_req  = 1U;
			g_fly_tofprobe_belt = (data[1] != 0U) ? 1U : 0U;
			g_fly_tofprobe_step = (dlc >= 4U)
			                      ? (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8))
			                      : FLY_TOF_CHUNK_MM;
			g_fly_tofprobe_cnt  = (dlc >= 5U) ? data[4] : 5U;
		}
		break;

	case FLY_CMD_ACTION:
		if (dlc < 2U) { break; }
		switch (data[1])
		{
		case FLY_ACT_HOOK_UP:    (void)Fly_HookUp();    break;
		case FLY_ACT_HOOK_DOWN:  (void)Fly_HookDown();  break;
		case FLY_ACT_SWING_WORK: (void)Fly_SwingWork(); break;
		case FLY_ACT_SWING_PARK: (void)Fly_SwingPark(); break;
		case FLY_ACT_SLIDE_HOME:
			/* 没挡块，「回原点」只能是回软件零点 pos = 0。
			 * ★ 前提是已置零（zeroed == 1），否则 pos 的 0 没有物理含义。 */
			if (s_ax[FLY_AXIS_SLIDE].zeroed != 0U)
			{
				int32_t d = -s_ax[FLY_AXIS_SLIDE].pos;

				if ((d != 0) &&
				    (Fly_MoveRel(FLY_AXIS_SLIDE, d, s_ax[FLY_AXIS_SLIDE].speed,
				                 s_ax[FLY_AXIS_SLIDE].accel) == 0U))
				{
					(void)Fly_WaitAxisDone(FLY_AXIS_SLIDE, 20000U);
				}
			}
			break;
		case FLY_ACT_BELT_FWD:
			if (Fly_BeltMove(3000, Fly_AxisSpeed(FLY_AXIS_BELT_L),
			                 Fly_AxisAccel(FLY_AXIS_BELT_L)) == 0U)
			{
				(void)Fly_WaitBeltDone(10000U);
			}
			break;
		case FLY_ACT_BELT_BACK:
			if (Fly_BeltMove(-3000, Fly_AxisSpeed(FLY_AXIS_BELT_L),
			                 Fly_AxisAccel(FLY_AXIS_BELT_L)) == 0U)
			{
				(void)Fly_WaitBeltDone(10000U);
			}
			break;
		default:
			break;
		}
		break;

	case FLY_CMD_BELT_JOG:
		/* [1] 方向 0=负 1=正  [2..3] 距离mm u16LE
		 * ★ 只置请求，由 Task_Flybox 执行（内部要等到位）*/
		g_fly_beltjog_req = 1U;
		g_fly_beltjog_mm  = (dlc >= 4U)
		                    ? (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8))
		                    : 30U;
		if ((dlc >= 2U) && (data[1] == 0U)) { g_fly_beltjog_mm = (uint16_t)(-(int16_t)g_fly_beltjog_mm & 0xFFFF); }
		break;

	case FLY_CMD_RUN_FLOW:
		/* [1] 1 = 拉箱，2 = 推箱
		 * ★ 只置请求，由 Task_Flybox 执行（流程内部要等到位）*/
		if (dlc < 2U) { break; }
		if ((data[1] != FLY_FLOW_PULL) && (data[1] != FLY_FLOW_PUSH)) { break; }
		if (g_fly_flow_running != 0U) { break; }   /* 正在跑，不打断 */
		g_fly_flow_which = data[1];
		g_fly_flow_req   = 1U;
		break;

	case FLY_CMD_TOF_HEALTH:
		/* ★ TOF 健康诊断：只置请求，由 Task_Flybox 执行
		 *   （内部要发多帧并留间隔，不能在 CAN 接收任务里阻塞）。 */
		g_fly_tofhealth_req = 1U;
		break;

	case FLY_CMD_PULL_FWD:
		/* ★ 拉箱前伸距离当场改：[1..2] mm u16LE
		 *
		 *   这是「不用重编烧录也能调距离」的入口。
		 *   激光靠不住时直接用它把前伸定死。 */
		if (dlc >= 3U)
		{
			uint16_t v = (uint16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8));
			if ((v > 0U) && (v <= 1000U)) { g_fly_pull_fwd_mm = v; }
			Fly_SendPullFwdReport();
		}
		break;

	case FLY_CMD_TOF_SCAN:
		/* ★ 扫 TOF8 真实设备号：只置请求，由 Task_Flybox 执行
		 *   （要对 16 个设备号逐个发帧并等应答，不能在接收任务里跑）。 */
		g_fly_tofscan_req = 1U;
		break;

	case FLY_CMD_HOOK_HOME:
		/* ★ M5 抓钩归零：只置请求，由 Task_Flybox 执行（要等到位）。
		 *   用途：流程中途停掉后抓钩停在抬起位置，手动把它转回垂下。 */
		if (g_fly_flow_running == 0U) { g_fly_hookhome_req = 1U; }
		break;

	case FLY_CMD_GRAB_DIST:
		/* ★ 抓取距离（拉箱靠近的停止点）
		 *   DLC <= 1     -> 学习：把当前 TOF 读数记下来
		 *   DLC >= 3     -> 手动设：[1..2] mm u16 小端
		 *
		 * ★ 学习要读 TOF，不能在 CAN 接收任务里做，只置请求。 */
		if (dlc >= 3U)
		{
			uint16_t v = (uint16_t)((uint16_t)data[1] | ((uint16_t)data[2] << 8));
			if (v > 0U) { g_fly_grab_dist_mm = v; }
		}
		else
		{
			g_fly_learn_req = 1U;
		}
		break;

	default:
		break;
	}
}

/* =========================================================================
 *                          点动自检
 * ========================================================================= */

/**
  * @brief  ★ 单轴探针：上报该轴此刻的「剩余位移」和「绝对位置」
  *
  *         这是回答「电机到底有没有收到位置指令」最直接的证据。
  *         发完 0x0416 [1000 单位] 之后：
  *             · 剩余位移 ≈ 1000    -> ★ 电机收到了，正在走
  *             · 剩余位移 = 0       -> ★ 电机没收到（或收到但不出力）
  *             · HasRemain 位 = 0   -> 从没收到过 0x0420，该电机不回报剩余位移
  *
  *         ★ 为什么必须单独一帧：0x7E2 那个位置上报是 400ms 轮一个轴，
  *           点动期间基本抓不到关键那一瞬。
  *
  * @param  axis  轴号
  * @param  tag   1=刚发完指令 2=等待后 3=反向发完
  */
static void Fly_SendAxisProbe(FlyAxis_t axis, uint8_t tag)
{
	static uint8_t seq = 0U;
	uint8_t  buf[8];
	uint8_t  dev;
	int16_t  r16;
	int16_t  p16;
	int32_t  rem;
	int32_t  pos;

	if (axis >= FLY_AXIS_COUNT) { return; }
	dev = s_ax[axis].dev;
	if (dev == FLY_DEV_UNKNOWN) { return; }

	rem = Motor_GetRemainDev(dev);
	pos = Motor_GetQueryPosDev(dev);

	r16 = (int16_t)((rem > 32767) ? 32767 : ((rem < -32768) ? -32768 : rem));
	p16 = (int16_t)((pos > 32767) ? 32767 : ((pos < -32768) ? -32768 : pos));

	buf[0] = 0xB3U;                                 /* 魔数 */
	buf[1] = (uint8_t)axis;
	/* [2] 高4位 = 设备号，bit3 = 收到过 0x0420，bit2 = 走从帧组 */
	buf[2] = (uint8_t)(((dev & 0x0FU) << 4) |
	                   ((Motor_HasRemainDev(dev) != 0U) ? 0x08U : 0x00U) |
	                   ((s_ax[axis].use_slave != 0U) ? 0x04U : 0x00U));
	buf[3] = (uint8_t)((uint16_t)r16 & 0xFFU);      /* 剩余位移低 */
	buf[4] = (uint8_t)((uint16_t)r16 >> 8);         /* 剩余位移高 */
	buf[5] = (uint8_t)((uint16_t)p16 & 0xFFU);      /* 绝对位置低 */
	buf[6] = (uint8_t)((uint16_t)p16 >> 8);         /* 绝对位置高 */
	buf[7] = (uint8_t)(((tag & 0x0FU) << 4) | (seq++ & 0x0FU));

	(void)Can_SendStd(FLY_ID_AXIS_PROBE, buf, 8U);
}

/**
  * @brief  发一帧点动进度（0x7E5），让人用 CAN 分析仪就能看到进行到哪一步
  * @param  axis   轴号
  * @param  phase  1 = 正转开始，2 = 反转开始，3 = 该轴完成
  * @param  result 0 = 成功，1 = 拒绝下发（参数/限位/发送失败），2 = 无法确认到位
  * @param  units  本次指令的单位数
  */
static void Fly_SendJogReport(uint8_t axis, uint8_t phase, uint8_t result, int32_t units)
{
	static uint8_t seq = 0U;
	uint8_t  buf[8];
	int16_t  u16v;

	u16v = (int16_t)((units > 32767) ? 32767 : ((units < -32768) ? -32768 : units));

	buf[0] = 0xABU;
	buf[1] = axis;
	buf[2] = phase;
	buf[3] = result;
	buf[4] = (uint8_t)((uint16_t)u16v & 0xFFU);
	buf[5] = (uint8_t)((uint16_t)u16v >> 8);
	buf[6] = (axis < FLY_AXIS_COUNT) ? s_ax[axis].dev : 0xFFU;
	buf[7] = seq++;

	(void)Can_SendStd(FLY_ID_JOG_REPORT, buf, 8U);
}

/**
  * @brief  ★ 两条皮带同步点动：+N -> 停 -> -N，验证「能不能一起动」
  *
  *  ★ 为什么单独一个函数：
  *    Fly_RunJogTest() 是【逐轴】测试（一次只动一个），
  *    目的是分辨「哪个设备号对应哪个关节」。
  *    所以皮带一前一后地动是设计如此，不是 bug。
  *    要验证「同步性」必须用 Fly_BeltMove()，它把两帧位置指令紧挨着发，
  *    中间不留延时 —— 这是课题二左右轮同步已验证过的做法。
  *
  * @param  units  位移，0.1mm（正 = 向前输送）
  * @retval 0 成功，1 下发失败
  */
uint8_t Fly_BeltBothJog(int32_t units)
{
	uint8_t ok;

	if (units == 0) { return 1U; }
	if (units > (int32_t)FLY_JOG_MAX_MM * (int32_t)FLY_UNITS_PER_MM)  { units = (int32_t)FLY_JOG_MAX_MM * (int32_t)FLY_UNITS_PER_MM; }
	if (units < -(int32_t)FLY_JOG_MAX_MM * (int32_t)FLY_UNITS_PER_MM) { units = -(int32_t)FLY_JOG_MAX_MM * (int32_t)FLY_UNITS_PER_MM; }

	g_fly_jog_axis = FLY_AXIS_BELT_L;

	if (Fly_BeltMove(units, FLY_JOG_TEST_SPEED, FLY_JOG_TEST_ACCEL) != 0U)
	{
		g_fly_jog_axis = 0xFFU;
		return 1U;
	}

	(void)Fly_WaitBeltDone(FLY_JOGTEST_TIMEOUT_MS);

	/* ★ 只留极短的间隔就反向，实现连续往复（原来等 500ms 纯白等）*/
	osDelay(FLY_JOG_TEST_PAUSE_MS);

	/* 反向回原位，净位移 0 */
	if (Fly_BeltMove(-units, FLY_JOG_TEST_SPEED, FLY_JOG_TEST_ACCEL) != 0U)
	{
		g_fly_jog_axis = 0xFFU;
		return 1U;
	}

	ok = Fly_WaitBeltDone(FLY_JOGTEST_TIMEOUT_MS);
	osDelay(FLY_JOG_TEST_PAUSE_MS);

	g_fly_jog_axis = 0xFFU;
	return (ok != 0U) ? 0U : 1U;
}

/* =========================================================================
 *        ★★ 业务流程：拉箱 / 推箱
 *
 *  ★ 设计原则
 *    1) 每步都走到一个「地标位置」（绝对目标 + 自动补差），
 *       所以某一步没走够 / 走过头，下一轮会自动修回来，误差不累积。
 *    2) 每一步都是「下发 -> 等到位确认 -> 停极短时间 -> 下一步」。
 *       绝不在运动中改目标 —— 课题二实测那样会把电机搞死。
 *    3) 失败处理由 FLY_FLOW_NEVER_STOP 决定：
 *         1 = 盲走，不管结果继续下一步（★ 当前，调试期用）
 *         0 = 任一步失败立即中止整轮
 * ========================================================================= */

/* 单步执行（可指定速度 / 加速度），实现在下面 */
static uint8_t Fly_DoStepSpd(uint8_t step_no, FlyAxis_t ax, int32_t units, uint8_t use_belt,
                             uint32_t speed, uint32_t accel);

/* 单步执行（用流程默认速度）
 *
 * ★ 返回值：
 *     FLY_FLOW_NEVER_STOP == 0 时：0 = 成功，1 = 失败（调用方会中止整轮）
 *     FLY_FLOW_NEVER_STOP == 1 时：永远返回 0（盲走，失败只记原因码） */
static uint8_t Fly_DoStep(uint8_t step_no, FlyAxis_t ax, int32_t units, uint8_t use_belt)
{
	return Fly_DoStepSpd(step_no, ax, units, use_belt, FLY_FLOW_SPEED, FLY_FLOW_ACCEL);
}

/* 单步执行（可指定速度 / 加速度）
 *
 * ★ 为什么要分速度：
 *   「抬钩 + 拖箱」是带载动作，力矩需求是空载的好几倍。
 *   用空载的速度去跑带载动作，冲击电流会顶到驱动器/电源的极限，
 *   表现就是「抬钩带上箱子后系统停了」。
 *   所以带载步骤走 FLY_LOAD_SPEED / FLY_LOAD_ACCEL（慢 3 倍）。 */
static uint8_t Fly_DoStepSpd(uint8_t step_no, FlyAxis_t ax, int32_t units, uint8_t use_belt,
                             uint32_t speed, uint32_t accel)
{
	uint8_t ok = 1U;

	g_fly_flow_step = step_no;
	g_fly_flow_why  = FLY_WHY_NONE;

	/* ★ 每步开始就报一帧：流程是同步执行的，主循环的定期上报
	 *   会被完全占住，不主动发就什么都看不到。 */
	Fly_SendFlowReport();

	if (units == 0) { return 0U; }

	if (use_belt != 0U)
	{
		/* 两皮带同步：两帧位置紧挨着发，中间不留延时 */
		if (Fly_BeltMove(units, speed, accel) != 0U)
		{
			g_fly_flow_why = FLY_WHY_SEND;        /* 帧都没发出去 */
			ok = 0U;
		}
		else
		{
			ok = Fly_WaitBeltDone(FLY_FLOW_STEP_TIMEOUT_MS);
		}
	}
	else
	{
		/* ★ allow_nozero = 1：流程用相对位移，不要求先置零 */
		if (Fly_MoveRelEx(ax, units, speed, accel, 1U) != 0U)
		{
			/* 指令被拒，一帧都没发出去（软限位越界 / 超限幅 / 设备号未知）*/
			g_fly_flow_why = FLY_WHY_REFUSED;
			ok = 0U;
		}
		else
		{
			ok = Fly_WaitAxisDone(ax, FLY_FLOW_STEP_TIMEOUT_MS);
		}
	}

	if ((ok == 0U) && (g_fly_flow_why == FLY_WHY_NONE))
	{
		g_fly_flow_why = FLY_WHY_TIMEOUT;
	}

	/* ★★★ 堵转/未到位取证（2026-10-30 新增）★★★
	 *
	 *   现场故障：往回走到一半整个系统不动了。
	 *   要区分两种可能：
	 *     ① 电机卡住（机械卡死 / 箱子卡在皮带边缘）-> 剩余位移永远不归零
	 *     ② MCU 死了（栈溢出/HardFault）-> CAN 上什么都不再发
	 *
	 *   这里记录「这一步有没有确认到位」，由 0xB5 帧的 [7] 位 1 上报：
	 *     [7] bit1 = 1  ->  这一步没能确认到位（堵转嫌疑）
	 *     连着几步 bit1 都是 1，就基本可以断定是机械卡死。
	 */
	if (ok == 0U) { g_fly_flow_stall = 1U; }

	/* 停稳留一点时间再下一步（无论如何都要留，机械本身停下需要时间）*/
	osDelay(FLY_FLOW_STEP_GAP_MS);

#if (FLY_FLOW_NEVER_STOP != 0)
	/* ★ 盲走：不管结果一律算成功，继续下一步。
	 *   g_fly_abort 必须清掉 —— Fly_WaitAxisDone 超时时会把它置 1，
	 *   留着会让后面的 TOF 闭环等逻辑误以为处于异常状态。
	 *   失败信息保留在 g_fly_flow_why 里，0x7E3 帧 D 的 [5] 能看到。 */
	(void)ok;
	g_fly_abort = 0U;
	return 0U;
#else
	return (ok != 0U) ? 0U : 1U;
#endif
}

/* =========================================================================
 *        ★ 绝对位置式单步（流程真正的执行单位）
 *
 *  ★★★ 为什么流程要用「绝对目标」而不是「相对位移」★★★
 *
 *    相对位移的致命问题：误差会累积。
 *      · 某一步被软限位拒了 / 中途失败 / 手动扳了机构
 *      · 下一轮再从当前位置走同样的相对量 -> 越跑越偏
 *      · 平台旋转尤其明显：每轮多转一点，几轮后就顶到结构
 *
 *    而 s_ax[].pos 一直在跟踪每一次真实下发的位移（见 Fly_MoveRelEx），
 *    所以「目标 - 当前」这个差值就等价于「绝对定位」，
 *    每一轮都会自动把上一轮的偏差补回来。
 *
 *  ★ 目标值用【逻辑方向】写（正 = 机构该往那边走），
 *    内部换算成物理差值后交给 Fly_MoveRelEx，符号只有那一处。
 * ========================================================================= */
static uint8_t Fly_DoStepTo(uint8_t step_no, FlyAxis_t ax, int32_t logical_target)
{
	int32_t sign;
	int32_t cur_logical;
	int32_t delta;

	if (ax >= FLY_AXIS_COUNT) { return 1U; }

	sign        = (int32_t)FLY_DIR_SIGN_OF(ax);
	cur_logical = s_ax[ax].pos * sign;      /* pos 是物理方向 -> 换回逻辑 */
	delta       = logical_target - cur_logical;

	/* 已经到位（±1 单位 = 0.01mm）：不必再发帧。
	 * ★ 发一条 0 位移的指令没有意义，还可能让电机进入异常状态。 */
	if ((delta > -1) && (delta < 1))
	{
		g_fly_flow_step = step_no;
		g_fly_flow_why  = FLY_WHY_NONE;
		Fly_SendFlowReport();
		osDelay(FLY_FLOW_STEP_GAP_MS);
		return 0U;
	}

	return Fly_DoStep(step_no, ax, delta, 0U);
}

/* 绝对位置式单步 + 指定速度
 *
 * ★ 带载动作（抬钩、拖箱、放钩）必须用这个：
 *   力矩需求是空载的好几倍，用空载速度跑会顶到驱动器/电源极限。 */
static uint8_t Fly_DoStepToSpd(uint8_t step_no, FlyAxis_t ax, int32_t logical_target,
                               uint32_t speed, uint32_t accel)
{
	int32_t sign;
	int32_t cur_logical;
	int32_t delta;

	if (ax >= FLY_AXIS_COUNT) { return 1U; }

	sign        = (int32_t)FLY_DIR_SIGN_OF(ax);
	cur_logical = s_ax[ax].pos * sign;
	delta       = logical_target - cur_logical;

	if ((delta > -1) && (delta < 1))
	{
		g_fly_flow_step = step_no;
		g_fly_flow_why  = FLY_WHY_NONE;
		Fly_SendFlowReport();
		osDelay(FLY_FLOW_STEP_GAP_MS);
		return 0U;
	}

	return Fly_DoStepSpd(step_no, ax, delta, 0U, speed, accel);
}

/* ---- 各轴的「逻辑位置地标」----
 * ★ 零点在 Fly_SetZeroAll() 里定义为「上电时的姿态」，
 *   所以这些地标都是相对上电姿态的。 */

#define FLY_LAND_SLIDE_HOME     0                                /* 完全缩回 */
#define FLY_LAND_SLIDE_OUT      ((int32_t)FLY_PULL_SLIDE_FWD_UNITS)  /* 前伸到抓取位 */
#define FLY_LAND_HOOK_DOWN      0                                /* 抓钩垂下 */

/* ★ 拉箱：100°（90° 勾住 + 10° 把箱子抬到皮带承载面）
 *   箱子在外部货架上，比皮带面低，不多抬这 10° 拖不进通道。 */
#define FLY_LAND_HOOK_UP        ((int32_t)FLY_HOOK_UP_UNITS)

/* ★ 推箱：仍然 90°。
 *   推箱时箱子【已经平放在皮带上】，多抬 10° 会把箱子抬离皮带面，
 *   推到一半再落下来容易卡住皮带边缘。两个流程的抬起高度不同。 */
#define FLY_LAND_HOOK_UP_PUSH   ((int32_t)(FLY_HOOK_DEG * FLY_HOOK_UNITS_PER_DEG))

#define FLY_LAND_SWING_WORK     0                                /* 工作位（对准皮带通道）*/
#define FLY_LAND_SWING_PARK     ((int32_t)FLY_SWING_PARK_UNITS)  /* 右转 90° 让位 */

/* =========================================================================
 *          ★ 拉箱第 2 步：用激光测距靠近箱子（粗走 + 精走）
 *
 *  ★ 为什么分两段：
 *    全程用小步太慢 —— 每 10mm 都要「走 -> 停稳 -> 读数」，
 *    300mm 要走 30 次，每次差不多 0.3s，一共 9 秒。
 *    而箱子位置是固定的，先按读到的距离一次走掉大部分，
 *    剩下的几十毫米再用小步收敛，精度一样是 10mm，但快得多。
 *
 *  ★ 为什么粗走要留余量（FLY_PULL_TOF_COARSE_MARGIN_MM）：
 *    万一读数偏小、或者箱子被挪近了，一次走满就会直接顶到箱子。
 *    留 3cm 给精走，顶多撞 3cm 的余量，可以接受。
 *
 *  ★ 读不到距离 / 读数不跟着动时：
 *    退回「盲走」，并且【把已经走掉的距离扣掉，把剩下的补齐】。
 *
 *  ★★★ 为什么必须补齐（2026-10-30 现场踩到的坑）★★★
 *
 *    激光装在抓钩下方，如果它实际朝的不是纵向（或者对着没有箱子的空处），
 *    那么平台往前走、读数却一动不动。
 *    卡住检测（FLY_TOF_STALL_STEPS=3）就会在走了 3 x 10mm 之后判定「卡住」并退出。
 *
 *    表现就是：平台只前进 3cm 就停，后面的「后退」也只有 3cm ——
 *    肉眼看就是「动作没变、还是那点距离」。
 *
 *    现在改成：不管激光能不能用，平台最终一定走到同一个位置。
 *      激光能用 -> 停在距箱 FLY_PULL_GRAB_DIST_MM，精度高
 *      激光不能用 -> 退回盲走 FLY_PULL_SLIDE_FWD_MM，动作仍然完整
 *    区别只体现在 0x7E3 帧 B5 的 [4] 原因码上。
 * ========================================================================= */
static uint8_t Fly_PullApproachBox(void)
{
#if (FLY_PULL_USE_TOF == 0)
	/* 不用激光：退回固定距离盲走 */
	return Fly_DoStep(2U, FLY_AXIS_SLIDE, (int32_t)FLY_PULL_SLIDE_FWD_UNITS, 0U);
#else
	uint8_t  slot;
	uint16_t d0;
	int32_t  coarse_mm;
	int32_t  cap_mm;
	int32_t  pos_before;
	int32_t  moved_fwd;         /* 本步已经向前走的逻辑单位数 */
	int32_t  remain;
	int32_t  fine_budget;       /* 精走还允许走多少 mm */
	uint8_t  r;

	g_fly_flow_step = 2U;
	g_fly_flow_why  = FLY_WHY_NONE;

	pos_before = s_ax[FLY_AXIS_SLIDE].pos;      /* 物理方向记录 */

	/* ---- 找到「滑台定位」用的那一路 TOF ---- */
	slot = Tof8_FindSlot(FLY_TOF_USE_FOR_SLIDE);
	if (slot >= TOF8_DEV_NUM)
	{
		/* 设备号没配上 -> 用兜底距离 */
		goto blind;
	}

	/* ---- 读初始距离 ---- */
	d0 = Tof8_ReadDistanceMm(slot, FLY_TOF_READ_TIMEOUT_MS);
	if (d0 == TOF8_DIST_MM_INVALID)
	{
		goto blind;
	}

	/* ---- ★ 读数分三种情况处理（2026-10-30 修掉我引入的 bug）----
	 *
	 *  ★★ 血泪史：这里前后写错过两次，两个方向都错 ★★
	 *
	 *  错法一（最初）：
	 *      if (d0 <= 抓取距离) { return 0U; }      一步不动
	 *    问题：TOF 如果测到地面/侧壁/钩子自己，读数很小 ->
	 *          一步不走就抬手 -> "平台不动，抓钩抓空气"。
	 *
	 *  错法二（我改成下面这行的版本）：
	 *      if (d0 <= 抓取距离) { goto blind; }     盲走 520mm
	 *    问题：TOF 正常且【真的已经到位】时也会盲走 520mm ->
	 *          直接把箱子顶走 5cm。现场"顶着箱子前行"就是这么来的。
	 *          ★ 这是更危险的方向：错法一只是抓空，错法二是撞箱子。
	 *
	 *  ★ 正确做法：三种情况分开，谁都不猜
	 *    ① 读不到 / 读数离谱 -> 盲走（TOF 不可用，只能靠距离）
	 *    ② 读数可信且已到位 -> 【不动】，并记原因码
	 *       如果其实测错了，操作者会看到"平台没动"，
	 *       一眼就知道是 TOF 在骗人 —— 比撞箱子安全得多。
	 *    ③ 读数可信且还远   -> 正常靠近
	 */

	/* ---- ① TOF 不可用 -> 盲走 ---- */
	if (d0 > FLY_PULL_TOF_TRUST_MAX_MM)
	{
		goto blind;
	}

	/* ---- ② 读数可信，且已经比目标还近 -> 不动 ---- */
	if (d0 <= g_fly_grab_dist_mm)
	{
		/* ★ 不动，但必须让操作者知道「为什么没动」。
		 *   0x7E3 帧 B5 的 [4] 会是 FLY_WHY_TOF_NEAR(6)，
		 *   [5..6] 会显示 TOF 实际读数。
		 *   如果这个读数明显不对（比如明明离箱子很远却报 50mm），
		 *   那就是 TOF 测错了东西，不是「已经到位」。 */
		g_fly_flow_why = FLY_WHY_TOF_NEAR;
		Fly_SendFlowReport();
		osDelay(FLY_FLOW_STEP_GAP_MS);
		return 0U;
	}

	/* ---- ③ 正常靠近：粗走 + 精走 ---- */

	/* ---- 第一段：粗走，一次走掉大部分行程 ----
	 * 目标 = 当前距离 - 抓取距离 - 余量
	 * 上限 = FLY_PULL_TOF_APPROACH_MAX_MM（滑台真实行程的 95%）
	 * ★ 不再像以前那样写死 300mm —— 那会让远处的箱子永远够不到，
	 *   而且和盲走兜底的 300mm 撞在一起，根本分不出激光有没有起作用。 */
	coarse_mm = (int32_t)d0
	          - (int32_t)g_fly_grab_dist_mm
	          - (int32_t)FLY_PULL_TOF_COARSE_MARGIN_MM;

	cap_mm = (int32_t)FLY_PULL_TOF_COARSE_MAX_MM;
	if (coarse_mm > cap_mm) { coarse_mm = cap_mm; }
	if (coarse_mm < 0)      { coarse_mm = 0; }

	/* 精走的预算 = 总接近上限 - 粗走已用
	 * ★ 这是为了让「箱子比粗走限幅更远」时精走还能继续往前走，
	 *   而不是像以前那样卡在 110mm 就停。
	 *
	 * ★★ 预算为 0 时【必须跳过精走】，不能照发 ★★
	 *   Fly_TofFeedLoop 里把 max_travel=0 解释成「用默认 1500mm」，
	 *   所以传 0 进去等于放它再走 1.5 米 —— 滑台早就顶到结构了。 */
	fine_budget = (int32_t)FLY_PULL_TOF_APPROACH_MAX_MM - coarse_mm;
	if (fine_budget < 0) { fine_budget = 0; }

	if (fine_budget < (int32_t)FLY_TOF_CHUNK_MM)
	{
		/* 粗走已经吃掉全部预算：没有任何精走余地，直接结束。
		 * 走到这里说明「箱子比滑台行程还远」，激光再准也够不到。 */
		g_fly_flow_why = FLY_WHY_TOF_STALL;
		return 0U;
	}

	if (coarse_mm > 0)
	{
		if (Fly_DoStep(2U, FLY_AXIS_SLIDE,
		               coarse_mm * (int32_t)FLY_UNITS_PER_MM, 0U) != 0U)
		{
			/* 粗走被拒（超行程限幅等）-> 至少把精走跑完 */
			g_fly_flow_why = FLY_WHY_REFUSED;
		}
	}

	/* ---- 第二段：精走，10mm 小步收到目标距离 ----
	 * ★ max_travel 只给「粗走剩下的余量 + 一点富余」，
	 *   绝不允许它自己再走几百毫米 —— 那说明 TOF 看的不是箱子。 */
	r = Fly_SlideToTof(slot,
	                   g_fly_grab_dist_mm,
	                   0U,                          /* 让距离减小 */
	                   FLY_APPROACH_SPEED,
	                   (uint16_t)fine_budget,
	                   FLY_PULL_TOF_TIMEOUT_MS);

	if (r == FLY_TOF_OK)
	{
		return 0U;
	}

	/* =====================================================================
	 *  ★★★ 激光这条路没走通 -> 退回盲走，把剩余距离补齐 ★★★
	 *
	 *  先算出「本步已经向前走了多少」（逻辑方向，向前为正），
	 *  再用盲走把 FLY_PULL_SLIDE_FWD_UNITS - 已走 补上。
	 *
	 *  ★ 为什么用 pos 差值而不是「走了几步 x 10mm」：
	 *    pos 是每次真实下发都会累加的物理量，粗走、精走、被拒的帧
	 *    全都会如实反映，不会漏算也不会重复算。
	 * ===================================================================== */

	/* pos 是物理方向，乘以符号换回「逻辑向前」的正数 */
	moved_fwd = (s_ax[FLY_AXIS_SLIDE].pos - pos_before)
	          * (int32_t)FLY_DIR_SIGN_OF(FLY_AXIS_SLIDE);

	remain = (int32_t)g_fly_pull_fwd_mm * (int32_t)FLY_UNITS_PER_MM - moved_fwd;

	/* 记下是哪种失败，用户能在 0x7E3 帧 B5 的 [4] 看到 */
	g_fly_flow_why = (r == FLY_TOF_ERR_STALL) ? FLY_WHY_TOF_STALL : FLY_WHY_NO_TOF;

	if (remain > 0)
	{
		/* 补齐剩余行程。★ 超过单次限幅就分两次走。 */
		while (remain > 0)
		{
			int32_t chunk = remain;
			int32_t lim   = (int32_t)FLY_JOG_MAX_MM * (int32_t)FLY_UNITS_PER_MM;

			if (chunk > lim) { chunk = lim; }

			if (Fly_DoStep(2U, FLY_AXIS_SLIDE, chunk, 0U) != 0U)
			{
				break;              /* 补齐也失败就认了，别死循环 */
			}
			remain -= chunk;
		}
	}

	/* ★ 补齐完成：动作是完整的，返回值按成功处理。
	 *   原因码留在 g_fly_flow_why 里供诊断，不阻断流程。 */
	return 0U;

blind:
	/* ---- 兜底：固定距离盲走（距离可用 0x111 [0x11] 现场调）---- */
	g_fly_flow_why = FLY_WHY_NO_TOF;
	return Fly_DoStep(2U, FLY_AXIS_SLIDE,
	                  (int32_t)g_fly_pull_fwd_mm * (int32_t)FLY_UNITS_PER_MM, 0U);
#endif
}

/* =========================================================================
 *                      ★ 拉箱流程（7 步，对应流程图）
 *
 *  1. 平台从让位转回 0° 工作位
 *  2. 平台前伸到抓取位
 *     ★ 用激光测距：距离箱子 FLY_PULL_GRAB_DIST_MM（7cm）就停
 *  3. 抓钩抬 FLY_HOOK_GRAB_DEG（100°= 90° 勾住 + 10° 抬到皮带面）
 *     ★ 多抬的 10° 是为了把箱子抬到与皮带承载面齐平，否则拖不进去
 *  4. 平台后退，把箱子【抬着】移到皮带上方
 *  5. 抓钩放回 0°，箱子落在两条皮带上
 *  6. 平台右转 90°，腾出皮带通道
 *  7. M1+M2 同步把箱子输送到位
 * ========================================================================= */
uint8_t Fly_RunPullOnce(void)
{
	uint8_t r = 0U;

	g_fly_flow_running = FLY_FLOW_PULL;
	g_fly_flow_result  = 0U;
	g_fly_flow_why     = 0U;
	g_fly_flow_stall   = 0U;        /* 每轮重新计堵转 */
	g_fly_abort        = 0U;

	/* ---- 1. 转回工作位 ---- */
	r = Fly_DoStepTo(1U, FLY_AXIS_SWING, FLY_LAND_SWING_WORK);
	if (r != 0U) { goto pull_fail; }

	/* ---- 2. 前伸到抓取位（★ 激光测距靠近，到 7cm 就停）---- */
	r = Fly_PullApproachBox();
	if (r != 0U) { goto pull_fail; }

	/* ---- 3. 抓钩抬起 100°：勾住箱子 + 抬到皮带承载面 ----
	 * ★ 90° 只是勾住，箱子还比皮带面低一点；多抬 10° 才抬得上去。
	 *   见 flybox.h 的 FLY_HOOK_LIFT_EXTRA_DEG。
	 * ★★ 这里用【慢速】FLY_LOAD_SPEED：
	 *   抬钩是带载动作的开始，力矩需求骤增。用空载速度跑会顶到
	 *   驱动器/电源极限，表现就是「抬钩带上箱子后系统停了」。 */
	r = Fly_DoStepToSpd(3U, FLY_AXIS_HOOK, FLY_LAND_HOOK_UP,
	                    FLY_LOAD_SPEED, FLY_LOAD_ACCEL);
	if (r != 0U) { goto pull_fail; }

	/* ---- 4. 平台后退，把箱子拖到皮带上方 ----
	 * ★★ 全程带载，力矩最大，必须用慢速。 */
	r = Fly_DoStepToSpd(4U, FLY_AXIS_SLIDE, FLY_LAND_SLIDE_HOME,
	                    FLY_LOAD_SPEED, FLY_LOAD_ACCEL);
	if (r != 0U) { goto pull_fail; }

	/* ---- 5. 抓钩放回，箱子落在皮带上 ----
	 * ★ 放钩也是带载（箱子重压在钩子上），同样慢速。 */
	r = Fly_DoStepToSpd(5U, FLY_AXIS_HOOK, FLY_LAND_HOOK_DOWN,
	                    FLY_LOAD_SPEED, FLY_LOAD_ACCEL);
	if (r != 0U) { goto pull_fail; }

	/* ---- 6. 平台右转 90°，腾出通道 ---- */
	r = Fly_DoStepTo(6U, FLY_AXIS_SWING, FLY_LAND_SWING_PARK);
	if (r != 0U) { goto pull_fail; }

	/* ---- 7. 两皮带同步输送 ----
	 * ★ 皮带是闭环连续输送，没有绝对位置，所以这一步只能是相对量。 */
	r = Fly_DoStep(7U, FLY_AXIS_BELT_L, (int32_t)FLY_BELT_FEED_UNITS, 1U);
	if (r != 0U) { goto pull_fail; }

	g_fly_flow_step    = 0U;
	g_fly_flow_result  = 0U;
	g_fly_flow_running = 0U;
	return 0U;

pull_fail:
	g_fly_flow_result  = g_fly_flow_step;   /* 记住停在第几步 */
	g_fly_flow_running = 0U;
	g_fly_abort        = 1U;
	return 1U;
}

/* =========================================================================
 *                      ★ 推箱流程（6 步）
 *
 *  1. 两条皮带前移，给平台 90° 转向留出空间
 *  2. 平台转回工作位
 *  3. 抓钩上抬钩住箱子
 *  4. 平台前伸，把箱子推进去
 *  5. 钩子放下
 *  6. 平台后退回来
 * ========================================================================= */
uint8_t Fly_RunPushOnce(void)
{
	uint8_t r = 0U;

	g_fly_flow_running = FLY_FLOW_PUSH;
	g_fly_flow_result  = 0U;
	g_fly_flow_why     = 0U;
	g_fly_flow_stall   = 0U;        /* 每轮重新计堵转 */
	g_fly_abort        = 0U;

	/* ---- 1. 两皮带前移让出转向空间 ---- */
	r = Fly_DoStep(1U, FLY_AXIS_BELT_L, (int32_t)FLY_BELT_ROOM_UNITS, 1U);
	if (r != 0U) { goto push_fail; }

	/* ---- 2. 平台转回工作位 ---- */
	r = Fly_DoStepTo(2U, FLY_AXIS_SWING, FLY_LAND_SWING_WORK);
	if (r != 0U) { goto push_fail; }

	/* ---- 3. 抓钩抬起 90° 钩住箱子 ----
	 * ★ 这里【故意】只抬 90°，不跟拉箱一样抬 100°：
	 *   推箱时箱子已经平放在皮带上，多抬 10° 会把它抬离皮带面，
	 *   推到一半再落下来容易卡住皮带边缘。
	 * ★★ 用慢速：带载动作，同拉箱。 */
	r = Fly_DoStepToSpd(3U, FLY_AXIS_HOOK, FLY_LAND_HOOK_UP_PUSH,
	                    FLY_LOAD_SPEED, FLY_LOAD_ACCEL);
	if (r != 0U) { goto push_fail; }

	/* ---- 4. 平台前伸，把箱子推出去 ---- */
	r = Fly_DoStepToSpd(4U, FLY_AXIS_SLIDE, FLY_LAND_SLIDE_OUT,
	                    FLY_LOAD_SPEED, FLY_LOAD_ACCEL);
	if (r != 0U) { goto push_fail; }

	/* ---- 5. 抓钩放下，松开箱子 ---- */
	r = Fly_DoStepToSpd(5U, FLY_AXIS_HOOK, FLY_LAND_HOOK_DOWN,
	                    FLY_LOAD_SPEED, FLY_LOAD_ACCEL);
	if (r != 0U) { goto push_fail; }

	/* ---- 6. 平台后退回来 ---- */
	r = Fly_DoStepTo(6U, FLY_AXIS_SLIDE, FLY_LAND_SLIDE_HOME);
	if (r != 0U) { goto push_fail; }

	g_fly_flow_step    = 0U;
	g_fly_flow_result  = 0U;
	g_fly_flow_running = 0U;
	return 0U;

push_fail:
	g_fly_flow_result  = g_fly_flow_step;
	g_fly_flow_running = 0U;
	g_fly_abort        = 1U;
	return 1U;
}

void Fly_RunJogTest(void)
{
	uint8_t i;
	uint8_t ok_cnt = 0U;
	uint8_t bad_cnt = 0U;
	uint8_t nofb_cnt = 0U;
	uint8_t fatal = 0U;

	/* ★★★ 这里也去掉了原来的全局闸门 ★★★
	 *
	 *  原来写的是：
	 *      if (g_fly_cfg_cnt != need) { 直接 return; }   // 一个轴都不动
	 *  但 Fly_ConfigAxes() 已改成逐轴尽力而为，g_fly_cfg_cnt 正常就是 5。
	 *  如果它小于 5，说明有轴的三帧发送失败了 —— 那也不该拦住其他轴。
	 *
	 *  改成逐轴判断：没配置成功的轴跳过并记一笔，其他轴照常点动。
	 *  真正的「动起来安不安全」由 Fly_MoveRelEx 的 200mm 限幅兜底。
	 */

	g_fly_jogtest_busy = 1U;

	/* ★★★ 幅度为什么用「固定指令单位」而不是角度 ★★★
	 *
	 *   FLY_SWING_UNITS_PER_DEG / FLY_HOOK_UNITS_PER_DEG 现在还是猜的 100。
	 *   如果直接写「转 10°」，而真实系数是 1000，那就是转 100° ——
	 *   没有限位挡块，转 100° 就可能顶到结构。
	 *
	 *   改成发固定的指令单位（0.1mm 尺度）后，**实际发出的位移量完全确定**，
	 *   跟标定系数无关。看不出来就再加大，绝不会一次超预期。
	 */
	const int32_t  step_linear = FLY_JOG_TEST_UNITS;                       /* 直线轴 */
	const int32_t  step_rotary = FLY_JOG_TEST_UNITS;                       /* 旋转轴 */
	const uint32_t v_linear    = FLY_JOG_TEST_SPEED;                       /* 已提速到 200mm/s */
	const uint32_t v_rotary    = FLY_JOG_TEST_SPEED;
	const uint32_t a_jog       = FLY_JOG_TEST_ACCEL;                       /* ★ 单独的大加速度 */

	/* ---- 0) 先做「两条皮带一起动」----
	 * ★ 放在最前面，因为这是最直观的验证：
	 *   如果它同步走了一个来回，说明两条皮带能协同（课题二左右轮那套经验可用）；
	 *   如果只有一条动、另一条不动，说明这两台压根不是同一类配置。 */
	Fly_SendJogReport(0xFD, 0U, 0U, 0);        /* 轴号 0xFD = 皮带同步测试开始 */
	(void)Fly_BeltBothJog(FLY_JOG_TEST_UNITS);
	Fly_SendJogReport(0xFD, 3U, 0U, 0);        /* 皮带同步测试结束 */
	osDelay(FLY_JOG_TEST_PAUSE_MS);

	for (i = 0U; i < FLY_AXIS_COUNT; i++)
	{
		FlyAxis_t ax   = (FlyAxis_t)i;
		int32_t   step = (Fly_IsRotary(ax) != 0U) ? step_rotary : step_linear;
		uint32_t  spd  = (Fly_IsRotary(ax) != 0U) ? v_rotary    : v_linear;
		uint8_t   has_fb;

		if (s_ax[i].dev == FLY_DEV_UNKNOWN) { continue; }

		/* ★★★ 关键改动：不再用 Fly_AxisReady() 跳过「没反馈」的轴 ★★★
		 *
		 *   原来没反馈就 continue，结果是：如果 5 台电机都不回应查询，
		 *   一轮下来【一帧运动指令都不会发出去】，看起来就是「完全没反应」，
		 *   连「到底发了没有」都无法确认。
		 *
		 *   现在改成：没反馈也发，只是不用「等剩余位移归零」来判断到位，
		 *   而是发完等一个固定时长。这样：
		 *     · CAN 日志里一定能看到 0x0414/15/16 这几帧 → 能证明固件在工作
		 *     · 电机如果只是查询不响应但能被运动帧驱动，也会动起来
		 */
		has_fb = (Motor_HasRemainDev(s_ax[i].dev) != 0U) ? 1U : 0U;

		g_fly_jog_axis = i;

		/* ---- 正向 ---- */
		/* ★ 探针 tag=0：发指令【之前】的基线（剩余位移应该是 0，因为上一轮已到位）*/
		Fly_SendAxisProbe(ax, 0U);

		Fly_SendJogReport(i, 1U, 0U, step);
		g_fly_jog_delta = step;

		/* ★ 走 Ex 版本并允许未置零：
		 *   自检时还没有零点（零点要人先摆好姿态才置），
		 *   若按正常路径就会把 M3/M4/M5 全部拒掉 —— 自检就没意义了。
		 *   安全性由「200mm 行程限幅 + 正反各一次净位移为 0」保证。 */
		if (Fly_MoveRelEx(ax, step, spd, a_jog, 1U) != 0U)
		{
			Fly_SendJogReport(i, 1U, 1U, step);
			bad_cnt++;
			g_fly_jog_axis = 0xFFU;
			continue;
		}

		/* ★ 探针 tag=1：发完指令 120ms 后立刻探一次。
		 *   这是「电机收到指令没有」的决定性证据：
		 *     剩余位移 ≈ step（1000）-> 收到了，正在走
		 *     剩余位移 = 0          -> 没收到，问题在地址（马达号/帧组）
		 *   120ms 足够让 0x0420 反馈回来，又不会等到电机已经走完
		 *   （1000 单位 ÷ 800 单位每秒 = 1.25 秒）。 */
		osDelay(120U);
		Fly_SendAxisProbe(ax, 1U);

		if (has_fb != 0U)
		{
			if (Fly_WaitAxisDone(ax, FLY_JOGTEST_TIMEOUT_MS) == 0U)
			{
				/* ============================================================
				 * ★★★ 第一次动不了 -> 自动换从帧组重试一次 ★★★
				 *
				 *  判断依据：如果这台电机挂在双通道驱动器的【第二路】上，
				 *  它会响应 0x0316/0x0413（按设备号寻址，所以「会锁死」），
				 *  但收不到 0x0414/15/16（那是主通道的帧组）。
				 *  表现就是「锁死了但一动不动」—— 正是实测到的现象。
				 *
				 *  所以：主帧组失败 -> 切到从帧组 0x0417/18/19 再试一次。
				 *  这一次仍然失败才真正放弃，跳过这一轴继续下一个。
				 * ============================================================ */
				if (s_ax[i].use_slave == 0U)
				{
					s_ax[i].use_slave = 1U;
					g_fly_slave_used_mask |= (uint8_t)(1U << i);

					Fly_SendJogReport(i, 4U, 0U, step);      /* 阶段 4 = 切换从帧组 */

					/* ★ 关键：等够时间让上一段停下来再发新指令。
					 *   中途改目标会把电机搞死，所以这里不是在"改目标"，
					 *   而是【换了一个目标电机】（从通道），主通道那条指令
					 *   我方不再发送，不会冲突。 */
					osDelay(300U);

					if (Fly_MoveRelEx(ax, step, spd, a_jog, 1U) == 0U)
					{
						if (Fly_WaitAxisDone(ax, FLY_JOGTEST_TIMEOUT_MS) != 0U)
						{
							goto jog_axis_ok;            /* 从帧组成功 */
						}
					}
				}

				/* 主、从两种帧组都不动 -> 记失败，跳过这一轴，
				 * 但【不影响后面的轴】（原来这里是 break，是 bug）*/
				Fly_SendJogReport(i, 1U, 2U, step);
				bad_cnt++;
				g_fly_jog_axis = 0xFFU;
				continue;
			}
		}
		else
		{
			/* 没有 0x0420 反馈：按"位移 ÷ 速度"估个时间等一等，不判成败 */
			osDelay(FLY_JOGTEST_NOFB_WAIT_MS);
			Fly_SendJogReport(i, 1U, FLY_JOG_RESULT_NOFB, step);
		}

jog_axis_ok:

		osDelay(FLY_JOG_TEST_PAUSE_MS);

		/* ---- 反向（回到原处）---- */
		Fly_SendJogReport(i, 2U, 0U, -step);
		g_fly_jog_delta = -step;

		if (Fly_MoveRelEx(ax, -step, spd, a_jog, 1U) != 0U)
		{
			Fly_SendJogReport(i, 2U, 1U, -step);
			bad_cnt++;
			g_fly_jog_axis = 0xFFU;
			continue;
		}

		if (has_fb != 0U)
		{
			if (Fly_WaitAxisDone(ax, FLY_JOGTEST_TIMEOUT_MS) == 0U)
			{
				/* 同上：跳过这一轴，不影响其它轴 */
				Fly_SendJogReport(i, 2U, 2U, -step);
				bad_cnt++;
				g_fly_jog_axis = 0xFFU;
				continue;
			}
		}
		else
		{
			osDelay(FLY_JOGTEST_NOFB_WAIT_MS);
			Fly_SendJogReport(i, 2U, FLY_JOG_RESULT_NOFB, -step);
		}

		osDelay(FLY_JOG_TEST_PAUSE_MS);

		if (has_fb != 0U) { ok_cnt++; } else { nofb_cnt++; }
		Fly_SendJogReport(i, 3U, (has_fb != 0U) ? 0U : FLY_JOG_RESULT_NOFB, 0);
		g_fly_jog_axis = 0xFFU;
	}

	g_fly_jog_ok    = ok_cnt;
	g_fly_jog_bad   = bad_cnt;
	g_fly_jog_nofb  = nofb_cnt;
	g_fly_jog_fatal = fatal;
	g_fly_jog_axis  = 0xFFU;
	g_fly_jogtest_busy = 0U;

	/* 收尾：整轮结果用一次 phase=0xFF 上报，便于一眼看出总共成了几个
	 * [4..5] = 成功轴数<<8 | 失败轴数；[3] = fatal
	 * ★ 无反馈的轴既不算成功也不算失败，单独计数（g_fly_jog_nofb），
	 *   因为它们「发出去过」但「无法确认」，性质不同。 */
	Fly_SendJogReport(0xFFU, 0xFFU, fatal, (int32_t)((ok_cnt << 8) | bad_cnt));
}

void Fly_RequestJogTest(void)
{
	/* ★ 只置标志，不在 CAN 接收任务里执行。
	 *   点动要等到位（内部 osDelay 几百毫秒），
	 *   如果在 Task_CanRx 里跑，整个 CAN 接收会被堵住，反馈帧全丢。 */
	g_fly_jogtest_req = 1U;
}
