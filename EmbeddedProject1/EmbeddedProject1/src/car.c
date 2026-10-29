/**
  ******************************************************************************
  * @file    car.c
  * @brief   小车运动层（基础版）：航向展开 + 里程计 + 运动状态机（见 car.h）
  *
  *  三个核心：
  *    1) 航向展开 unwrap：陀螺仪输出 -180~+180，正方形要转 1800°，
  *       直接用会在 ±180 处跳变；这里累加相邻差值，得到连续航向。
  *    2) 里程计：电机的 0x0420 是"剩余位移"（还要走多少），不是累计里程。
  *       所以用「绝对指令位置 - 剩余位移」反推当前真实位置：
  *           当前位置 = 指令位置 - 剩余位移
  *       每次下发相对位移 d 时先算当前位置，再令 指令位置 = 当前位置 + d。
  *       ★ 不能简单把 d 累加到指令位置上：电机运动中收到新的相对命令会丢弃旧目标，
  *         那样子未走完的剩余位移会被重复计入（上位机 Motor.cs 里也踩过这个坑）。
  *    3) 状态机：IDLE -> MOVING/TURNING -> DONE/ABORTED，含超时保护。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "car.h"
#include "motor.h"
#include "angle.h"

/* Private define ------------------------------------------------------------*/
#define CAR_PI_F        3.14159265358979f
#define CAR_GYRO_STALE_MS   500U    /* 超过这么久没有新的角度块就认为陀螺仪没在跑 */

/* Private variables ---------------------------------------------------------*/

/* 航向 */
static float    s_yaw;              /* 连续航向，° */
static float    s_last_raw;         /* 上次的原始角度（已乘安装方向）*/
static uint8_t  s_yaw_valid;        /* 是否已建立基准 */
static uint16_t s_last_seq;         /* 上次的 Angle seq */
static uint32_t s_gyro_tick;        /* 上次 seq 变化的时刻 */
static uint8_t  s_gyro_alive;       /* 陀螺仪数据是否在更新 */

/* 里程计：每轮的绝对指令位置（0.1mm）与本地剩余位移估计 */
static int32_t  s_cmd_pos[MOTOR_COUNT];
static int32_t  s_remain_est[MOTOR_COUNT];  /* 本地估计的剩余位移，抵消反馈延迟 */
static uint32_t s_fb_seq[MOTOR_COUNT];      /* 上次采用的反馈序号 */
static uint8_t  s_odo_valid;                /* 是否已建立基准 */

/* 状态机 */
static CarState_t s_state = CAR_IDLE;
static uint32_t s_start_tick;       /* 本段开始时刻 */
static uint32_t s_timeout_ms;       /* 本段超时 */
static int32_t  s_fwd_at_start;     /* 本段开始时的车体前进距离 */
static float    s_yaw_at_start;     /* 本段开始时的航向 */
static int32_t  s_turn_target_cdeg; /* 转向目标，0.01°（整型便于比较）*/
static int32_t  s_seg_target;       /* 本段目标值：位移 0.1mm / 角度 0.01° */
static uint8_t  s_abort_reason;

/* Watch 变量 */
volatile uint8_t  g_car_state        = 0U;
volatile float    g_car_yaw          = 0.0f;
volatile float    g_car_yaw_raw      = 0.0f;
volatile int32_t  g_car_odo_left     = 0;
volatile int32_t  g_car_odo_right    = 0;
volatile int32_t  g_car_odo_forward  = 0;
volatile int32_t  g_car_remain_left  = 0;
volatile int32_t  g_car_remain_right = 0;
volatile int32_t  g_car_seg_target   = 0;
volatile int32_t  g_car_seg_actual   = 0;
volatile uint32_t g_car_elapsed_ms   = 0U;
volatile uint8_t  g_car_abort_reason = 0U;

/* Private function prototypes -----------------------------------------------*/
static void    Car_UpdateYaw(void);
static void    Car_UpdateOdometer(void);
static int32_t Car_WheelPos(uint8_t idx);
static uint8_t Car_SendRel(uint8_t idx, int32_t dist_0p1mm);
static uint32_t Car_EstTimeout(uint32_t dist_0p1mm, uint32_t speed, uint32_t acc);
static void    Car_EnterDone(void);
static void    Car_EnterAbort(uint8_t reason);

/* Exported functions --------------------------------------------------------*/

void Car_Init(void)
{
	s_state          = CAR_IDLE;
	s_start_tick     = 0U;
	s_timeout_ms     = 0U;
	s_fwd_at_start   = 0;
	s_yaw_at_start   = 0.0f;
	s_turn_target_cdeg = 0;
	s_seg_target     = 0;
	s_abort_reason   = 0U;

	s_yaw            = 0.0f;
	s_last_raw       = 0.0f;
	s_yaw_valid      = 0U;
	s_last_seq       = 0U;
	s_gyro_tick      = 0U;
	s_gyro_alive     = 0U;

	s_cmd_pos[MOTOR_LEFT]  = 0;
	s_cmd_pos[MOTOR_RIGHT] = 0;
	s_remain_est[MOTOR_LEFT]  = 0;
	s_remain_est[MOTOR_RIGHT] = 0;
	s_fb_seq[MOTOR_LEFT]  = 0U;
	s_fb_seq[MOTOR_RIGHT] = 0U;
	s_odo_valid      = 0U;

	g_car_state        = (uint8_t)CAR_IDLE;
	g_car_yaw          = 0.0f;
	g_car_yaw_raw      = 0.0f;
	g_car_odo_left     = 0;
	g_car_odo_right    = 0;
	g_car_odo_forward  = 0;
	g_car_remain_left  = 0;
	g_car_remain_right = 0;
	g_car_seg_target   = 0;
	g_car_seg_actual   = 0;
	g_car_elapsed_ms   = 0U;
	g_car_abort_reason = 0U;
}

/**
  * @brief  周期调用（CAR_UPDATE_PERIOD_MS = 20ms）
  *         顺序：先更新航向与里程（传感器），再推进状态机
  */
void Car_Update(void)
{
	uint32_t now = HAL_GetTick();
	int32_t  arc;
	uint8_t  done = 0U;

	Car_UpdateYaw();
	Car_UpdateOdometer();

	switch (s_state)
	{
	case CAR_MOVING:
		g_car_elapsed_ms = now - s_start_tick;

		/* 车体前进距离 = (左 + 右) / 2 */
		arc = (Car_WheelPos(MOTOR_LEFT) + Car_WheelPos(MOTOR_RIGHT)) / 2 - s_fwd_at_start;
		g_car_seg_actual = arc;

		if ((Motor_HasRemain(MOTOR_LEFT) != 0U) && (Motor_HasRemain(MOTOR_RIGHT) != 0U))
		{
			/* 有剩余位移反馈：两个轮子都归零才算到位 */
			if ((Motor_GetRemain(MOTOR_LEFT)  <= CAR_STRAIGHT_TOL_0P1MM) &&
			    (Motor_GetRemain(MOTOR_LEFT)  >= -CAR_STRAIGHT_TOL_0P1MM) &&
			    (Motor_GetRemain(MOTOR_RIGHT) <= CAR_STRAIGHT_TOL_0P1MM) &&
			    (Motor_GetRemain(MOTOR_RIGHT) >= -CAR_STRAIGHT_TOL_0P1MM))
			{
				done = 1U;
			}
		}
		else if (g_car_elapsed_ms >= s_timeout_ms)
		{
			/* 没有反馈就没法判断，只能判失败，避免"假装走完" */
			Car_EnterAbort(3U);
			return;
		}

		if (done != 0U)
		{
			Car_EnterDone();
		}
		else if (g_car_elapsed_ms >= s_timeout_ms)
		{
			Car_EnterAbort(1U);                 /* 超时 */
		}
		break;

	case CAR_TURNING:
		g_car_elapsed_ms = now - s_start_tick;

		/* 航向误差，0.01° 整数比较 */
		arc = (int32_t)((s_yaw - s_yaw_at_start) * 100.0f);
		g_car_seg_actual = arc;

		if (s_gyro_alive != 0U)
		{
			int32_t err = arc - s_turn_target_cdeg;
			int32_t tol = (int32_t)(CAR_TURN_TOL_DEG * 100.0f);

			if ((err <= tol) && (err >= -tol))
			{
				done = 1U;
			}
		}

		if (done != 0U)
		{
			Car_EnterDone();
		}
		else if (g_car_elapsed_ms >= s_timeout_ms)
		{
			Car_EnterAbort((s_gyro_alive != 0U) ? 1U : 3U);   /* 超时 / 陀螺仪没数据 */
		}
		break;

	case CAR_IDLE:
	case CAR_DONE:
	case CAR_ABORTED:
	default:
		break;
	}

	g_car_state = (uint8_t)s_state;
}

uint8_t Car_IsBusy(void)
{
	return ((s_state == CAR_MOVING) || (s_state == CAR_TURNING)) ? 1U : 0U;
}

float Car_GetYaw(void)          { return s_yaw; }
float Car_Wrap180(float deg)    { while (deg > 180.0f) { deg -= 360.0f; } while (deg <= -180.0f) { deg += 360.0f; } return deg; }
int32_t Car_GetOdoLeft(void)    { return Car_WheelPos(MOTOR_LEFT); }
int32_t Car_GetOdoRight(void)   { return Car_WheelPos(MOTOR_RIGHT); }
int32_t Car_GetOdoForward(void) { return (Car_WheelPos(MOTOR_LEFT) + Car_WheelPos(MOTOR_RIGHT)) / 2; }

uint8_t Car_HasWheelFeedback(void)
{
	return ((Motor_HasRemain(MOTOR_LEFT) != 0U) && (Motor_HasRemain(MOTOR_RIGHT) != 0U)) ? 1U : 0U;
}

uint8_t Car_GyroAlive(void) { return s_gyro_alive; }

/**
  * @brief  航向清零：把当前航向当作 0
  */
void Car_ResetYaw(void)
{
	AngleData_t d;

	Angle_GetData(&d);
	s_last_raw  = Car_Wrap180(d.angle_deg) * (float)CAR_YAW_SIGN;
	s_yaw       = 0.0f;
	s_yaw_valid = 1U;
	g_car_yaw     = 0.0f;
	g_car_yaw_raw = d.angle_deg;
}

/**
  * @brief  里程计归零：把电机当前的剩余位移当作基准，之后从 0 开始累计
  *         （直接清成 0 会在下一次 Update 时跳变，所以要对齐到实际位置）
  */
void Car_ResetOdometer(void)
{
	uint8_t i;
	int32_t remain;

	for (i = 0U; i < MOTOR_COUNT; i++)
	{
		remain = (Motor_HasRemain(i) != 0U) ? Motor_GetRemain(i) : 0;
		s_cmd_pos[i]    = remain;               /* 令"指令位置 = 剩余位移" */
		s_remain_est[i] = remain;               /* => 当前位置 = 0，不产生跳变 */
		s_fb_seq[i]     = Motor_GetFeedbackSeq(i);
	}
	s_odo_valid = 1U;

	g_car_odo_left    = 0;
	g_car_odo_right   = 0;
	g_car_odo_forward = 0;
}

/* ------------------------------------------------------------------ 换算 */

int32_t Car_DegToLinear0p1mm(float deg, uint16_t wheel_0p01mm)
{
	float v = deg * CAR_PI_F * (float)wheel_0p01mm / 360.0f * 0.1f;

	return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

float Car_LinearToDeg(int32_t linear_0p1mm, uint16_t wheel_0p01mm)
{
	if (wheel_0p01mm == 0U)
	{
		return 0.0f;
	}
	return (float)linear_0p1mm * 360.0f / (CAR_PI_F * (float)wheel_0p01mm * 0.1f);
}

int32_t Car_TurnArc0p1mm(float deg, float wheel_base_mm)
{
	/* 每轮弧长(0.1mm) = B(mm) × deg × π / 36 */
	float v = (float)wheel_base_mm * deg * CAR_PI_F / 36.0f;

	return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* ------------------------------------------------------------------ 运动指令 */

/**
  * @brief  直线：两轮同向走 dist_0p1mm
  * @param  dist_0p1mm   位移 0.1mm（负 = 后退）
  * @param  speed_0p1mmps 速度 0.1mm/s
  * @param  acc_0p1mmps2  加速度 0.1mm/s^2
  * @retval 0 已接受，1 拒绝
  */
uint8_t Car_StartStraight(int32_t dist_0p1mm, uint32_t speed_0p1mmps, uint32_t acc_0p1mmps2)
{
	int32_t absdist = (dist_0p1mm >= 0) ? dist_0p1mm : -dist_0p1mm;

	if ((Car_IsBusy() != 0U) || (absdist == 0) || (dist_0p1mm == (int32_t)0x80000000))
	{
		return 1U;
	}
	if ((speed_0p1mmps == 0U) || (acc_0p1mmps2 == 0U))
	{
		return 1U;
	}
	if (Motor_IsReady() == 0U)
	{
		return 1U;                              /* 电机还没初始化完，不许动 */
	}

	/* 先设加速度，再下发位置（顺序不能反）*/
	if ((Motor_SendAcc(MOTOR_LEFT,  acc_0p1mmps2) != 0U) ||
	    (Motor_SendAcc(MOTOR_RIGHT, acc_0p1mmps2) != 0U))
	{
		return 1U;
	}
	if ((Car_SendRel(MOTOR_LEFT,  dist_0p1mm) != 0U) ||
	    (Car_SendRel(MOTOR_RIGHT, dist_0p1mm) != 0U))
	{
		return 1U;
	}

	s_start_tick     = HAL_GetTick();
	s_timeout_ms     = Car_EstTimeout(absdist, speed_0p1mmps, acc_0p1mmps2);
	s_fwd_at_start   = Car_GetOdoForward();
	s_seg_target     = dist_0p1mm;
	s_abort_reason   = 0U;
	s_state          = CAR_MOVING;

	g_car_seg_target   = s_seg_target;
	g_car_seg_actual   = 0;
	g_car_elapsed_ms   = 0U;
	g_car_abort_reason = 0U;
	g_car_state        = (uint8_t)s_state;
	return 0U;
}

/**
  * @brief  原地转向：两轮反向等量转动，到位用陀螺仪航向判断
  * @param  deg 车体转角（°），正 = 逆时针（与 CAR_YAW_SIGN 一致）
  * @retval 0 已接受，1 拒绝
  */
uint8_t Car_StartTurn(float deg, uint32_t speed_0p1mmps, uint32_t acc_0p1mmps2)
{
	int32_t arc;
	int32_t absarc;
	float   absdeg = (deg >= 0.0f) ? deg : -deg;

	if ((Car_IsBusy() != 0U) || (absdeg < 0.1f))
	{
		return 1U;
	}
	if ((speed_0p1mmps == 0U) || (acc_0p1mmps2 == 0U))
	{
		return 1U;
	}
	if (Motor_IsReady() == 0U)
	{
		return 1U;
	}

	arc    = Car_TurnArc0p1mm(absdeg, CAR_WHEEL_BASE_MM);
	absarc = arc;

	if (arc == 0)
	{
		return 1U;
	}

	/* 先设加速度，再下发位置（顺序不能反）*/
	if ((Motor_SendAcc(MOTOR_LEFT,  acc_0p1mmps2) != 0U) ||
	    (Motor_SendAcc(MOTOR_RIGHT, acc_0p1mmps2) != 0U))
	{
		return 1U;
	}

	/* 逆时针：左轮后退、右轮前进（按轮子正方向定义，方向符号在 car_cfg.h 里配）*/
	if (deg >= 0.0f)
	{
		if ((Car_SendRel(MOTOR_LEFT,  -absarc) != 0U) ||
		    (Car_SendRel(MOTOR_RIGHT,  absarc) != 0U))
		{
			return 1U;
		}
	}
	else
	{
		if ((Car_SendRel(MOTOR_LEFT,   absarc) != 0U) ||
		    (Car_SendRel(MOTOR_RIGHT, -absarc) != 0U))
		{
			return 1U;
		}
	}

	s_start_tick       = HAL_GetTick();
	s_timeout_ms       = Car_EstTimeout(absarc, speed_0p1mmps, acc_0p1mmps2);
	s_yaw_at_start     = s_yaw;
	s_turn_target_cdeg = (int32_t)(deg * 100.0f);
	s_seg_target       = s_turn_target_cdeg;
	s_abort_reason     = 0U;
	s_state            = CAR_TURNING;

	g_car_seg_target   = s_seg_target;
	g_car_seg_actual   = 0;
	g_car_elapsed_ms   = 0U;
	g_car_abort_reason = 0U;
	g_car_state        = (uint8_t)s_state;
	return 0U;
}

/**
  * @brief  停车：给两轮发"相对位移 0"，让电机保持在当前位置
  *         不调用 0x0413 释放，避免坡道溜车；需要释放时另行调用 Motor_SendLock
  */
void Car_Stop(void)
{
	(void)Car_SendRel(MOTOR_LEFT,  0);
	(void)Car_SendRel(MOTOR_RIGHT, 0);
}

CarState_t Car_GetState(void) { return s_state; }

const char *Car_GetStateName(void)
{
	switch (s_state)
	{
	case CAR_IDLE:    return "IDLE";
	case CAR_MOVING:  return "MOVING";
	case CAR_TURNING: return "TURNING";
	case CAR_DONE:    return "DONE";
	case CAR_ABORTED: return "ABORTED";
	default:          return "?";
	}
}

int32_t Car_GetLastTarget(void) { return s_seg_target; }
int32_t Car_GetLastActual(void) { return g_car_seg_actual; }

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  更新连续航向：把 -180~+180 的原始角度展开
  *         相邻两帧的差值一定很小，用 wrap180 求差值再累加，就不会在 ±180 处跳变
  */
static void Car_UpdateYaw(void)
{
	AngleData_t d;
	float raw;
	float diff;

	Angle_GetData(&d);

	raw = Car_Wrap180(d.angle_deg) * (float)CAR_YAW_SIGN;
	g_car_yaw_raw = d.angle_deg;

	/* 用 seq 变化判断陀螺仪是否在出数据 */
	if (d.seq != s_last_seq)
	{
		s_last_seq  = d.seq;
		s_gyro_tick = HAL_GetTick();
		s_gyro_alive = 1U;
	}
	else if ((HAL_GetTick() - s_gyro_tick) > CAR_GYRO_STALE_MS)
	{
		s_gyro_alive = 0U;
	}

	if (s_yaw_valid == 0U)
	{
		s_yaw       = raw;
		s_last_raw  = raw;
		s_yaw_valid = 1U;
	}
	else
	{
		diff = Car_Wrap180(raw - s_last_raw);       /* 本帧的增量，跨 ±180 也不会错 */
		s_yaw += diff;
		s_last_raw = raw;
	}

	g_car_yaw = s_yaw;
}

/**
  * @brief  更新里程计：当前位置 = 指令位置 - 剩余位移
  *         没有反馈的电机保持上一次的值，不做推算（宁可显示不动，也不显示错的数据）
  */
static void Car_UpdateOdometer(void)
{
	uint8_t i;
	uint32_t seq;

	if (s_odo_valid == 0U)
	{
		Car_ResetOdometer();
	}

	/* 只有来了新的反馈帧才更新剩余位移，否则沿用本地估计（避免把旧值当新值）*/
	for (i = 0U; i < MOTOR_COUNT; i++)
	{
		if (Motor_HasRemain(i) == 0U)
		{
			continue;
		}
		seq = Motor_GetFeedbackSeq(i);
		if (seq != s_fb_seq[i])
		{
			s_fb_seq[i]     = seq;
			s_remain_est[i] = Motor_GetRemain(i);
		}
	}

	g_car_odo_left    = Car_WheelPos(MOTOR_LEFT);
	g_car_odo_right   = Car_WheelPos(MOTOR_RIGHT);
	g_car_odo_forward = (g_car_odo_left + g_car_odo_right) / 2;
	g_car_remain_left  = Motor_GetRemain(MOTOR_LEFT);
	g_car_remain_right = Motor_GetRemain(MOTOR_RIGHT);
}

/**
  * @brief  某个轮的当前实际位置（0.1mm）
  * @retval 有反馈时返回 指令位置 - 剩余位移；无反馈返回 0（不猜）
  */
static int32_t Car_WheelPos(uint8_t idx)
{
	if (idx >= MOTOR_COUNT)
	{
		return 0;
	}

	/* 指令位置 - 剩余位移估计 = 已经走过的距离 */
	return s_cmd_pos[idx] - s_remain_est[idx];
}

/**
  * @brief  下发相对位移，并同步更新"绝对指令位置"
  *         指令位置 = 当前位置 + d = (指令位置 - 剩余位移) + d
  *         同时把本地剩余位移估计设为 d，抵消反馈帧到达前的延迟：
  *         否则发完立刻读，电机还没回帧，里程会先跳 +d 再跳回来。
  */
static uint8_t Car_SendRel(uint8_t idx, int32_t dist_0p1mm)
{
	int32_t cur;

	if (idx >= MOTOR_COUNT)
	{
		return 1U;
	}

	cur = Car_WheelPos(idx);                    /* 必须在发送之前取 */

	if (Motor_SendPosRel(idx, dist_0p1mm) != 0U)
	{
		return 1U;
	}

	s_cmd_pos[idx]    = cur + dist_0p1mm;
	s_remain_est[idx] = dist_0p1mm;
	return 0U;
}

/**
  * @brief  估算本段超时：运动时间 + 加速段 + 余量
  */
static uint32_t Car_EstTimeout(uint32_t dist_0p1mm, uint32_t speed, uint32_t acc)
{
	float t_move;
	float t_ramp;
	uint32_t t;

	if ((speed == 0U) || (acc == 0U))
	{
		return CAR_MIN_TIMEOUT_MS;
	}

	t_move = (float)dist_0p1mm / (float)speed * 1000.0f;    /* ms */
	t_ramp = (float)speed / (float)acc * 1000.0f;           /* ms */

	t = (uint32_t)(t_move + t_ramp) + CAR_TIMEOUT_MARGIN_MS;
	if (t < CAR_MIN_TIMEOUT_MS)
	{
		t = CAR_MIN_TIMEOUT_MS;
	}
	return t;
}

static void Car_EnterDone(void)
{
	Car_Stop();
	s_state = CAR_DONE;
	g_car_state = (uint8_t)s_state;
}

static void Car_EnterAbort(uint8_t reason)
{
	Car_Stop();
	s_abort_reason = reason;
	s_state = CAR_ABORTED;
	g_car_abort_reason = reason;
	g_car_state = (uint8_t)s_state;
}
