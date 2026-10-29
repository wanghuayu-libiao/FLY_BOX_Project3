/**
  ******************************************************************************
  * @file    angle.c
  * @brief   角度计算与零偏处理
  *
  *  角度：每 1ms 读一次角速度原始值（24 位，实测 17920 LSB/(°/s)，手册为 71680），
  *        10 点平均成 1 个 10ms 块，扣零偏后用梯形法对时间积分，结果归一化到 -180° ~ +180°。
  *
  *  零偏（芯片 25℃ 零偏最大 ±1°/s，随温度还会漂 ±5°/s，不处理 7 秒就会积出 7°）：
  *   1) 上电标定：等芯片输出稳定(>=200ms) 后，静止采样 ANGLE_CALIB_SAMPLES 个点取平均。
  *      采样期间若发现在动（峰峰值超限），丢弃重来。
  *   2) 零偏 + 温度补偿跟踪：零偏模型 B(T) = b + k*(T - Tr)，b 是零偏，k 是温度系数(LSB/℃)，Tr 是参考温度。
  *      每个 1s 静止窗口用 2 状态卡尔曼滤波同时更新 b 和 k（k 只有温度变化时才会被学到）；
  *      窗口之间——包括电机转动、没有静止窗口可用的时候——零偏按 k 随温度外推，这才是抑制温漂的关键。
  *      静止判定：窗口峰峰值很小，且窗口均值与“预测零偏”的偏差很小。偏差较大（≈0.15~2°/s）的窗口
  *      不立即接受（可能是慢转动），连续很多个窗口都这样才当成零偏接受。
  *      振动：峰峰值大但均值仍接近零偏的窗口也用放大后的测量噪声慢速更新（否则零偏永远学不到）。
  *   3) 抗振动：只做不掩盖真实运动的处理——
  *        a) 1kHz 过采样 + 10 点平均：50Hz 以上的振动不再折叠成低频/假零偏，白噪声按 √10 下降，
  *           积分随机游走 σ·√(T·Δt) 同比下降（这是唯一能真正减小漂移的软件手段）；
  *        b) 振动窗口的零偏用加大测量方差的方式慢速学习，零均值的振动不污染零偏。
  *      明确不做的事：不冻结积分、不把角度“锁定”。角度始终是真积分，读数可信。
  *      若仍有残漂，只能靠降低陀螺带宽、继续提高采样率、机械减振或电机位置融合（绝对基准）。
  *   4) 命令重标：Angle_RequestRecalibrate()（只重置 b，已学到的 k 保留）。
  *   与 GyroHostCS/BiasTracker.cs 逐行对应，调参和仿真在上位机那边做。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "angle.h"
#include "protocol.h"
#include "xv7001bb.h"

/* Private define ------------------------------------------------------------*/
#define ANGLE_DT_S              0.010f      /* 采样周期（与任务周期一致） */
#define ANGLE_SCALE_LSB_DPS     17920.0f    /* 实测标定：手转 90° 显示 22.5°，即手册值 71680 的 1/4（原因未明） */
#define ANGLE_SIGN              1.0f        /* 安装方向与期望正方向相反时改成 -1.0f */
#define ANGLE_REG_STATUS        0x04U       /* StsRd 状态寄存器 */
#define ANGLE_SLOW_DIV          10U         /* 温度/状态每 10 个周期(100ms)读一次 */

#define ANGLE_WARMUP_MS         300U        /* 手册 tSTA=200ms，留余量 */
#define ANGLE_CALIB_SAMPLES     200         /* 标定采样点数（2s） */
#define ANGLE_CALIB_MAX_PP      35840       /* 标定时峰峰值上限 ≈2°/s，超过认为在动 */

#define ANGLE_WIN_SAMPLES       100         /* 零偏更新窗口（1s） */
#define ANGLE_TEMP_ALPHA        0.25f       /* 温度每 100ms 读一次，一阶低通系数 */
#define ANGLE_STILL_MAX_PP      8960        /* 窗口峰峰值上限 ≈0.5°/s（静置噪声峰峰值约 0.1°/s） */
#define ANGLE_TIGHT_DEV         2688.0f     /* 窗口均值与预测零偏的偏差 ≤ 此值（≈0.15°/s）：直接跟踪 */
#define ANGLE_WIDE_DEV          35840.0f    /* TIGHT~WIDE（≈2°/s）之间：连续 PERSIST 个窗口才接受；更大：判为转动 */
#define ANGLE_PERSIST_WINDOWS   40          /* 标定 20s 以后需要连续的窗口数 */
#define ANGLE_EARLY_WINDOWS     20          /* 标定后的前 20 个窗口里…… */
#define ANGLE_EARLY_PERSIST     8           /* ……只需要连续 8 个窗口：标定时被带偏的零偏要尽快纠正 */

/* 抗振动（不冻结积分，只做真实信号处理） */
#define ANGLE_DECIM             10U         /* 过采样倍数：1kHz 采样，10 点平均后输出 100Hz */
#define ANGLE_USE_MEDIAN3       0U          /* 1 = 积分前做 3 点中值；已做过采样平均，默认不用 */
#define ANGLE_VIB_MAX_DEV       8960.0f     /* 振动窗口允许的“均值-预测零偏”偏差 ≈0.5°/s，超过当转动 */

/* 卡尔曼参数（原始值 LSB 单位，与 BiasTracker.cs 一致） */
#define ANGLE_KF_R              8100.0f     /* 窗口均值噪声方差 ≈(0.005°/s)² */
#define ANGLE_KF_R_VIB          81000.0f    /* 振动窗口的测量方差 ≈(0.016°/s)²：增益 ≈0.07，时间常数 ≈14s */
#define ANGLE_KF_Q_BIAS         405.0f      /* 零偏随机游走（每窗口），稳态增益 ≈0.2 */
#define ANGLE_KF_Q_SLOPE        4.0f        /* 温度系数随机游走（每窗口） */
#define ANGLE_KF_K_MAX          2688.0f     /* |k| 上限 ≈0.15°/s/℃ */
#define ANGLE_KF_K_INIT_VAR     1.0e6f      /* k 初始方差 (1000 LSB/℃)² */

#define ANGLE_SAT_RAW           7040000     /* 接近 24 位满量程视为饱和（与灵敏度系数无关） */

/* Private types -------------------------------------------------------------*/
typedef enum
{
	STATE_WARMUP = 0,                       /* 等待芯片输出稳定 */
	STATE_CALIB,                            /* 静止标定零偏 */
	STATE_RUN                               /* 正常积分 */
} AngleState_t;

/* Private variables ---------------------------------------------------------*/
static AngleState_t s_state = STATE_WARMUP;
static float        s_angle = 0.0f;
static float        s_rate = 0.0f;
static float        s_prev_rate = 0.0f;
static float        s_bias_raw = 0.0f;      /* 当前温度下的零偏，单位：原始值 LSB（每个采样周期由 b、k、温度算出） */
static float        s_temp = 0.0f;
static float        s_tf = 0.0f;            /* 低通后的温度 */
static uint8_t      s_tf_valid = 0U;

/* 零偏 + 温度补偿：B(T) = s_b + s_k * (T - s_tr) */
static float        s_b = 0.0f;             /* 在 s_tr 温度下的零偏，LSB */
static float        s_k = 0.0f;             /* 温度系数，LSB/℃（在线学习，上电为 0） */
static float        s_tr = 0.0f;            /* 参考温度，℃ */
static float        s_p00 = ANGLE_KF_R;     /* 卡尔曼协方差 [b, k] */
static float        s_p01 = 0.0f;
static float        s_p11 = ANGLE_KF_K_INIT_VAR;
static uint16_t     s_age = 0U;             /* 标定后经过的窗口数 */
static uint16_t     s_big = 0U;             /* 连续“偏差较大但很安静”的窗口数 */
static int32_t      s_med = 0;              /* 3 点中值去尖峰后的原始值 */
static int32_t      s_med1 = 0, s_med2 = 0;
static uint8_t      s_med_n = 0U;
static int32_t      s_acc_sum = 0;          /* 过采样累加（1kHz 采样，ANGLE_DECIM 点平均成 1 个 10ms 块） */
static uint8_t      s_acc_n = 0U;
static uint8_t      s_status = 0U;
static uint8_t      s_saturated = 0U;
static uint8_t      s_slow_cnt = 0U;
static uint16_t     s_seq = 0U;

static int32_t      s_cal_n = 0;
static int32_t      s_cal_sum = 0;
static int32_t      s_cal_min = 0;
static int32_t      s_cal_max = 0;

static int32_t      s_win_n = 0;
static int32_t      s_win_sum = 0;
static int32_t      s_win_min = 0;
static int32_t      s_win_max = 0;
static float        s_win_tsum = 0.0f;

/* ★ 标度因子 / 时间基 自检 */
static float        s_angle_total = 0.0f;   /* 不回绕累计角度 ° */
static uint32_t     s_blk = 0U;             /* 完成的 10ms 块数 */
static uint32_t     s_t0  = 0U;             /* 进入 RUN 态时的 tick */

static volatile uint8_t s_req_zero = 0U;
static volatile uint8_t s_req_recal = 0U;
static volatile uint8_t s_learn_en = 1U;    /* 零偏学习开关，默认开 */
static volatile float   s_scale_k  = 1.0f;  /* 角速度标度修正系数，默认 1.0 = 不修正 */
static uint8_t          s_win_dirty = 0U;   /* 本窗口里出现过"学习关"，整窗作废 */

static AngleData_t  s_pub;                  /* 对外发布的快照 */

/* Private functions ---------------------------------------------------------*/

static void StartCalibration(void)
{
	s_state = STATE_CALIB;
	s_cal_n = 0;
	s_cal_sum = 0;
	s_rate = 0.0f;
	s_prev_rate = 0.0f;
}

#if (ANGLE_USE_MEDIAN3 != 0U)
/**
  * @brief  3 点中值：只削掉单点尖脉冲，不改直流、不改趋势（默认关闭：过采样平均已能压制尖峰）
  */
static int32_t Median3(int32_t a, int32_t b, int32_t c)
{
	int32_t t;

	if (a > b) { t = a; a = b; b = t; }
	if (b > c) { b = c; }
	if (a > b) { b = a; }
	return b;
}
#endif

/**
  * @brief  一个 1s 窗口结束：判断是否静止，并用卡尔曼滤波更新零偏 b 和温度系数 k。
  *         与 GyroHostCS/BiasTracker.cs 的 Step() 窗口结束部分逐行对应。
  */
static void BiasWindowUpdate(void)
{
	float mean = (float)s_win_sum / (float)ANGLE_WIN_SAMPLES;
	float tw = s_win_tsum / (float)ANGLE_WIN_SAMPLES;
	float dT = tw - s_tr;
	float pred = s_b + s_k * dT;
	float dev = mean - pred;
	float adev = (dev < 0.0f) ? -dev : dev;
	uint16_t need = (s_age < ANGLE_EARLY_WINDOWS) ? ANGLE_EARLY_PERSIST : ANGLE_PERSIST_WINDOWS;
	float p00, p01, p11, s, g0, g1, new_k, r_meas;

	if (s_age < 0xFFFFU)
	{
		s_age++;
	}

	r_meas = ANGLE_KF_R;
	if (adev > ANGLE_WIDE_DEV)
	{
		s_big = 0U;                         /* 均值离预测零偏太远：在转动 */
		return;
	}

	if ((s_win_max - s_win_min) > ANGLE_STILL_MAX_PP)
	{
		/* 峰峰值大 = 振动。振动零均值，1s 均值仍无偏地反映零偏，但单窗均值噪声大得多，
		   所以放大测量方差慢速平均（≈14s），不会因为峰峰值大就完全停止学习零偏。
		   偏差超过 0.5°/s 仍认为是真实转动。 */
		if (adev > ANGLE_VIB_MAX_DEV)
		{
			s_big = 0U;
			return;
		}
		r_meas = ANGLE_KF_R_VIB;
	}
	else if (adev > ANGLE_TIGHT_DEV)
	{
		s_big++;
		if (s_big < need)
		{
			return;                         /* 偏差较大但还不确定是零偏还是慢转动：先观察 */
		}
		s_b = mean;                         /* 持续很久且峰峰值很小：当成零偏接受（k 不动） */
		s_tr = tw;
		s_p00 = ANGLE_KF_R;
		s_p01 = 0.0f;
		s_big = 0U;
		return;
	}
	s_big = 0U;

	/* 预测 */
	p00 = s_p00 + 2.0f * dT * s_p01 + dT * dT * s_p11 + ANGLE_KF_Q_BIAS;
	p01 = s_p01 + dT * s_p11;
	p11 = s_p11 + ANGLE_KF_Q_SLOPE;

	/* 更新 */
	s = p00 + r_meas;
	g0 = p00 / s;
	g1 = p01 / s;
	new_k = s_k + g1 * dev;
	if (new_k > ANGLE_KF_K_MAX)  { new_k = ANGLE_KF_K_MAX; }
	if (new_k < -ANGLE_KF_K_MAX) { new_k = -ANGLE_KF_K_MAX; }

	s_b = pred + g0 * dev;
	s_k = new_k;
	s_p00 = (1.0f - g0) * p00;
	s_p01 = (1.0f - g0) * p01;
	s_p11 = p11 - g1 * p01;
	if (s_p11 < 1.0f)
	{
		s_p11 = 1.0f;
	}
	s_tr = tw;
}

static void Publish(void)
{
	AngleData_t tmp;
	uint32_t primask;

	tmp.angle_deg = s_angle;
	tmp.rate_dps  = s_rate;
	tmp.temp_c    = s_temp;
	tmp.bias_dps  = s_bias_raw / ANGLE_SCALE_LSB_DPS;
	tmp.temp_coef = s_k / ANGLE_SCALE_LSB_DPS;
	tmp.seq       = s_seq;
	tmp.status    = s_status;
	tmp.flags     = 0U;

	if (s_state == STATE_RUN)
	{
		tmp.flags |= PROTO_FLAG_CALIBRATED;
	}
	if (s_state == STATE_CALIB)
	{
		tmp.flags |= PROTO_FLAG_CALIBRATING;
	}
	if ((s_status == 0x00U) || (s_status == 0xFFU))
	{
		tmp.flags |= PROTO_FLAG_COMM_FAULT;
	}
	if (s_saturated != 0U)
	{
		tmp.flags |= PROTO_FLAG_SATURATED;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	s_pub = tmp;
	__set_PRIMASK(primask);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化：先读一次状态和温度，使第一帧数据就带有效值（需在 XV7001bb_Init() 之后调用）
  */
void Angle_Init(void)
{
	s_state = STATE_WARMUP;
	s_angle = 0.0f;
	s_rate = 0.0f;
	s_prev_rate = 0.0f;
	s_bias_raw = 0.0f;
	s_b = 0.0f;
	s_k = 0.0f;
	s_p00 = ANGLE_KF_R;
	s_p01 = 0.0f;
	s_p11 = ANGLE_KF_K_INIT_VAR;
	s_age = 0U;
	s_big = 0U;
	s_saturated = 0U;
	s_slow_cnt = 0U;
	s_seq = 0U;
	s_acc_sum = 0;
	s_acc_n = 0U;
	s_med_n = 0U;
	s_req_zero = 0U;
	s_req_recal = 0U;

	s_status = XV7001bb_ReadReg(ANGLE_REG_STATUS);
	s_temp = XV7001bb_ReadTmp();
	s_tf = s_temp;
	s_tr = s_temp;
	s_tf_valid = 1U;

	Publish();
}

/**
  * @brief  每 ANGLE_SAMPLE_PERIOD_MS（1ms）调用一次：采样角速度，每 ANGLE_DECIM 次平均成一个 10ms 块，
  *         然后在这个块上更新零偏、积分角度。
  * @retval 1 = 本次调用刚好完成一个 10ms 输出块（新数据已 Publish），0 = 只是采了一个样
  */
uint8_t Angle_Step(void)
{
	int32_t raw;
	float rate;

	/* 先处理其它任务发来的请求 */
	if (s_req_recal != 0U)
	{
		s_req_recal = 0U;
		StartCalibration();
	}
	if (s_req_zero != 0U)
	{
		s_req_zero = 0U;
		s_angle = 0.0f;
		s_saturated = 0U;
	}

	raw = XV7001bb_ReadAngle();

	/* ---- 过采样：1kHz 采样 + 10 点平均 = 100Hz 输出 ----
	   这是整个抗振里唯一能真正减小噪声的环节：
	     1) 陀螺 50Hz 以上的振动/噪声不再被 100Hz 采样折叠成低频和假零偏（混叠无法被任何后续滤波去掉）；
	     2) 白噪声均值降 √10，积分随机游走（σ·√(T·Δt)）按同样比例下降。
	   若陀螺内部数据率本来就只有 100Hz，10 个采样值相同，平均值=原值，无副作用。 */
	s_acc_sum += raw;
	s_acc_n++;
	if (s_acc_n < ANGLE_DECIM)
	{
		return 0U;                                      /* 还没凑满一个 10ms 输出块 */
	}
	raw = s_acc_sum / (int32_t)ANGLE_DECIM;
	s_acc_sum = 0;
	s_acc_n = 0U;

	/* 温度与状态变化慢，降频读取以减少 SPI 占用 */
	s_slow_cnt++;
	if (s_slow_cnt >= ANGLE_SLOW_DIV)
	{
		s_slow_cnt = 0U;
		s_status = XV7001bb_ReadReg(ANGLE_REG_STATUS);
		s_temp = XV7001bb_ReadTmp();
		s_tf += ANGLE_TEMP_ALPHA * (s_temp - s_tf);     /* 温度低通，去掉 1/16℃ 量化抖动 */
	}

	switch (s_state)
	{
	case STATE_WARMUP:
		if (HAL_GetTick() >= ANGLE_WARMUP_MS)
		{
			StartCalibration();
		}
		break;

	case STATE_CALIB:
		if (s_cal_n == 0)
		{
			s_cal_min = raw;
			s_cal_max = raw;
		}
		else
		{
			if (raw < s_cal_min) { s_cal_min = raw; }
			if (raw > s_cal_max) { s_cal_max = raw; }
		}
		s_cal_sum += raw;
		s_cal_n++;

		if ((s_cal_max - s_cal_min) > ANGLE_CALIB_MAX_PP)
		{
			s_cal_n = 0;                    /* 在动：丢弃已采样的点，重新开始 */
			s_cal_sum = 0;
		}
		else if (s_cal_n >= ANGLE_CALIB_SAMPLES)
		{
			s_b = (float)s_cal_sum / (float)s_cal_n;         /* 标定时的零偏；已学到的 k 保留 */
			s_tr = s_tf;
			s_p00 = ANGLE_KF_R;
			s_p01 = 0.0f;
			s_age = 0U;
			s_big = 0U;
			s_med_n = 0U;
			s_bias_raw = s_b;
			s_angle = 0.0f;
			s_rate = 0.0f;
			s_prev_rate = 0.0f;
			s_saturated = 0U;
			s_win_n = 0;

			/* ★ 自检基准清零：从"进入 RUN 态"这一刻开始算累计角度和时间 */
			s_angle_total = 0.0f;
			s_blk = 0U;
			s_t0  = HAL_GetTick();

			s_state = STATE_RUN;
		}
		break;

	case STATE_RUN:
	default:
#if (ANGLE_USE_MEDIAN3 != 0U)
		/* 3 点中值去尖脉冲：振动是对称的，只有偶发尖峰会让积分单向累积。
		   只延迟 10ms，对几十 Hz 以下的真实转动几乎无影响，不隐藏任何漂移。 */
		if (s_med_n < 2U)
		{
			s_med1 = raw;
			s_med2 = raw;
			s_med_n = 2U;
		}
		s_med = Median3(raw, s_med1, s_med2);
		s_med1 = s_med2;
		s_med2 = raw;
#else
		s_med = raw;
#endif

		s_bias_raw = s_b + s_k * (s_tf - s_tr);          /* 当前温度下的零偏 */
		rate = ANGLE_SIGN * ((float)s_med - s_bias_raw) / ANGLE_SCALE_LSB_DPS;
		rate *= s_scale_k;                               /* 标度 ppm 修正（默认 1.0，不改变原行为）*/
		s_rate = rate;

		/* 诚实积分：角度始终是角速度的真实积分，不冻结、不锁定 */
		s_angle += 0.5f * (s_prev_rate + rate) * ANGLE_DT_S;
		while (s_angle >= 180.0f) { s_angle -= 360.0f; }
		while (s_angle < -180.0f) { s_angle += 360.0f; }

		/* ★ 标度因子自检：同一个增量，但【不回绕】，转 10 圈能读到 3600° 左右。
		 *   用来判断 ANGLE_SCALE_LSB_DPS 准不准（见 angle.h 的 Angle_GetTotal）。 */
		s_angle_total += 0.5f * (s_prev_rate + rate) * ANGLE_DT_S;

		if (s_blk < 0xFFFFFFFFU)
		{
			s_blk++;
		}

		s_prev_rate = rate;

		if ((raw >= ANGLE_SAT_RAW) || (raw <= -ANGLE_SAT_RAW))
		{
			s_saturated = 1U;
		}

		/* 静止窗口：统计均值/峰峰值/温度，窗口结束时更新零偏和温度系数 */
		if (s_win_n == 0)
		{
			s_win_min = s_med;
			s_win_max = s_med;
			s_win_sum = 0;
			s_win_tsum = 0.0f;
		}
		else
		{
			if (s_med < s_win_min) { s_win_min = s_med; }
			if (s_med > s_win_max) { s_win_max = s_med; }
		}
		s_win_sum += s_med;
		s_win_tsum += s_tf;
		s_win_n++;

		/* 学习关闭期间的窗口整窗作废：窗口可能跨在"走"和"停"之间，
		 * 半个窗口的运动数据混进去，均值就不是零偏了。 */
		if (s_learn_en == 0U)
		{
			s_win_dirty = 1U;
		}

		if (s_win_n >= ANGLE_WIN_SAMPLES)
		{
			if (s_win_dirty != 0U)
			{
				s_win_dirty = 0U;
				s_big = 0U;                 /* 不让"偏差较大"的连续计数跨过这个无效窗口 */
			}
			else
			{
				BiasWindowUpdate();
			}
			s_win_n = 0;
		}
		break;
	}

	s_seq++;
	Publish();
	return 1U;
}

/**
  * @brief  取一份一致的数据快照（可在任意任务里调用）
  */
void Angle_GetData(AngleData_t *out)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	*out = s_pub;
	__set_PRIMASK(primask);
}

void Angle_RequestZero(void)
{
	s_req_zero = 1U;
}

/**
  * @brief  ★ 标度因子 / 时间基 自检（陀螺仪到底准不准）
  * @param  angle_total_deg  不回绕累计角度 °（转 10 圈约 ±3600）
  * @param  ppm              时间基误差 ppm
  *                          (名义时间 - 实际时间) / 实际时间 × 1e6
  *                          名义时间 = 完成的 10ms 块数 × 10ms
  *                          ±4300 ppm 正好等于「每 360° 偏 1.55°」
  * @param  secs             进入 RUN 态后经过的秒数
  * @param  blocks           完成的 10ms 块数
  */
void Angle_GetTotal(float *angle_total_deg, int16_t *ppm, uint16_t *secs, uint32_t *blocks)
{
	uint32_t primask = __get_PRIMASK();
	float    at;
	uint32_t blk;

	__disable_irq();
	at  = s_angle_total;
	blk = s_blk;
	__set_PRIMASK(primask);

	if (angle_total_deg != 0) { *angle_total_deg = at; }
	if (blocks          != 0) { *blocks = blk; }

	if (ppm != 0)
	{
		/* 名义时间 = blk × 10ms；实际时间 = 从 RUN 态开始到现在的 tick 差 */
		uint32_t actual_ms = HAL_GetTick() - s_t0;
		uint32_t nominal_ms = blk * ANGLE_OUTPUT_PERIOD_MS;
		float    p;

		if (actual_ms < 100U)
		{
			*ppm = 0;                        /* 时间太短，比值没意义 */
		}
		else
		{
			p = ((float)nominal_ms - (float)actual_ms) / (float)actual_ms * 1000000.0f;
			if (p >  32000.0f) { p =  32000.0f; }
			if (p < -32000.0f) { p = -32000.0f; }
			*ppm = (int16_t)p;
		}
	}

	if (secs != 0)
	{
		uint32_t s = (HAL_GetTick() - s_t0) / 1000U;
		*secs = (s > 65535U) ? 65535U : (uint16_t)s;
	}
}

void Angle_RequestRecalibrate(void)
{
	s_req_recal = 1U;
}

void Angle_SetBiasLearning(uint8_t enable)
{
	s_learn_en = (enable != 0U) ? 1U : 0U;
}

void Angle_SetScalePpm(int32_t ppm)
{
	s_scale_k = 1.0f + (float)ppm * 1.0e-6f;
}
