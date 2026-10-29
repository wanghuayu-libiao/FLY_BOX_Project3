/**
  ******************************************************************************
  * @file    car_bringup.c
  * @brief   STM32 脱机主控：双电机绑定初始化 + 差速运动学（直发版，见 car_bringup.h）
  *
  *  ★ 单位换算只有一个入口：Bu_MmTo0p1mm() / Bu_MmpsTo0p1mmps()。
  *      协议里位移单位是 0.1mm、速度是 0.1mm/s，所以「毫米 × 10」这一步
  *      在全文件里只出现一次，改也只改一处，不会再出现漏乘 / 多乘。
  *
  *  ★ 轮径字节序：
  *      7500 = 0x1D4C，协议要求小端，所以 Data[2]=0x4C（低字节）、Data[3]=0x1D。
  *      若写成 1D 4C 会被电机解析成 0x4C1D = 19485 = 194.85mm，相差 2.6 倍。
  *      本文件用 (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) 生成，天然正确。
  *
  *  ★ 右轮反转（负数补码）：
  *      右轮直接下发 -delta，编译器按 int32 二进制补码存进 uint32_t，
  *      所以 Data[0~3] 就是正确的负数补码，不需要手工取反加一。
  *      例：-3047 = 0xFFFFF419 -> 字节 19 F4 FF FF
  *
  *  ★ 为什么不用 osDelay 之外的方式延时：
  *      本文件所有延时都是 osDelay，它会挂起任务、把 CPU 让给其它任务；
  *      HAL_Delay 是忙等，会阻塞调度，任务里禁止使用。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "can.h"
#include "protocol.h"
#include "motor.h"
#include "angle.h"
#include "car_cfg.h"
#include "car_bringup.h"

/* Private define ------------------------------------------------------------*/
#define CAR_BU_PI           3.14159265358979f

/* 原地转向一圈的弧长系数：弧长 = 轮距 × π × 角度 / 360 */
#define CAR_BU_ARC_K        ((float)CAR_WHEEL_BASE_MM * CAR_BU_PI / 360.0f)

/* 一次整圈（诊断用）的转速：慢一点方便数圈数，由 car_bringup.h 的 CAR_BU_DIAG_MMPS 控制 */

/* Private variables ---------------------------------------------------------*/
volatile uint8_t  g_car_bu_inited     = 0U;
volatile uint32_t g_car_bu_tx_count   = 0U;
volatile uint32_t g_car_bu_tx_fail    = 0U;
volatile int32_t  g_car_bu_last_delta = 0;
volatile uint32_t g_car_bu_last_speed = 0U;
volatile int32_t  g_car_bu_last_arc   = 0;
volatile uint32_t g_car_bu_scale_pct  = (uint32_t)CAR_BU_DIST_SCALE_PCT;
volatile uint8_t  g_car_bu_seq_stage  = 0U;
volatile int32_t  g_car_bu_seq_ret[2] = { -1, -1 };
volatile uint32_t g_car_bu_seq_waited_ms = 0U;   /* 上一步实际等了多久（Watch 用）*/
volatile uint32_t g_car_bu_boot_sn_count = 0U;   /* 上电时收到的 0x0312 广播条数 */
volatile uint8_t  g_car_bu_sn_conflict = 0U;     /* 1 = 两台算出的 SN 相同（绑定必然失败）*/
volatile uint8_t  g_car_bu_sn_source   = 0U;     /* 0=car_cfg.h 兜底 1=上电广播 2=混合 */

/* ---- 往返测试 / 方形测试的进度（Watch 观察用）---- */
volatile uint32_t g_car_rt_round = 0U;          /* 往返：第几轮（0 起）*/
volatile uint32_t g_car_rt_leg   = 0U;          /* 往返：第几段（0 起）*/
volatile uint32_t g_car_sq_round = 0U;          /* 方形：第几轮（0 起）*/
volatile uint32_t g_car_sq_loop  = 0U;          /* 方形：第几个正方形（0 起）*/
volatile uint32_t g_car_sq_side  = 0U;          /* 方形：第几条边（0..3）*/
volatile uint8_t  g_car_tst_done = 0U;          /* 全部测试跑完 = 1 */

/* ---- 转角闭环（陀螺仪）---- */
volatile float    g_car_yaw_target = 0.0f;      /* 目标绝对航向 ° */
volatile float    g_car_yaw_actual = 0.0f;      /* 最近一次读到的航向 ° */
volatile float    g_car_yaw_err    = 0.0f;      /* 还差多少度 */
volatile uint8_t  g_car_yaw_iters  = 0U;        /* 这次转角补了几次 */

/* ---- 原地 360° 转圈标定 ---- */
/* ★ 上电初值取自编译期常量 CAR_BU_TURN_GAIN_PCT（99.61 = 补偿掉转多 0.389%）。
 *   标定后会被覆盖，但断电失效；想永久生效改那个宏。 */
volatile float    g_car_turn_gain     = ((float)CAR_BU_TURN_GAIN_PCT / 100.0f);
volatile float    g_car_turn_arc_off  = CAR_BU_TURN_ARC_OFFSET_MM;  /* 固定弧长补偿 mm */

/* ---- ★ 转角停止点系统偏差（在线学习）----
 *
 *  ★ 为什么必须有它：
 *    实测 5 轮累计偏 100mm -> 每轮 20mm -> 每段残余 eps = 0.23°。
 *    而容差已经不能再收了：0.3° 只有 1.0mm 弧长，a=500mm/s² 下峰值速度
 *    仅 23mm/s，顶不动静摩擦，闭环会卡在"补了不走、反复重试"。
 *    容差 0.8° 时峰值 37mm/s 才勉强能动。
 *    也就是说【残余 0.2~0.8° 是物理下限，靠收容差到不了 1cm】。
 *
 *  ★ 但残余是【固定方向】的（实测就是一直往右），可以学出来补掉：
 *    闭环每次补转的最后一下弧长只有零点几毫米，静摩擦/齿隙吃掉一块，
 *    于是总是差一点点，方向一致 -> 每轮同向偏移 -> 5 轮累积成 10cm。
 *
 *  ★ 学习律：纯积分  bias += K·err
 *    ★★ 不能写 bias += K·(err - bias)：那是 K·(真值 - 2·bias)，
 *       会收敛到【真值的一半】。这个坑踩过一次。
 *    补偿：arc += sign(angle) × bias × CAR_BU_ARC_K
 *
 *  ★ 收敛后：固定残余被补掉，只剩随机重复性。
 *    随机部分不累积（正负抵消），落点 RMS ≈ √5 × L × σ_repeat。
 *
 *  ★ 限幅 ±STOP_MAX：顶到限幅也只是每次多转这么多，
 *    闭环下一轮看得见、能纠回来，不会把车转歪。
 *  ★ 关掉：CAR_TST_YAW_STOP_LEARN = 0
 *  ★ 进度看 CAN 帧 0x7F3（bias / 样本数 / 等效弧长）
 */
volatile float    g_car_turn_stop_bias = CAR_TST_YAW_STOP_INIT; /* 停止点偏差 °（初值见 car_bringup.h）*/
volatile uint32_t g_car_turn_stop_n    = 0U;   /* 学了几次 */
volatile uint32_t g_car_turn_corr_cnt  = 0U;   /* 累计补转了几次 */

/* ---- ★ 航向闭环直行（治「直行越走越偏」）----
 * 每走一小段就用陀螺仪把航向纠回来，通过改【目标位移】而不是改速度
 * （位置模式下改速度无效：两轮最终位移相同，航向净变化为 0）。 */
volatile float    g_car_hdg_yaw_err  = 0.0f;   /* 最近一次分段的航向误差 ° */
volatile float    g_car_hdg_corr_mm  = 0.0f;   /* 最近一次发出去的修正量 mm */
volatile uint8_t  g_car_hdg_seg      = 0U;     /* 当前第几段 */
volatile uint32_t g_car_bu_nofb_cnt  = 0U;     /* 0x0420 反馈缺失（假到位）次数 */
volatile uint8_t  g_car_bu_abort     = 0U;     /* 1 = 因无法确认停车而终止整轮测试 */
volatile uint8_t  g_car_hdg_ref_mode = 0U;     /* 0=当前航向作基准 1=绝对目标航向作基准 */

/* ---- ★ 连续直行航向补偿（两轮目标位移差 + 在线学习）----
 *   trim_mm 是核心：走 5m 时左轮走 D+trim、右轮走 D-trim，
 *   使航线变成微弧，正好抵消打滑造成的漂移。
 *   关系：α = 0.2954·Δ（度），y = 12.89·Δ（mm，D=5000）
 *   实测右偏 20mm -> 需要 Δ = 1.55mm */
volatile float    g_car_hdg_trim_mm  = CAR_HC_TRIM_INIT_MM;   /* 学到/生效的位置差补偿 mm */
volatile float    g_car_hdg_alpha    = 0.0f;   /* 上一段实际转过的角度 °（目标 0）*/
volatile uint8_t  g_car_hdg_trim_n   = 0U;     /* 学了几次 */
volatile uint8_t  g_car_bu_turn_i    = 0U;     /* 开环转角验证：当前第几圈 */
volatile uint8_t  g_car_bu_sn_match  = 0U;     /* bit0=配置的左轮SN在总线上找到；bit1=右轮 */
volatile uint8_t  g_car_bu_turnerr   = 0U;     /* 掉头失败次数（0x7F1 [2] 可看）*/
volatile uint8_t  g_car_dither_cnt   = 0U;     /* 抖动法用了几次 */

/* ★ 抖动法专用：临时跳过 Car_RotateInPlaceEx 内部的 bias 补偿。
 *   原因见 Car_RotateInPlaceEx 里 s_bu_skip_bias 的注释：
 *   抖动两步会正反各调一次，若都叠 bias 就互相抵消了。 */
static uint8_t    s_bu_skip_bias     = 0U;

/* 本次指令的物理最短运行时间 ms = 位移 ÷ 速度。
 * ★ 用于 Bu_WaitMotionDone 的兜底下限：收不到 0x0420 时不能提前认为"走完了"。 */
static uint32_t   s_expect_ms        = 0U;

/* 本次指令期间【曾经】见到过"忙"。
 * ★ 必须是"每条指令"级别的状态，不能是"每次等待"级别：
 *   一条指令可能被等两次（先由 Car_MoveStraightHeading 内部逐段等，
 *   外层再确认一次），若按"每次等待"判，第二次必然 seen==0，
 *   就会把每条正常的指令都误报成"反馈缺失"。 */
static uint8_t    s_motion_seen_busy = 0U;

/* ---- 转角误差分辨测试（判「固定偏移」还是「增益误差」）---- */
volatile uint8_t  g_car_angchk_phase  = 0U;     /* 0=待机 1=测量中 3=完成 */
volatile uint8_t  g_car_angchk_set    = 0U;     /* 当前测第几个角度点 */
volatile uint8_t  g_car_angchk_run    = 0U;     /* 当前第几次重复 */
volatile uint8_t  g_car_angchk_class  = 0U;     /* 0=未判 1=固定偏移 2=增益误差 3=混合 */
volatile float    g_car_angchk_deg[CAR_ANGCHK_SETS];     /* 各角度点的测试角度 ° */
volatile float    g_car_angchk_err[CAR_ANGCHK_SETS];     /* 各角度点的平均误差 ° */
volatile float    g_car_angchk_run_err[CAR_ANGCHK_SETS][CAR_ANGCHK_REPS];
volatile float    g_car_angchk_ratio  = 0.0f;   /* e(最小角)/e(最大角)，0.25=增益 1.0=固定 */
volatile float    g_car_angchk_arc_sug = 0.0f;  /* 建议的 CAR_BU_TURN_ARC_OFFSET_MM */
volatile float    g_car_angchk_gain_sug = 1.0f; /* 建议的 CAR_BU_TURN_GAIN_PCT（百分比）*/
volatile float    g_car_angchk_verify_err = 0.0f; /* ★ 把建议补偿真的加上后，360° 的残余误差 */
volatile uint8_t  g_car_angchk_verified = 0U;   /* 1 = 补偿已验证有效 */
volatile uint8_t  g_car_spin_phase    = 0U;     /* 0=待机 1=正转 2=反转 3=完成 */
volatile uint32_t g_car_spin_loop     = 0U;     /* 当前第几圈 */
volatile float    g_car_spin_fwd_total = 0.0f;  /* 正转 N 圈累积残余 ° */
volatile float    g_car_spin_rev_total = 0.0f;  /* 反转 N 圈累积残余 ° */
volatile float    g_car_spin_fwd_per   = 0.0f;  /* 正转每圈偏差 ° */
volatile float    g_car_spin_rev_per   = 0.0f;  /* 反转每圈偏差 ° */
volatile float    g_car_spin_gain_err  = 0.0f;  /* 转角增益误差 */
volatile float    g_car_spin_bias_drift = 0.0f; /* 陀螺仪零漂 °/分钟 */
volatile float    g_car_spin_wb_suggest = 0.0f; /* 建议的 CAR_WHEEL_BASE_MM */
volatile float    g_car_spin_gain_prev  = 1.0f; /* 标定前生效的增益 */

/* ★ 每圈累积残余 °：既是"记录陀螺仪数据"，也是线性度检查的依据。
 *   数组必须放 .bss（不能用局部变量）—— Watch 要能看。 */
volatile float    g_car_spin_fwd_loop[CAR_SPIN_MAX_LOOPS];
volatile float    g_car_spin_rev_loop[CAR_SPIN_MAX_LOOPS];
/* ★ 补偿后复测：把 g_car_turn_gain 真的加上再转一遍的结果。
 *   SPIN 前两个阶段是在【增益=1.0】下测原始误差（所以永远显示 1.4°），
 *   这一组才是"补偿到底有没有用"的答案。 */
volatile float    g_car_spin_verify_per   = 0.0f;  /* 复测后每圈偏差 °（应该 ≈0）*/
volatile uint8_t  g_car_spin_verified     = 0U;    /* 1 = 补偿有效 */
volatile float    g_car_spin_verify_loop[CAR_SPIN_MAX_LOOPS];

/* ---- 直行测试 ---- */
volatile uint8_t  g_car_str_phase     = 0U;     /* 0=待机 1=前进 2=后退 3=完成 */
volatile uint32_t g_car_str_run       = 0U;     /* 当前第几组 */
volatile float    g_car_str_dist_cmd  = 0.0f;   /* 命令距离 mm */
volatile float    g_car_str_scale_cmd = 0.0f;   /* 命令时用的距离系数 */
volatile float    g_car_str_yaw_fwd   = 0.0f;   /* 前进末端偏角 ° */
volatile float    g_car_str_yaw_bwd   = 0.0f;   /* 后退末端偏角 ° */
volatile float    g_car_str_yaw_per_m = 0.0f;   /* 每米跑偏 °/m */
volatile float    g_car_str_bias      = 0.0f;   /* 等效轮差 */
volatile float    g_car_str_lat_fwd_mm = 0.0f;  /* 前进末端横偏 mm */
volatile float    g_car_str_lat_bwd_mm = 0.0f;  /* 后退末端横偏 mm */
volatile float    g_car_str_yaw_drift = 0.0f;   /* 零偏贡献 °/m */
volatile float    g_car_str_fwd_run[CAR_STR_MAX_RUNS];
volatile float    g_car_str_bwd_run[CAR_STR_MAX_RUNS];

/* ---- 自动方向识别（推车法）----
 * ★ 初值由 car_cfg.h 的 CAR_LEFT_DIR_SIGN / CAR_RIGHT_DIR_SIGN 决定，
 *   不用再直接改这里。{+1,-1} 与 {-1,+1} 互为整体取反，
 *   同时反转直行和转向，"车倒着走"就互换这两个。 */
volatile int8_t   g_car_dir_sign[2]  = { (int8_t)CAR_LEFT_DIR_SIGN, (int8_t)CAR_RIGHT_DIR_SIGN };
volatile int8_t   g_car_dir_left_dev = 0;           /* 认定为左轮的设备号（0 或 1）*/
volatile int32_t  g_car_dirdet_push[2] = { 0, 0 };  /* ①推车阶段：剩余位移积分 */
volatile int32_t  g_car_dirdet_rot[2]  = { 0, 0 };  /* ②左转阶段：剩余位移积分 */
volatile int32_t  g_car_dirdet_torque[2] = { 0, 0 };/* 推车阶段力矩积分（交叉验证）*/
volatile uint8_t  g_car_dirdet_state = 0U;          /* 0=未开始 1=等推车 2=采推车
                                                     * 3=等左转 4=采左转 5=完成 9=失败 */
volatile uint8_t  g_car_dirdet_ok    = 0U;          /* 1 = 识别成功 */
volatile uint8_t  g_car_dirdet_fail  = 0U;          /* 失败原因码，见 Car_AutoDetectDirections */

/* ★ 本次 STM32 上电是否已写过 0x0316（电机参数）。
 *   0x0316 写第二次会把电机写死（实测：位移 61 -> 8 -> 0，ACK 也没了），
 *   所以这里必须严格只写一次，除非确认电机真的重新上电了。 */
static uint8_t s_param_written = 0U;
/* 1 = 电机刚重启过，必须重写一次 0x0316 */
static uint8_t s_force_param   = 0U;

/* Private function prototypes -----------------------------------------------*/
static int32_t  Bu_Round(float v);
static float    Bu_AbsF(float v);
static float    Bu_Wrap180(float deg);
static int32_t  Bu_AbsI32(int32_t v);
static float    Bu_Scale(void);
static int32_t  Bu_MmTo0p1mm(float mm);
static uint32_t Bu_MmpsTo0p1mmps(float mm_s);
static uint32_t Bu_AccScaled(void);
static uint32_t Bu_AccTo0p1mmps2(uint32_t acc_0p1mmps2);
static uint32_t Bu_MoveTimeout(float distance_mm, float speed_mm_s);
static uint32_t Bu_TurnTimeout(float angle_deg, float speed_deg_s);

static uint8_t  Bu_Send(uint16_t id, const uint8_t *data, uint8_t dlc);
static uint8_t  Bu_SendBind(uint8_t dev, const uint8_t *sn7);
static uint8_t  Bu_SendParam(uint8_t dev, uint8_t motor_no, uint8_t dir_reversed);
static uint8_t  Bu_SendWheel(uint8_t dev, uint16_t wheel_0p01mm);
static uint8_t  Bu_SendLock(uint8_t dev, uint8_t code);
/* 下面三个的第二个含义是「马达号」，不是通道！
 *    马达号 0 = 左轮，马达号 1 = 右轮，写进每条报文的 Data[4]。
 *    帧 ID 一律用主帧组：
 *        0x0414 加速度 / 0x0415 速度 / 0x0416 位置（DLC=6，带目标类型 0x00）
 * ★ 不要改用从帧组 0x0417/0x0418/0x0419 —— 主从是同一台电机的两个控制入口，
 *   同时下发会互相抵消（实测 主 +2000 / 从 -2000 -> 转角 0.1°，车纹丝不动）。 */
static uint8_t  Bu_SendSpeed(uint8_t motor_no, uint32_t speed_0p1mmps);
static uint8_t  Bu_SendAccel(uint8_t motor_no, uint32_t acc_0p1mmps2);
static uint8_t  Bu_SendPosRel(uint8_t motor_no, int32_t dist_0p1mm);

static uint8_t  Bu_StartMotion(int32_t left_0p1mm, int32_t right_0p1mm, uint32_t speed_0p1mmps,
                               uint32_t accel_0p1mmps2);
static uint8_t  Bu_EmitMotion(int32_t left_0p1mm, int32_t right_0p1mm, uint32_t speed_0p1mmps,
                              uint32_t accel_0p1mmps2);
static void     Bu_WaitMotorBoot(void);
static void     Bu_ReinitIfMotorRebooted(void);
static uint8_t  Bu_WaitMotionDone(uint32_t timeout_ms);
static void     Bu_TurnFinish(float err);
static float    Bu_SpinOneDirection(int8_t sign, volatile float *loop_rec);
static float    Bu_StraightOneLeg(float distance_mm);
static float    Bu_TurnRawOnce(float angle_deg);

/**
  * @brief  运行时决定「设备 dev 要绑哪个 SN」
  *
  *         优先级：
  *           ① 上电广播 0x0312 收到的第 dev 条（dev = 0 / 1）
  *           ② 广播不够时退回 car_cfg.h 里写的 SN（兜底，老车仍可用）
  *
  *         ★ 两台必须拿到不同的 SN。motor.c 的 0x0312 分支已经按
  *           「首次出现的不同 SN」去重分槽，所以 ① 天然不会撞号；
  *           ② 兜底值只要 car_cfg.h 里两条不同就没问题。
  *           真正的撞号风险在「一台用广播、一台用兜底」时：
  *           如果那台电机的真实 SN 恰好就是 car_cfg.h 里写的兜底值，
  *           它会在两个槽里各被记一次 —— 那种情况由 Car_Motor_Init
  *           里的 g_car_bu_sn_conflict 标记出来。
  *
  * @param  dev  设备号（0 / 1）
  * @param  out  输出 7 字节 SN
  * @retval None
  */
static void Bu_ResolveSn(uint8_t dev, uint8_t *out)
{
	static const uint8_t cfg[2][7] = { { CAR_LEFT_SN_BYTES }, { CAR_RIGHT_SN_BYTES } };
	const uint8_t *want = cfg[(dev != 0U) ? 1U : 0U];
	uint8_t used = Motor_GetBootSnUsed();
	uint8_t i, k;

	/* ★★★ 按 SN 【内容】匹配，不能按广播顺序 ★★★
	 *
	 *   原来的写法是 "src = Motor_GetBootSn(dev)"，即把【第 dev 条收到的广播】
	 *   当成设备 dev 的 SN。但 0x0312 广播的到达顺序取决于谁先上电/谁先发，
	 *   与物理左右轮毫无关系。
	 *
	 *   一旦右轮先广播：Bu_ResolveSn(0) 返回右轮 SN -> 右轮被绑成设备 0
	 *   -> 后面所有 "设备0 = 左轮" 的假设全部错位，
	 *   表现就是「前进变后退」「转向反向」这一类方向乱掉的问题。
	 *
	 *   现在的做法：拿 car_cfg.h 里【明确写死】的 SN 去广播列表里找，
	 *   找到就说明这台电机确实在总线上；找不到也仍然用配置值
	 *   （配置值是用户从电机铭牌上抄下来的，是唯一权威来源）。
	 *
	 *   并把"是否匹配上"记进 g_car_bu_sn_match，可用 0x7F1 的 bit3 查看。 */
	for (k = 0U; k < 7U; k++) { out[k] = want[k]; }   /* 先填配置值（权威）*/

	for (i = 0U; i < used; i++)
	{
		const uint8_t *bs = Motor_GetBootSn(i);
		uint8_t same = 1U;

		for (k = 0U; k < 7U; k++)
		{
			if (bs[k] != want[k]) { same = 0U; break; }
		}
		if (same != 0U)
		{
			/* 配置的 SN 在总线上确实存在 */
			g_car_bu_sn_match = (uint8_t)(g_car_bu_sn_match | (uint8_t)(1U << dev));
			return;
		}
	}
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  双电机绑定初始化（阻塞，约 0.5 秒）
  *
  *         严格按顺序（每发一条都 osDelay(CAR_BU_STEP_DELAY_MS) = 50ms）：
  *           1) 绑定 SN 设置设备号  0x0313，DLC=8：[SN0..SN6, 设备号]
  *                左 [SN_左 0..6, 00]
  *                右 [SN_右 0..6, 01]
  *              ★ SN 现在【不写死】：优先用电机上电广播的 0x0312（见 Bu_ResolveSn），
  *                只有广播不到时才退回 car_cfg.h。换车不用改代码。
  *           2) 配置电机参数        0x0316，DLC=8：B4 BF CF AA 00 5F [方向|马达号] [设备号]
  *                左 [B4 BF CF AA 00 5F 00 00]
  *                右 [B4 BF CF AA 00 5F 80 01]   <- 80 = Bit7 方向位，反转
  *           3) 配置轮径（连发 3 次，间隔 10ms）0x0408，DLC=4：[设备号, 0x24, 4C, 1D]
  *                左 [00 24 4C 1D]
  *                右 [01 24 4C 1D]
  *           4) 使能                0x0413，DLC=5：[0x5A, 0, 0, 0, 设备号]
  *                左 [5A 00 00 00 00]
  *                右 [5A 00 00 00 01]
  *         末尾补一条加速度帧（0x0414/0x0417），由 CAR_BU_SEND_ACCEL 控制。
  *         实测：位置模式下电机没收到加速度会只回 ACK、剩余位移不减少（不动）。
  *
  * @param  None
  * @retval None
  */
void Car_Motor_Init(void)
{
	/* ★★★ SN 不再写死：优先用电机上电自己广播的 0x0312 ★★★
	 *
	 *   原来这里用 car_cfg.h 里写死的 CAR_LEFT_SN_BYTES / CAR_RIGHT_SN_BYTES，
	 *   换一台车就得改代码 —— 真实场景里不可能每台都写死。
	 *
	 *   现在的取法（Bu_ResolveSn）：
	 *     ① 上电广播收集到两条不同 SN -> 设备 0 用第 1 条、设备 1 用第 2 条
	 *     ② 只收到一条 -> 设备 0 用它，设备 1 退回 car_cfg.h（兜底）
	 *     ③ 一条都没收到 -> 两台都用 car_cfg.h（兜底）
	 *
	 *   ★ 顺序不确定（谁先广播）不影响功能：绑错的只是"哪台算左"，
	 *     而左右由 Car_AutoDetectDirections() 推车测出来，不依赖这里的顺序。
	 *   ★ 但两台必须拿到【不同的】SN，否则会被绑成同一个设备号 ——
	 *     那就是"车只会原地打转"的经典故障。下面有显式校验。 */
	uint8_t sn_left[7];
	uint8_t sn_right[7];
	uint8_t need_param;
	uint8_t i;
	uint8_t same = 1U;

	Bu_ResolveSn(0U, sn_left);
	Bu_ResolveSn(1U, sn_right);

	/* ---- 记录 SN 来源，方便上位机显示 ---- */
	if (Motor_GetBootSnUsed() >= 2U)      { g_car_bu_sn_source = 1U; }
	else if (Motor_GetBootSnUsed() == 1U) { g_car_bu_sn_source = 2U; }
	else                                  { g_car_bu_sn_source = 0U; }

	/* ---- 校验：两个 SN 不能相同，否则必然绑成同一个设备号 ---- */
	for (i = 0U; i < 7U; i++)
	{
		if (sn_left[i] != sn_right[i]) { same = 0U; break; }
	}
	g_car_bu_sn_conflict = same;

	g_car_bu_inited    = 0U;
	g_car_bu_tx_count  = 0U;
	g_car_bu_tx_fail   = 0U;
	g_car_bu_scale_pct = (uint32_t)CAR_BU_DIST_SCALE_PCT;

	/* ★★★ 0x0316 只能写一次 ★★★
	 * 实测（同一台电机，间隔 1.2 秒各采一次）：
	 *     写 1 次 0x0316 -> 位移=61 心跳=6，之后 4.8 秒稳定在 61/6，没问题
	 *     写第 2 次      -> 位移 8 -> 0，心跳 0，ACK 也没有了，电机彻底不响应
	 * 所以判据是「电机是否刚上电」：
	 *     收到过 0x0312 广播（g_car_bu_boot_sn_count > 0）-> 电机刚复位，
	 *         参数已丢，必须写一次 0x0316
	 *     一次广播都没收到 -> 电机一直在跑，参数还在，
	 *         ★ 绝对不能写 0x0316，只补绑定就行
	 *
	 * 这条以前是「只动左轮 / 右轮不动 / 电机锁住不动」反复出现的总根源：
	 * 每次烧录或重启 STM32 都会再写一次 0x0316，把电机的参数区写坏。 */
	need_param = ((g_car_bu_boot_sn_count > 0U) || (s_force_param != 0U)) ? 1U : 0U;

	if (s_param_written != 0U)
	{
		need_param = 0U;                  /* 本次上电已经写过，绝不再写 */
	}

	/* ---------------- 1. 绑定 SN（0x0313）：两台都要绑，并且绑到不同的设备号 ----------------
	 * 0x0313 报文 = [SN0..SN6, 设备号]
	 * ★★ 设备号必须不同（0 和 1）：它决定后面 0x0316 写给谁。
	 *    两台都绑 0 的话，0x0316 会被两台同时收到，马达号永远分不开。
	 * ★ 绑定可以反复重发（实测重绑 3 次反馈一直稳定）。 */
	(void)Bu_SendBind(CAR_BU_LEFT_MOTOR_NO, sn_left);
	osDelay(CAR_BU_STEP_DELAY_MS);

	(void)Bu_SendBind(CAR_BU_RIGHT_MOTOR_NO, sn_right);
	osDelay(CAR_BU_STEP_DELAY_MS);

	/* ---------------- 2. 配置电机参数（0x0316）—— ★ 每台一条，写各自的马达号 ----------------
	 * 0x0316 报文 = [B4 BF CF AA 00 5F, Byte6, Byte7]
	 *     Byte6 bit3..0 = 马达号（0 = 左轮，1 = 右轮）  ★ 差速的根
	 *     Byte6 bit5/bit4 = 0：自动回传剩余位移 / 力矩（置 1 就没有 0x0420/0x0440 了）
	 *     Byte7 = 设备号，决定这一帧被哪台电机接收
	 *
	 * ★★★ 同一台电机绝不能连续收到两条 0x0316 ★★★
	 *   实测：1.2 秒内写第 2 次 -> 位移 8 -> 0、心跳 0、ACK 消失，电机彻底不响应，
	 *        只能断电重启。所以用 need_param 保证本次上电每台只写一条。
	 *   现在两台设备号不同，各自只会收到自己那一条，天然不会写重。 */
	if (need_param != 0U)
	{
		(void)Bu_SendParam(CAR_BU_LEFT_MOTOR_NO,  (uint8_t)CAR_BU_LEFT_MOTOR_NO,
		                   (uint8_t)CAR_LEFT_DIR_REVERSED);
		osDelay(CAR_BU_STEP_DELAY_MS);

		(void)Bu_SendParam(CAR_BU_RIGHT_MOTOR_NO, (uint8_t)CAR_BU_RIGHT_MOTOR_NO,
		                   (uint8_t)CAR_RIGHT_DIR_REVERSED);
		s_param_written = 1U;
		osDelay(CAR_BU_STEP_DELAY_MS);
	}

	/* ---------------- 3. 配置轮径（0x0408），每台连发 3 次 ----------------
	 * 报文 = [设备号, 0x24, 轮径低字节, 轮径高字节]；左右轮分开标定
	 * ★ 实测这条是安全的：发完后反馈帧数不变 */
	for (i = 0U; i < CAR_BU_WHEEL_REPEAT; i++)
	{
		(void)Bu_SendWheel(CAR_BU_LEFT_MOTOR_NO, (uint16_t)CAR_LEFT_WHEEL_0P01MM);
		osDelay(CAR_BU_WHEEL_GAP_MS);
	}
	for (i = 0U; i < CAR_BU_WHEEL_REPEAT; i++)
	{
		(void)Bu_SendWheel(CAR_BU_RIGHT_MOTOR_NO, (uint16_t)CAR_RIGHT_WHEEL_0P01MM);
		osDelay(CAR_BU_WHEEL_GAP_MS);
	}
	osDelay(CAR_BU_STEP_DELAY_MS);

	/* ---------------- 4. 上电使能（0x0413，Data[0]=0x5A）：两台都要发 ----------------
	 * 报文 = [0x5A, 0, 0, 0, 马达号]
	 * ★★★ 这条必须发！
	 *   实测：不发 0x0413 -> 剩余位移照常递减，但速度只有 ~28 mm/s、力矩≈0，
	 *         即电机只更新目标、不出力。发了 -> 78~108 mm/s，位移干净走到 0。
	 *   （之前误判成"0x0413 会把电机搞死"，真凶是紧跟着又写了一遍 0x0316。） */
	(void)Bu_SendLock(CAR_BU_LEFT_MOTOR_NO, MOTOR_LOCK_ENABLE);
	osDelay(CAR_BU_STEP_DELAY_MS);
	(void)Bu_SendLock(CAR_BU_RIGHT_MOTOR_NO, MOTOR_LOCK_ENABLE);
	osDelay(CAR_BU_STEP_DELAY_MS);

	/* ---------------- 5. 两台各自设加速度和速度 ----------------
	 * ★ 只配一台时，另一台速度恒为 0：位置指令会被接受（剩余位移设上了），
	 *   但轮子一步不走（剩余位移卡在目标值）。这正是"只有一个轮子会转"的成因之一。 */
#if (CAR_BU_SEND_ACCEL != 0)
	(void)Bu_SendAccel(CAR_BU_LEFT_MOTOR_NO, Bu_AccScaled());
	osDelay(CAR_BU_STEP_DELAY_MS);

	(void)Bu_SendAccel(CAR_BU_RIGHT_MOTOR_NO, Bu_AccScaled());
	osDelay(CAR_BU_STEP_DELAY_MS);
#endif

	(void)Bu_SendSpeed(CAR_BU_LEFT_MOTOR_NO, Bu_MmpsTo0p1mmps(CAR_BU_DEFAULT_SPEED_MMPS));
	osDelay(CAR_BU_STEP_DELAY_MS);

	(void)Bu_SendSpeed(CAR_BU_RIGHT_MOTOR_NO, Bu_MmpsTo0p1mmps(CAR_BU_DEFAULT_SPEED_MMPS));
	osDelay(CAR_BU_STEP_DELAY_MS);

	g_car_bu_inited = 1U;
}

/**
  * @brief  直线运动：左右轮走相同的正/负位移
  * @param  distance_mm  位移 mm，负值 = 后退
  * @param  speed_mm_s   速度 mm/s
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t Car_MoveStraight(float distance_mm, float speed_mm_s)
{
	return Car_MoveStraightEx(distance_mm, speed_mm_s, 0U);
}

/**
  * @brief  ★ 直线运动接口（带加速度）—— 课题三·要求 1
  *
  *         输入参数：距离、速度、加速度。
  *         加速度传 0 表示用默认值 CAR_BU_ACC_0P1MMPS2。
  *
  *         ★ 加速度也要乘标定系数（Bu_AccTo0p1mmps2），
  *           理由和距离/速度一致：所有量纲同比例放大，加速时间才不变。
  * @param  distance_mm      位移 mm，负值 = 后退
  * @param  speed_mm_s       速度 mm/s（取绝对值）
  * @param  accel_0p1mmps2   加速度 0.1mm/s²，0 = 用默认值
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t Car_MoveStraightEx(float distance_mm, float speed_mm_s, uint32_t accel_0p1mmps2)
{
	int32_t  delta = Bu_MmTo0p1mm(distance_mm);             /* mm -> 0.1mm（含标定）*/
	uint32_t speed = Bu_MmpsTo0p1mmps(speed_mm_s);          /* mm/s -> 0.1mm/s（含标定）*/

	/* 未标定的值单独存一份，Watch 核对公式用 */
	g_car_bu_last_arc = Bu_Round(distance_mm * 10.0f);

	/* 两轮同向同量 -> 直行 */
	return Bu_StartMotion(delta, delta, speed, accel_0p1mmps2);
}

/**
  * @brief  ★★★ 航向闭环直行：分段走，每段之间用陀螺仪把航向纠回来 ★★★
  *
  *   课题二·治「直行越走越偏」的唯一手段。
  *
  *   ★★★ 为什么不能"改速度" ★★★
  *     位置模式下发的是【目标位移】，速度只是"多快走完"的上限。
  *     中途把左轮设 550、右轮设 650 -> 右轮先走完 5000mm 停下，
  *     左轮再走完 5000mm -> 两轮最终位移依然相同 -> 航向净变化 = 0。
  *     中途看着晃了一下，落点一点没变。
  *     所以要纠航向，必须改【目标位移】：
  *         左轮 = d + corr,  右轮 = d - corr
  *
  *   ★ 差速量推导：
  *     要让车在走 d 的同时转过 theta（rad），两轮差 = theta × 轮距
  *     即 corr = theta × 轮距 / 2               （每轮加减一半）
  *     theta = 0.1° 时 corr = 0.34mm，占 1000mm 段的 0.03% —— 很精细，
  *     但正因如此才能做到亚角度级纠偏。
  *
  *   ★★★ 为什么必须分段 ★★★
  *     实测"运动中途改 0x0416 目标位置会把电机搞死（只能断电重启）"，
  *     所以只能在【上一段自然走完、剩余位移归零之后】再发下一段。
  *     这就是分段的由来：不是想分，是电机协议只允许在停稳时改目标。
  *
  *   ★ 段长怎么定（受减速距离限制）：
  *         300mm/s -> 减速距离  90mm -> 段长 >= 180mm -> 5m 最多 27 段
  *         400mm/s -> 减速距离 160mm -> 段长 >= 320mm -> 5m 最多 15 段
  *         600mm/s -> 减速距离 360mm -> 段长 >= 720mm -> 5m 最多  6 段
  *     段太短就跑不到设定速度（三角形速度曲线），所以要按速度选段数。
  *     CAR_TST_HEADING_SEGMENTS 就是段数，填 0 或 1 = 退回一次性走完（老行为）。
  *
  *   ★★★ 这个方案能治什么、治不了什么 ★★★
  *     能治：航向漂移导致的横向偏移【累积】。
  *           不闭环时偏移是按 t² 增长的（航向一直在偏）；
  *           每段纠一次后变成按 t 增长（每段只留段内那点偏移）。
  *     治不了：已经发生的横向偏移。车没有侧向传感器，
  *             不知道自己在中心线左边还是右边，只能保证"朝向对"。
  *             纠完航向车会平行于原方向走，偏出去的位移保留。
  *     所以这个改动是把"越偏越大"压成"偏一点就不动了"，
  *     不是把落点拉回 0。
  *
  * @param  distance_mm      总位移 mm，负值 = 后退
  * @param  speed_mm_s       速度 mm/s
  * @param  accel_0p1mmps2   加速度 0.1mm/s²，0 = 默认
  * @param  segments         分几段（0/1 = 不分段）
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t Car_MoveStraightHeading(float distance_mm, float speed_mm_s,
                                uint32_t accel_0p1mmps2, uint8_t segments)
{
	return Car_MoveStraightHdg(distance_mm, speed_mm_s, accel_0p1mmps2,
	                           segments, 0.0f, 0U);
}

/**
  * @brief  ★ 航向闭环直行（带绝对航向基准）
  *
  *         ★★★ 为什么要传基准航向 ★★★
  *         Car_RotateToHeading 的收敛判据是 |err| <= tol（默认 0.5°），
  *         也就是说掉头后航向可能是 目标+0.5°。
  *         若直行段把"当前航向"当基准（has_target=0），那 0.5° 就被
  *         当成正确方向写进参考，整段 5m 都带着它走：
  *             5000 × sin(0.5°) = 44mm
  *         这是当前最大的单项误差来源，比打滑大一个数量级。
  *         传 h_target 进来后，掉头残余会在第一段直行里被纠掉。
  *
  * @param  target_deg       绝对目标航向 °（has_target=1 时有效）
  * @param  has_target       1 = 用 target_deg 作基准；0 = 用当前航向作基准
  * @retval 0 成功，1 参数非法 / 发送失败 / 某段等不到位
  */
uint8_t Car_MoveStraightHdg(float distance_mm, float speed_mm_s,
                            uint32_t accel_0p1mmps2, uint8_t segments,
                            float target_deg, uint8_t has_target)
{
	AngleData_t ad;
	float    seg_mm;
	float    h0;
	float    yaw;
	float    yaw_start;
	float    alpha;
	float    err_deg;
	float    corr_mm;
	float    corr_max;
	int32_t  l0p1;
	int32_t  r0p1;
	uint32_t speed;
	uint8_t  i;

	if ((distance_mm == 0.0f) || (speed_mm_s == 0.0f))
	{
		return 1U;
	}

	/* ★ 等车停稳再读陀螺仪（运动中读数带振动噪声） */
	osDelay(CAR_TST_YAW_SETTLE_MS);
	Angle_GetData(&ad);

	yaw_start = ad.angle_deg;              /* ★ 起步实测航向（学习要用它作参照）*/

	if (has_target != 0U)
	{
		h0 = Bu_Wrap180(target_deg);       /* 绝对基准：顺带纠掉掉头残差 */
		g_car_hdg_ref_mode = 1U;
	}
	else
	{
		h0 = ad.angle_deg;                 /* 相对基准：当前航向 */
		g_car_hdg_ref_mode = 0U;
	}

	g_car_hdg_yaw_err = Bu_Wrap180(h0 - ad.angle_deg);   /* 起步时就有多少偏差 */

	if (segments < 2U)
	{
		/* ==================================================================
		 *   连续一次走完（当前方案）：两轮目标位移差 + 在线学习
		 * ==================================================================
		 *
		 *  ★ 为什么不用"实时调速"：
		 *      走 5m 耗时 8.3s，纠 0.5° 需左轮多走 3.4mm，
		 *      折成速度差只有 0.41mm/s —— 占 600mm/s 的 0.07%，
		 *      比两轮打滑差异还小，速度环分辨不出来。
		 *
		 *  ★ 为什么用"改目标位移"：
		 *      位置环分辨率 0.1mm；抵消 20mm 横向偏移只需 1.55mm 位移差
		 *      （0.031%），完全在能力范围内。
		 *
		 *  ★ 关系式（Δ 恒定走完全程）：
		 *          α = 2Δ·180/(π·W) = 0.2954·Δ   走完转过的角度（度）
		 *          y = D·α_rad/2   ≈ 12.89·Δ     横向偏移（mm，D=5000）
		 *      所以 y ∝ α：【把 α 学到 0，y 自然也是 0】，不需要侧向传感器。
		 *
		 *  ★ 学习的对象是「这段自己产生的转角」，不是"和绝对目标的差"：
		 *      用 yaw_start 而不是 h_target 作参照，否则掉头残差（±0.5°）
		 *      会被混进 trim，下一段残差方向一变 trim 就变错。
		 */

		corr_mm = g_car_hdg_trim_mm;
		if (corr_mm >  CAR_HC_MAX_MM) { corr_mm =  CAR_HC_MAX_MM; }
		if (corr_mm < -CAR_HC_MAX_MM) { corr_mm = -CAR_HC_MAX_MM; }

		l0p1 = Bu_MmTo0p1mm(distance_mm + corr_mm);
		r0p1 = Bu_MmTo0p1mm(distance_mm - corr_mm);

		g_car_hdg_corr_mm = corr_mm;
		g_car_hdg_seg     = 0U;
		g_car_bu_last_arc = Bu_Round(distance_mm * 10.0f);

		if (Bu_StartMotion(l0p1, r0p1, Bu_MmpsTo0p1mmps(speed_mm_s),
		                   accel_0p1mmps2) != 0U)
		{
			return 1U;
		}

		if (Bu_WaitMotionDone(Bu_MoveTimeout(distance_mm, speed_mm_s)) == 0U)
		{
			Angle_GetData(&ad);
			g_car_hdg_yaw_err = Bu_Wrap180(h0 - ad.angle_deg);
			return 1U;
		}

		/* ---- 走完：量这段自己转了多少，学掉它 ---- */
		osDelay(CAR_TST_YAW_SETTLE_MS);
		Angle_GetData(&ad);

		/* ★ 用【起步时的实测航向】作参照，不用 h0：
		 *   h0 可能是绝对目标（has_target=1），与实测起步航向差 ±0.5°，
		 *   拿它当参照会把掉头残差混进 alpha，学习就跑偏了。 */
		alpha = Bu_Wrap180(ad.angle_deg - yaw_start);
		g_car_hdg_alpha   = alpha;
		g_car_hdg_yaw_err = Bu_Wrap180(h0 - ad.angle_deg);

#if (CAR_HC_ENABLE != 0)
		/* 只在"像正常漂移"时才学 */
		if ((Bu_AbsF(alpha) <= CAR_HC_MAX_SAMPLE_DEG) &&
		    (Bu_AbsF(distance_mm) >= CAR_HC_MIN_DIST_MM))
		{
			/* 纯积分：α > 0（转多了）-> trim 减小
			 * ★ 不要写成 trim += K·(α - trim)，那会收敛到真值的一半 */
			g_car_hdg_trim_mm -= CAR_HC_LEARN_K * alpha;

			if (g_car_hdg_trim_mm >  CAR_HC_MAX_MM) { g_car_hdg_trim_mm =  CAR_HC_MAX_MM; }
			if (g_car_hdg_trim_mm < -CAR_HC_MAX_MM) { g_car_hdg_trim_mm = -CAR_HC_MAX_MM; }

			if (g_car_hdg_trim_n < 0xFFU) { g_car_hdg_trim_n++; }
		}
#endif

		return 0U;
	}

	seg_mm   = distance_mm / (float)segments;
	speed    = Bu_MmpsTo0p1mmps(speed_mm_s);

	/* 单段修正量上限：不能超过段长的一半，否则会变成"倒着走" */
	corr_max = Bu_AbsF(seg_mm) * 0.5f;

	for (i = 0U; i < segments; i++)
	{
		g_car_hdg_seg = i;               /* 供 Watch / 诊断帧观察 */

		/* ---- 读当前航向，算需要在本段内转回多少 ---- */
		Angle_GetData(&ad);
		yaw     = ad.angle_deg;
		err_deg = Bu_Wrap180(h0 - yaw);          /* 正 = 车头偏右，要往左转回来 */

		/* ★★★ corr 的符号（曾经写反，导致正反馈「越走越歪」）★★★
		 *
		 *  本工程的航向约定，由 Car_RotateInPlaceEx + Car_RotateToHeading 反推：
		 *      RotateInPlaceEx(+A) = Bu_StartMotion(左+A, 右-A)
		 *      -> 左轮比右轮走得多 -> angle_deg 增加 A
		 *  即：
		 *      Δangle_deg = (左轮位移 - 右轮位移) × 180 / (π × 轮距)
		 *
		 *  本段：左 = seg + corr，右 = seg - corr  ->  左 - 右 = 2 × corr
		 *  要求本段转过 err（err = h0 - yaw，正是"需要补回的角度"）：
		 *      2 × corr × 180 / (π × W) = err
		 *      corr = +err × (π/180) × (W/2)          ← 必须是正的
		 *
		 *  ★ 之前写成了 -(err...)，于是 err>0 时左轮反而少走，
		 *    航向朝错误方向再歪一份，每段误差翻倍。仿真：0.1° -> 3.1°（5 段），
		 *    横向偏移 270mm，正是"越偏越大"的表现。 */
		corr_mm = (err_deg * (CAR_BU_PI / 180.0f)) * ((float)CAR_WHEEL_BASE_MM * 0.5f);

		/* ★ 大误差保护：航向差超过这个量说明不是"漂移"，而是
		 *   陀螺仪刚复位（角度从 0 开始）、车被撞了、或者电机重启过。
		 *   这时候硬纠会把车甩到墙上，所以【放弃纠偏，只做诊断】。 */
		if (Bu_AbsF(err_deg) > CAR_TST_HEADING_MAX_ERR_DEG)
		{
			corr_mm = 0.0f;
		}

		/* ★ 限幅：一是不能超过段长一半，二是单段最多纠 CAR_TST_HEADING_MAX_CORR
		 *   （防止一次读到坏数据把车甩歪） */
		if (corr_mm >  corr_max) { corr_mm =  corr_max; }
		if (corr_mm < -corr_max) { corr_mm = -corr_max; }
		if (corr_mm >  CAR_TST_HEADING_MAX_CORR) { corr_mm =  CAR_TST_HEADING_MAX_CORR; }
		if (corr_mm < -CAR_TST_HEADING_MAX_CORR) { corr_mm = -CAR_TST_HEADING_MAX_CORR; }

		/* ★★★ 这里【不能】再乘前进方向 sign_d ★★★
		 *
		 *   Δangle_deg = (左轮位移 - 右轮位移) × 180/(π·W)
		 *   这个关系与"车在前进还是后退"无关，因为左右位移本身已经带符号：
		 *      倒退时两轮都是负值，左=-990、右=-1010 -> 左-右=+20 -> 同样转 +20k
		 *      （物理上右轮倒得多，车尾往右摆，车头往左转 = 正方向 ✓）
		 *
		 *   所以要转 +err，永远要求 左-右 = 2·corr > 0，
		 *   即 左 = seg+corr、右 = seg-corr，与 sign_d 无关。
		 *   乘上 sign_d 会让【倒车时的纠偏方向反过来】，变成正反馈。
		 *   （当前往返测试永远前进，所以这个错误是潜伏的，但必须修掉。） */
		l0p1 = Bu_MmTo0p1mm(seg_mm + corr_mm);
		r0p1 = Bu_MmTo0p1mm(seg_mm - corr_mm);

		/* ★ 记下来给诊断用：最后一段的发出去的两轮差 */
		g_car_hdg_corr_mm  = corr_mm;
		g_car_hdg_yaw_err  = err_deg;

		if (Bu_StartMotion(l0p1, r0p1, speed, accel_0p1mmps2) != 0U)
		{
			return 1U;
		}

		/* ★ 必须等这一段【真的走完、剩余位移归零】才能发下一段 ——
		 *   运动中途改 0x0416 目标会把电机搞死（实测只能断电重启）。
		 *   所以超时没到位就直接放弃剩下的段，绝不再发 0x0416。 */
		if (Bu_WaitMotionDone(Bu_MoveTimeout(seg_mm, speed_mm_s)) == 0U)
		{
			Angle_GetData(&ad);
			g_car_hdg_yaw_err = Bu_Wrap180(h0 - ad.angle_deg);
			return 1U;
		}
	}

	/* 收尾：把最终航向误差和累积修正量记下来 */
	Angle_GetData(&ad);
	g_car_hdg_yaw_err = Bu_Wrap180(h0 - ad.angle_deg);

	return 0U;
}

/**
  * @brief  原地转向（角度域速度入口）：左轮正转、右轮反转
  *
  *         ★★ 这是真正的差速转向，不需要任何额外硬件：
  *              左轮 0x0416 Data[4]=0 走 +弧长
  *              右轮 0x0416 Data[4]=1 走 -弧长（Bu_StartMotion 内部按 RIGHT_SIGN 取反）
  *            实测：两轮异号 -> 转角 0.00°（直行）；两轮同号 -> 转角 51.95°（原地转）。
  *
  * @param  angle_deg    车体转角 °，正 = 左转，负 = 右转
  * @param  speed_deg_s  车体角速度 °/s
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t Car_RotateInPlace(float angle_deg, float speed_deg_s)
{
	return Car_RotateInPlaceEx(angle_deg, speed_deg_s, 0U);
}

/**
  * @brief  ★ 原地转向接口（带加速度）—— 课题三·要求 2
  *
  *         输入参数：角度、速度、加速度。
  * @param  angle_deg        车体转角 °（正负方向约定同 Car_RotateInPlace）
  * @param  speed_deg_s      车体角速度 °/s（取绝对值）
  * @param  accel_0p1mmps2   加速度 0.1mm/s²，0 = 用默认值
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t Car_RotateInPlaceEx(float angle_deg, float speed_deg_s, uint32_t accel_0p1mmps2)
{
	float    arc_mm;
	float    v_mm_s;
	float    gain;
	int32_t  delta;

	if ((angle_deg == 0.0f) || (speed_deg_s == 0.0f))
	{
		return 1U;
	}

	/* ★ 转角增益补偿（由 Car_BuRunSpinCal 标定出来，0 = 不补偿）
	 *   实际转不够（g_car_turn_gain > 1）就多发一点弧长。
	 *   只乘弧长、不乘速度 —— 否则转角和角速度会一起变，标定就乱了。
	 *   限幅 0.5~2.0：超过这个范围说明不是增益问题，硬补会把车转飞。 */
	gain = g_car_turn_gain;
	if (gain < 0.5f) { gain = 0.5f; }
	if (gain > 2.0f) { gain = 2.0f; }

	/* 弧长(mm) = 轮距 × π × 角度 / 360；角速度换算同理 */
	arc_mm = CAR_BU_ARC_K * angle_deg * gain;
	v_mm_s = CAR_BU_ARC_K * ((speed_deg_s >= 0.0f) ? speed_deg_s : -speed_deg_s);

	/* ★ 固定弧长补偿（齿隙/静摩擦/死区）：与角度无关，只按方向加一次。
	 *   治的是「转不够」这类固定损失，和上面的比例增益是两回事，
	 *   详见 car_bringup.h 里 CAR_BU_TURN_ARC_OFFSET_MM 的说明。
	 *   ★ 只对「一次转向指令」有意义 —— 超过 360° 的大转角
	 *     损失仍然只有一份（齿隙只在开头吃一次）。 */
	if (g_car_turn_arc_off != 0.0f)
	{
		arc_mm += (angle_deg > 0.0f) ? g_car_turn_arc_off : -g_car_turn_arc_off;
	}

	/* ★★ 叠加【在线学到的停止点偏差】★
	 *   闭环每次补转的最后一下只有零点几毫米弧长，静摩擦/齿隙吃掉一块，
	 *   于是总是差一点点、方向一致 -> 每轮同向偏移 -> 5 轮累积 10cm。
	 *   这个偏差与角度无关，所以用一个固定弧长去补（不是乘系数）。
	 *
	 *   ★★★ s_bu_skip_bias：抖动法专用旁路 ★★★
	 *   抖动法把一次转角拆成「+over 然后 -over」，两次都调本函数。
	 *   若两次都叠 bias，则：
	 *       第1步实际 = (err+over) + bias - δ
	 *       第2步实际 = -over - bias + δ
	 *       净 = err                      ← bias 和 δ 都抵消掉了
	 *   于是 bias 在抖动阶段【完全无效】，学习也学不到东西。
	 *   所以抖动时置 s_bu_skip_bias=1，由抖动逻辑自己在第 1 步里
	 *   加恰好【一份】bias，保证「每次掉头只补一次」。 */
#if (CAR_TST_YAW_STOP_LEARN != 0)
	if ((g_car_turn_stop_bias != 0.0f) && (s_bu_skip_bias == 0U))
	{
		float soff = g_car_turn_stop_bias * CAR_BU_ARC_K;

		arc_mm += (angle_deg > 0.0f) ? soff : -soff;
	}
#endif

	/* 未标定的值单独存一份，方便 Watch 核对"×10 到底有没有做" */
	g_car_bu_last_arc = Bu_Round(arc_mm * 10.0f);

	delta = Bu_MmTo0p1mm(arc_mm);                           /* ★ 这里含 ×10 */

	/* 左轮 +弧长，右轮 -弧长 -> 车体原地转 */
	return Bu_StartMotion(delta, -delta, Bu_MmpsTo0p1mmps(v_mm_s), accel_0p1mmps2);
}

/**
  * @brief  ★ 转角闭环：用陀螺仪把车转到「绝对航向 target_deg」
  *
  *         逐次逼近：转 -> 等停稳 -> 读陀螺仪 -> 算残余误差 -> 再补转 -> ……
  *
  *         ★ 为什么传绝对航向而不是相对角度：
  *             调用者用「起始航向 + k×转角」累加目标，
  *             这样直行段跑偏的那几度会在下一个角自动纠回来。
  *             陀螺仪读数是 -180~+180 循环的，用 Bu_Wrap180 处理。
  *
  *         ★ 失败保护：
  *             · 补转了但陀螺仪几乎没变化 -> 记一次"没进展"，
  *               连续 2 次就退出（陀螺仪坏了 / 电机没执行）。
  *               没有这个保护的话，每轮都补转 90°，车会一直原地打转。
  *             · 最多补 max_iter 次。
  *
  * @param  target_deg      目标绝对航向 °
  * @param  speed_deg_s     角速度 °/s
  * @param  accel_0p1mmps2  加速度 0.1mm/s²，0 = 默认
  * @param  max_iter        最多补转几次
  * @param  tol_deg         允许误差 °
  * @retval 0 到位，1 未收敛或发送失败
  */
uint8_t Car_RotateToHeading(float target_deg, float speed_deg_s,
                            uint32_t accel_0p1mmps2,
                            uint8_t max_iter, float tol_deg)
{
	AngleData_t ad;
	float    prev_actual;
	float    err;
	float    learn_err = 0.0f;              /* ★ 学这个：第一条大转角之后的残余 */
	uint8_t  have_learn = 0U;
	uint8_t  i;
	uint8_t  stuck = 0U;

	if (tol_deg <= 0.0f)      { tol_deg = 0.5f; }
	if (max_iter == 0U)       { max_iter = 1U; }
	if (speed_deg_s == 0.0f)  { return 1U; }

	g_car_yaw_target = target_deg;
	g_car_yaw_iters  = 0U;

	Angle_GetData(&ad);
	prev_actual = ad.angle_deg;
	g_car_yaw_actual = prev_actual;

	for (i = 0U; i < max_iter; i++)
	{
		Angle_GetData(&ad);
		g_car_yaw_actual = ad.angle_deg;

		err = Bu_Wrap180(target_deg - ad.angle_deg);
		g_car_yaw_err = err;

		/* ★★★ 学习样本取【第一条大转角之后的残余】，不是最终残余 ★★★
		 *
		 *   为什么：抖动法会把最终残余压到 0.15° 以内，那时 err≈0，
		 *   学习（bias += K·err）就什么也学不到 —— 这正是之前的漏洞。
		 *   而真正反映"车每次都少转多少"的是【第一次转完之后差多少】。
		 *
		 *   时序：调用前已经转完 target 转角（含 bias 补偿），
		 *        第 1 次算出的 err 就是"还差多少" = bias - δ 的反号。
		 *        学习把它积起来 -> bias 收敛到 δ -> 初始转角就准了。 */
		if (have_learn == 0U)
		{
			learn_err  = err;
			have_learn = 1U;
		}

		if ((err <= tol_deg) && (err >= -tol_deg))
		{
			g_car_yaw_iters = i;
			Bu_TurnFinish(learn_err);              /* ★ 学掉这个固定残余 */
			return 0U;                       /* 已经在目标上 */
		}

		/* ★★★ 抖动法（dither）：把小步拆成"先过头、再回来"两次大步 ★★★
		 *
		 *  ★ 为什么需要：
		 *      补转残余只有零点几度时，弧长只有 1~2mm，
		 *      峰值速度 √(a·s) 只有 20~30mm/s，顶不动静摩擦（约需 40mm/s）。
		 *      指令发出去了车不动 -> 停滞检测接受现状 -> 残余就锁死了。
		 *
		 *  ★ 抖动法怎么破：
		 *      目标净转 err，但拆成两步，每步都给足速度：
		 *          第 1 步：转 (err + over)    弧长大，能走
		 *          第 2 步：转 -over           弧长大，能走
		 *          净转角 = (err+over) - over = err
		 *
		 *  ★ 额外好处：固定损失 δ 自动抵消
		 *          第 1 步实际 = (err+over) - δ
		 *          第 2 步实际 = -over + δ
		 *          净 = (err+over-δ) + (-over+δ) = err      ← δ 消掉了
		 *      所以抖动法不仅能突破静摩擦，还能自动补掉"每次都少转一点"。
		 *
		 *  ★ 只在 |err| 小于 CAR_DITHER_TRIGGER_DEG 时才用抖动，
		 *    残余较大时直接一步转过去（能走，不需要抖）。
		 */
#if (CAR_DITHER_ENABLE != 0)
		if (Bu_AbsF(err) < CAR_DITHER_TRIGGER_DEG)
		{
			float over = (err > 0.0f) ? CAR_DITHER_OVER_DEG : -CAR_DITHER_OVER_DEG;

			g_car_dither_cnt++;

			/* ★ 两步都关掉内层 bias：
			 *   抖动本身已经把固定损失 δ 抵消掉了（见上方推导），
			 *   再叠 bias 会让净转角变成 err+bias，等于补两次、过冲。
			 *   bias 只在【初始那条大转角】里生效一次，那才是它该干活的地方。 */
			s_bu_skip_bias = 1U;

			/* 第 1 步：转过头 */
			if (Car_RotateInPlaceEx(err + over, speed_deg_s,
			                        accel_0p1mmps2) != 0U)
			{
				s_bu_skip_bias = 0U;
				g_car_yaw_iters = i;
				return 1U;
			}
			Bu_WaitMotionDone(Bu_TurnTimeout(err + over, speed_deg_s));
			osDelay(CAR_TST_YAW_SETTLE_MS);

			/* 第 2 步：转回来（净转角 = err） */
			if (Car_RotateInPlaceEx(-over, speed_deg_s, accel_0p1mmps2) != 0U)
			{
				s_bu_skip_bias = 0U;
				g_car_yaw_iters = i;
				return 1U;
			}
			Bu_WaitMotionDone(Bu_TurnTimeout(over, speed_deg_s));
			osDelay(CAR_TST_YAW_SETTLE_MS);

			s_bu_skip_bias = 0U;

			Angle_GetData(&ad);
			g_car_yaw_actual = ad.angle_deg;

			prev_actual     = ad.angle_deg;
			g_car_yaw_iters = (uint8_t)(i + 1U);
			continue;                        /* 回到循环开头重新判容差 */
		}
#endif

		if (Car_RotateInPlaceEx(err, speed_deg_s, accel_0p1mmps2) != 0U)
		{
			g_car_yaw_iters = i;
			return 1U;                       /* 参数非法 / 发送失败 */
		}

		Bu_WaitMotionDone(Bu_TurnTimeout(err, speed_deg_s));
		osDelay(CAR_TST_YAW_SETTLE_MS);

		Angle_GetData(&ad);
		g_car_yaw_actual = ad.angle_deg;

		/* ★★ 停滞检测：补转了但陀螺仪几乎没动 -> 电机走不动（静摩擦/死区）。
		 *   ★ 这里【接受现状退出】，不返回失败：
		 *     调用者只看返回值不看角度，返回失败车就停在错误航向上，
		 *     紧接着又走 5m —— sin(2°)×5000 = 175mm，比残余本身严重得多。
		 *     剩下的残余交给 Bu_TurnFinish 学掉，下一个角接着纠。 */
		if (Bu_AbsF(Bu_Wrap180(ad.angle_deg - prev_actual)) < (tol_deg * 0.5f))
		{
			if (++stuck >= 2U)
			{
				g_car_yaw_err   = Bu_Wrap180(target_deg - ad.angle_deg);
				g_car_yaw_iters = (uint8_t)(i + 1U);
				Bu_TurnFinish(learn_err);
				return 0U;                   /* ★ 接受现状 */
			}
		}
		else
		{
			stuck = 0U;
		}

		prev_actual     = ad.angle_deg;
		g_car_yaw_iters = (uint8_t)(i + 1U);
	}

	Angle_GetData(&ad);
	g_car_yaw_actual = ad.angle_deg;
	g_car_yaw_err    = Bu_Wrap180(target_deg - ad.angle_deg);

	Bu_TurnFinish(learn_err);            /* ★ 迭代用完也要学 */

	return ((g_car_yaw_err <= tol_deg) && (g_car_yaw_err >= -tol_deg)) ? 0U : 1U;
}

/**
  * @brief  ★ 每次转角结束的统一收尾：统计 + 学习「停止点系统偏差」
  *
  *         ★ 为什么学这个：
  *           实测 5 轮累计偏 100mm，等价于每段有 0.23° 的【固定同向】残余。
  *           而容差已经到物理下限（0.3° 时电机就顶不动静摩擦了），
  *           再收紧只会卡住重试，收不出精度。
  *           唯一出路是把这 0.23° 学出来，用固定弧长补掉。
  *
  *         ★★ 更新律必须是纯积分  bias += K·err
  *            写成 bias += K·(err - bias) 等于 K·(真值 - 2·bias)，
  *            会收敛到【真值的一半】。
  *
  *         ★ 只在 |err| ≤ MAX_SAMPLE 时学（撞了/卡住的数据丢弃）。
  *
  * @param  err  这次转角的最终残余 °（正值 = 还差一点，转不够）
  * @retval None
  */
static void Bu_TurnFinish(float err)
{
	g_car_turn_corr_cnt += g_car_yaw_iters;

#if (CAR_TST_YAW_STOP_LEARN != 0)
	if (Bu_AbsF(err) <= CAR_TST_YAW_STOP_MAX_SAMPLE)
	{
		float b = g_car_turn_stop_bias + (CAR_TST_YAW_STOP_K * err);

		if (b >  CAR_TST_YAW_STOP_MAX) { b =  CAR_TST_YAW_STOP_MAX; }
		if (b < -CAR_TST_YAW_STOP_MAX) { b = -CAR_TST_YAW_STOP_MAX; }

		g_car_turn_stop_bias = b;
		g_car_turn_stop_n++;
	}
#endif
}

/**
  * @brief  单次转角 + 停稳补偿（方形专用，不学习）
  *
  *         流程：整段转到目标航向 -> 停稳读陀螺仪 -> 差多少补多少（只补一次）。
  *         不使用、不更新停止点偏差（g_car_turn_stop_bias），不抖动。
  *
  * @param  target_deg      目标绝对航向 °
  * @param  speed_deg_s     角速度上限 °/s
  * @param  accel_0p1mmps2  转向加速度
  * @param  tol_deg         到位容差 °
  * @param  settle_ms       每次读陀螺仪前等车停稳 ms
  * @retval 0 到位（残余在容差内），1 参数非法/发送失败，2 补转后残余仍超差
  */
static uint8_t Bu_TurnOnceCorrectEx(float target_deg, float speed_deg_s, uint32_t accel_0p1mmps2,
                                    float tol_deg, uint32_t settle_ms)
{
	AngleData_t ad;
	float err;
	uint8_t ret = 0U;

	/* 本次转角不叠加学到的停止点偏差，与陀螺仪原始读数直接比较 */
	s_bu_skip_bias = 1U;

	Angle_GetData(&ad);
	err = Bu_Wrap180(target_deg - ad.angle_deg);
	if (Car_RotateInPlaceEx(err, speed_deg_s, accel_0p1mmps2) != 0U)
	{
		s_bu_skip_bias = 0U;
		return 1U;
	}
	Bu_WaitMotionDone(Bu_TurnTimeout(err, speed_deg_s));
	osDelay(settle_ms);

	/* 停稳后读陀螺仪，差多少补多少 */
	Angle_GetData(&ad);
	err = Bu_Wrap180(target_deg - ad.angle_deg);
	if ((err > tol_deg) || (err < -tol_deg))
	{
		if (Car_RotateInPlaceEx(err, speed_deg_s, accel_0p1mmps2) != 0U)
		{
			s_bu_skip_bias = 0U;
			return 1U;
		}
		Bu_WaitMotionDone(Bu_TurnTimeout(err, speed_deg_s));
		osDelay(settle_ms);
		Angle_GetData(&ad);
		err = Bu_Wrap180(target_deg - ad.angle_deg);
	}

	s_bu_skip_bias = 0U;

	g_car_yaw_target = target_deg;
	g_car_yaw_actual = ad.angle_deg;
	g_car_yaw_err    = err;
	g_car_yaw_iters  = 1U;

	ret = ((err <= tol_deg) && (err >= -tol_deg)) ? 0U : 2U;
	return ret;
}

/* 方形专用包装：容差/停稳用 CAR_SQ_*，返回值语义不变（0 到位 / 1 失败或残余超差） */
static uint8_t Bu_TurnOnceCorrect(float target_deg, float speed_deg_s, uint32_t accel_0p1mmps2)
{
	return (Bu_TurnOnceCorrectEx(target_deg, speed_deg_s, accel_0p1mmps2,
	                             CAR_SQ_YAW_TOL_DEG, CAR_SQ_YAW_SETTLE_MS) == 0U) ? 0U : 1U;
}

/**
  * @brief  原地转向（轮子线速度入口）
  * @param  angle_deg         车体转角 °
  * @param  wheel_speed_mm_s  轮子线速度 mm/s
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t Car_RotateInPlaceWheel(float angle_deg, float wheel_speed_mm_s)
{
	float   arc_mm;
	float   gain = g_car_turn_gain;
	int32_t delta;

	if ((angle_deg == 0.0f) || (wheel_speed_mm_s == 0.0f))
	{
		return 1U;
	}

	/* 与 Car_RotateInPlaceEx 保持同一套增益补偿 */
	if (gain < 0.5f) { gain = 0.5f; }
	if (gain > 2.0f) { gain = 2.0f; }

	arc_mm = CAR_BU_ARC_K * angle_deg * gain;

	/* 固定弧长补偿，与 Car_RotateInPlaceEx 一致（含在线学到的停止点偏差） */
	if (g_car_turn_arc_off != 0.0f)
	{
		arc_mm += (angle_deg > 0.0f) ? g_car_turn_arc_off : -g_car_turn_arc_off;
	}
#if (CAR_TST_YAW_STOP_LEARN != 0)
	if (g_car_turn_stop_bias != 0.0f)
	{
		float soff = g_car_turn_stop_bias * CAR_BU_ARC_K;

		arc_mm += (angle_deg > 0.0f) ? soff : -soff;
	}
#endif

	g_car_bu_last_arc = Bu_Round(arc_mm * 10.0f);
	delta  = Bu_MmTo0p1mm(arc_mm);

	return Bu_StartMotion(delta, -delta, Bu_MmpsTo0p1mmps(wheel_speed_mm_s), 0U);
}

/**
  * @brief  停车：给两轮发"相对位移 0"，电机在当前位置保持
  *         不发 0x0413 释放，避免坡道溜车；需要下电释放请调 Car_BuRelease()
  * @retval None
  */
void Car_BuStop(void)
{
	(void)Bu_SendPosRel(CAR_BU_LEFT_MOTOR_NO,  0);   /* 左轮原地保持 */
	(void)Bu_SendPosRel(CAR_BU_RIGHT_MOTOR_NO, 0);   /* 右轮原地保持 */

	g_car_bu_last_delta = 0;
}

/**
  * @brief  下电释放
  *
  *         ★★★ 这里故意不发任何帧。
  *         实测 0x0413（0x5A 和 0x4A 都一样）一发出，电机立刻停止上报
  *         位移/力矩/心跳，随后连 ACK 都不回，只能断电重启才能恢复。
  *         所以固件全程不触碰 0x0413。
  * @retval None
  */
void Car_BuRelease(void)
{
	/* 0x0413 会把电机搞死，故意留空。需要松轴请直接断电机动力电源。 */
}

/**
  * @brief  诊断：两个轮子各走"一整圈"对应的线位移（π×75 = 235.62mm）
  *         ★ 把车架空，盯着一个轮子的辐条看：
  *             正好 1 圈    -> 电机内部轮径 = 75mm，0x0408 生效
  *             约 1/4 圈    -> 电机内部轮径 = 300mm，0x0408 没生效
  *             约 0.28 圈   -> 电机内部轮径 ≈ 270mm
  *         用原地反向的方式，车体不会跑掉。
  * @retval None
  */
void Car_BuDiagWheelOneRev(void)
{
	int32_t  delta = Bu_MmTo0p1mm(CAR_BU_ONE_REV_MM);
	uint32_t speed = Bu_MmpsTo0p1mmps(CAR_BU_DIAG_MMPS);

	g_car_bu_last_arc = Bu_Round(CAR_BU_ONE_REV_MM * 10.0f);   /* 2356 */

	(void)Bu_StartMotion(delta, -delta, speed, 0U);
}

/**
  * @brief  脱机动作序列：前进 -> 停 -> 后退 -> 停 -> 左转 90° -> 停 -> 右转 90° -> 停，循环
  *
  *         ★★★ 关于「两台电机都是设备号 0，怎么分别控制」
  *
  *             不需要让设备号不同！两台电机都挂在设备号 0 上，
  *             区分轮子靠的是「帧组」：
  *                 主帧组 0x0414 / 0x0415 / 0x0416  ->  A 轮
  *                 从帧组 0x0417 / 0x0418 / 0x0419  ->  B 轮
  *
  *             证明（同两条帧、只改符号，得到两种完全不同的结果）：
  *                 主 +4449 / 从 +4449  陀螺仪 +90.1°（车原地转）
  *                 主 -4449 / 从 -4449  陀螺仪 -90.0°（车反向原地转）
  *                 主 +2920 / 从 -2920  陀螺仪  +0.1°（车不转 = 直行）
  *                 主 -2920 / 从 +2920  陀螺仪  -0.1°（车不转 = 直行）
  *             如果两个通道不能各自驱动一个轮子，同号和异号不可能产生两种结果。
  *
  *         ★ 每步之间留 CAR_BU_SEQ_GAP_MS 停顿，方便肉眼分辨「直行」和「转向」。
  *
  * @param  None
  * @retval None
  */
void Car_BuRunTestSequence(void)
{
	/* 等 2s，让初始化时的参数真正被电机消化 */
	g_car_bu_seq_stage = 0U;
	osDelay(CAR_BU_SEQ_SETTLE_MS);

	for (;;)
	{
		/* 电机中途重启（0x0312）就自动重做一遍绑定和配置 */
		Bu_ReinitIfMotorRebooted();

		/* -------------------- 1. 前进 500mm -------------------- */
		g_car_bu_seq_stage = 1U;
		g_car_bu_seq_ret[0] = (int32_t)Car_MoveStraight(CAR_BU_SEQ_STRAIGHT_MM,
		                                                CAR_BU_SEQ_STRAIGHT_MMPS);
		Bu_WaitMotionDone(CAR_BU_SEQ_MOVE_TIMEOUT_MS);
		osDelay(CAR_BU_SEQ_GAP_MS);

		/* -------------------- 2. 后退 500mm 回起点 -------------------- */
		g_car_bu_seq_stage = 2U;
		g_car_bu_seq_ret[0] = (int32_t)Car_MoveStraight(-CAR_BU_SEQ_STRAIGHT_MM,
		                                                CAR_BU_SEQ_STRAIGHT_MMPS);
		Bu_WaitMotionDone(CAR_BU_SEQ_MOVE_TIMEOUT_MS);
		osDelay(CAR_BU_SEQ_GAP_MS);

		/* ---------- 3. 原地左转 90°（主 +arc / 从 +arc） ---------- */
		g_car_bu_seq_stage = 3U;
		g_car_bu_seq_ret[1] = (int32_t)Car_RotateInPlace(CAR_BU_SEQ_TURN_DEG,
		                                                 CAR_BU_SEQ_TURN_DPS);
		Bu_WaitMotionDone(CAR_BU_SEQ_MOVE_TIMEOUT_MS);
		osDelay(CAR_BU_SEQ_GAP_MS);

		/* ---------- 4. 原地右转 90°（回转复位） ---------- */
		g_car_bu_seq_stage = 4U;
		g_car_bu_seq_ret[1] = (int32_t)Car_RotateInPlace(-CAR_BU_SEQ_TURN_DEG,
		                                                 CAR_BU_SEQ_TURN_DPS);
		Bu_WaitMotionDone(CAR_BU_SEQ_MOVE_TIMEOUT_MS);
		osDelay(CAR_BU_SEQ_GAP_MS);

		/* -------------------- 5. 停车歇一下 -------------------- */
		g_car_bu_seq_stage = 5U;
		Car_BuStop();
		osDelay(1000U);
	}
}

/* ==========================================================================
 *                   课题四：直线往返测试（5m × 5 个来回）
 * ==========================================================================
 *
 *  流程（1 轮 = CAR_RT_ROUNDS 段 = 5 个来回）：
 *      前进 CAR_RT_DISTANCE_MM -> 掉头 CAR_RT_TURN_DEG
 *      -> 前进 CAR_RT_DISTANCE_MM -> 掉头 CAR_RT_TURN_DEG
 *      …… 重复 CAR_RT_ROUNDS 次（10 段 = 5 个来回），然后停车待机
 *
 *  ★ 跑完 5 个来回即停（CAR_RT_REPEAT = 1）。
 *    最后那一次掉头会把车摆回初始朝向，便于直接量落点。
 *    总行程 50m，约 2 分钟。
 *
 *  ★ 为什么前进方向永远传正数、靠掉头 180° 换方向：
 *    比每次都改符号更不容易出错，而且 180° 掉头和「车头朝哪」解耦。
 *    转一次 180°，下一段还是"前进"，车自然会往回走。
 *
 *  ★ 直行不再分段停顿（用户要求连续走）：
 *    每段都是一条 0x0416 走 5m，靠【两轮目标位移差 trim】做微弧补偿，
 *    trim 在线学习（见 Car_MoveStraightHdg 的注释）。
 *
 *  参数全在 car_bringup.h 的 CAR_RT_* 系列里，本函数不含任何魔法数字。
 */
void Car_BuRunRoundTrip(void)
{
	AngleData_t ad;
	uint32_t round, leg;
	uint32_t turn_to = Bu_TurnTimeout(CAR_RT_TURN_DEG,  CAR_RT_TURN_DPS);
	float    h_target;

	g_car_tst_done = 0U;
	g_car_bu_abort = 0U;
	g_car_bu_turnerr = 0U;
	g_car_bu_seq_stage = 0U;

	/* 等 2s，让初始化时的参数真正被电机消化 */
	osDelay(CAR_BU_SEQ_SETTLE_MS);

#if (CAR_RT_DIR_PRECHECK != 0)
	/* ★★★ 方向预检：慢速前进 300mm -> 停 2s -> 退回 300mm -> 停 2s ★★★
	 *
	 *  固件没有前向传感器，只有陀螺仪，而陀螺仪分不出"往前走"和"往后走"
	 *  （两者都是直线、转角都是 0）。所以【只有人的眼睛能判断】。
	 *
	 *  方向表 {+1,-1} 与 {-1,+1} 互为整体取反：填错时车会整段倒着走
	 *  （不是转圈，因为直行仍然直，只是前后反）。跑完 50m 才发现
	 *  代价太大，所以先走 300mm 让人看一眼。
	 *
	 *  判读：这一段【远离你】= 正确；【靠近你】= 互换 car_cfg.h 的两个符号。
	 */
	g_car_bu_seq_stage = 4U;

	(void)Car_MoveStraightEx(CAR_DT_DIST_MM, CAR_DT_SPEED_MMPS, CAR_TST_ACCEL_0P1MMPS2);
	if (Bu_WaitMotionDone(Bu_MoveTimeout(CAR_DT_DIST_MM, CAR_DT_SPEED_MMPS)) == 0U)
	{
		g_car_bu_abort = 1U;
	}

	if (g_car_bu_abort == 0U)
	{
		osDelay(CAR_DT_DWELL_MS);

		(void)Car_MoveStraightEx(-CAR_DT_DIST_MM, CAR_DT_SPEED_MMPS, CAR_TST_ACCEL_0P1MMPS2);
		if (Bu_WaitMotionDone(Bu_MoveTimeout(CAR_DT_DIST_MM, CAR_DT_SPEED_MMPS)) == 0U)
		{
			g_car_bu_abort = 1U;
		}

		osDelay(CAR_DT_DWELL_MS);
	}

	if (g_car_bu_abort != 0U)
	{
		g_car_bu_seq_stage = 3U;
		Car_BuStop();
		g_car_tst_done = 1U;
		for (;;) { osDelay(1000U); }
	}
	g_car_bu_seq_stage = 0U;
#endif

	/* 起始航向：后面每个角都是「起始 + k×180」，误差不会累积
	 *
	 * ★★★ 侧偏补偿（CAR_CROSS_BIAS_DEG）★★★
	 *
	 *  背景：所有掉头/直行闭环都调到最好后，落点仍固定偏左 12mm，
	 *  而且改 seed 完全没用。二维仿真找到了原因：
	 *
	 *    ┌────────────────────────────────┬──────────┬─────────────┐
	 *    │ 误差类型                        │ 10 段落点 │ 陀螺仪可见?  │
	 *    ├────────────────────────────────┼──────────┼─────────────┤
	 *    │ 车体坐标系侧滑（轮径差/打滑）    │  -0.1 mm │  看不到      │
	 *    │ 世界坐标系固定侧滑（地坪倾斜）    │ +11.9 mm │  看不到      │
	 *    └────────────────────────────────┴──────────┴─────────────┘
	 *
	 *  为什么车体侧滑会抵消：它随行进方向交替，去程偏左、回程偏右。
	 *  为什么世界侧滑不抵消：地坪往哪边斜是【房间的属性】，与车头朝哪无关。
	 *  去程往 +y 滑，回程也往 +y 滑 -> 10 段全部同向累加 -> 12mm。
	 *  陀螺仪只测【转角】，侧滑是纯平移，任何旋转闭环都看不见它。
	 *
	 *  补偿原理：让车【斜着走】一点点，用行进方向的 y 分量抵消侧滑。
	 *      第 k 段目标航向 H_k = 起始 + 180k + (−1)^k · θ
	 *      则每段的 y 分量都是 +D·sin(θ)（同向！不交替）
	 *          k=0: 5000·sin(θ)      k=1: 5000·sin(180−θ)=+5000·sin(θ)
	 *      总补偿 = 10 · D · sin(θ)，要抵消 12mm：
	 *          θ = asin(12 / (10 × 5000)) = 0.01375°
	 *
	 *  相邻两段的转向增量因此变成 180 ∓ 2θ（见下面的 h_target += ...）。
	 *
	 *  ★ 怎么定 θ 的方向（必须实测）：
	 *      0.01375 是"落点往左 12mm"对应的修正量。
	 *      若改成右偏，就把 θ 取负。
	 *  ★ 这个值依赖摆车的位置/房间地坪，换地方要重标。
	 */
	Angle_SetScalePpm(CAR_RT_YAW_SCALE_PPM);        /* 往返专用标度修正；必须在读起始航向之前设置 */
	Angle_GetData(&ad);
	h_target = ad.angle_deg + CAR_CROSS_BIAS_DEG;   /* 第 0 段就带上侧偏补偿 */
	g_car_hdg_trim_mm = CAR_RT_TRIM_INIT_MM;        /* 往返专用起点，不影响方形的 CAR_HC_TRIM_INIT_MM */

	for (round = 0U; round < CAR_RT_REPEAT; round++)
	{
		g_car_rt_round = round;

		for (leg = 0U; leg < CAR_RT_ROUNDS; leg++)
		{
			g_car_rt_leg = leg;

			/* ---------- 前进 ---------- */
			/* 电机中途重启（0x0312）就自动重做一遍绑定和配置 */
			Bu_ReinitIfMotorRebooted();

			g_car_bu_seq_stage = 1U;
			g_car_bu_seq_ret[0] = (int32_t)Car_MoveStraightHdg(CAR_RT_DISTANCE_MM,
			                                                  CAR_RT_SPEED_MMPS,
			                                                  CAR_RT_ACCEL_0P1MMPS2,
			                                                  CAR_RT_HEADING_SEGMENTS,
			                                                  h_target, 1U);
			/* ★ 不要再等一次：Car_MoveStraightHeading 内部已逐段等到位，
			 *   外面再等只会白等"物理最短时间"并干扰反馈缺失判据。 */
			osDelay(CAR_RT_GAP_MS);

			/* ★★★ 直行没走完就【整轮测试停掉】★★★
			 *
			 *   MoveStraightHeading 返回 1 只有两种原因：
			 *     ① 参数非法（不会发生）
			 *     ② 某一段等不到 0x0420 剩余位移归零
			 *   ② 意味着【无法确认车停下来了】。这时继续往下走会发掉头指令，
			 *   万一车还在动，就是"运动中途改目标" -> 电机当场瘫痪，
			 *   只能断电重启。所以宁可终止整轮测试。
			 *   终止后 Car_BuStop() 会发停车；车若已停，不会有副作用。 */
			if (g_car_bu_seq_ret[0] != 0)
			{
				g_car_bu_abort = 1U;
				break;
			}

			/* ---------- 掉头 180° ---------- */
			Bu_ReinitIfMotorRebooted();

			g_car_bu_seq_stage = 2U;

			h_target += CAR_RT_TURN_DEG - CAR_RT_TURN_TRIM_DEG
			          + (((leg % 2U) == 0U) ? (-2.0f * CAR_CROSS_BIAS_DEG)
			                                : (+2.0f * CAR_CROSS_BIAS_DEG));
			/* ★ 为什么是 ∓2θ：
			 *   要的航向序列 H_k = 起始 + 180k + (−1)^k·θ
			 *   相邻差  H_{k+1} − H_k = 180 − 2·(−1)^k·θ
			 *   所以第 0 段转完要少 2θ，第 1 段转完要多 2θ，交替。
			 *   这样每段的行进方向 y 分量都同号（见上面的推导），
			 *   才能累加出抵消地坪倾斜的位移。 */

#if (CAR_TST_YAW_CLOSE != 0)
			/* 闭环：转到绝对航向，停稳读陀螺仪，差多少补一次（与方形同算法，参数独立，不抖动不学习）
			 * 返回 2 = 补转后残余仍超容差，接受现状继续跑（与原闭环一致）；只有 1（发送失败）才终止 */
			(void)turn_to;
			g_car_bu_seq_ret[1] = (int32_t)Bu_TurnOnceCorrectEx(h_target,
			                                                   CAR_RT_TURN_DPS,
			                                                   CAR_RT_TURN_ACCEL_0P1MMPS2,
			                                                   CAR_RT_YAW_TOL_DEG,
			                                                   CAR_RT_YAW_SETTLE_MS);
			if (g_car_bu_seq_ret[1] == 2)
			{
				g_car_bu_seq_ret[1] = 0;
			}
#else
			(void)h_target;
			g_car_bu_seq_ret[1] = (int32_t)Car_RotateInPlaceEx(CAR_RT_TURN_DEG,
			                                                  CAR_RT_TURN_DPS,
			                                                  CAR_RT_TURN_ACCEL_0P1MMPS2);
			Bu_WaitMotionDone(turn_to);
#endif
			osDelay(CAR_RT_GAP_MS);

			/* ★★★ 掉头也要检查结果 ★★★
			 *
			 *   原来这里直接往下走，掉头失败（返回 1）会被【静默忽略】，
			 *   表现就是「直行完了没转弯就继续走下一段」。
			 *
			 *   现在失败就终止整轮测试，让车停下来，不再乱走。
			 *   掉头返回 1 的原因见 Car_RotateToHeading：
			 *     ① 发送失败（CAN 出错 / 电机没就绪）
			 *     ② 参数非法
			 *   （陀螺仪不收敛【不算失败】，那里会接受现状返回 0） */
			if (g_car_bu_seq_ret[1] != 0)
			{
				g_car_bu_abort   = 1U;
				g_car_bu_turnerr = (uint8_t)((g_car_bu_turnerr < 200U) ? (g_car_bu_turnerr + 1U) : 200U);
				break;
			}
		}

		if (g_car_bu_abort != 0U)
		{
			break;                           /* 直行没走完，整轮测试终止 */
		}
	}

	/* ---------- 收尾：停车待机 ---------- */
	g_car_bu_seq_stage = 3U;
	Car_BuStop();
	g_car_tst_done = 1U;

	for (;;)
	{
		osDelay(1000U);
	}
}

/* ==========================================================================
 *                   课题五：方形测试（2m 边长 × 5 个 × 1 轮）
 * ==========================================================================
 *
 *  流程（每个正方形）：
 *      前进 CAR_SQ_SIDE_MM -> 转 CAR_SQ_TURN_DEG
 *      -> 前进 CAR_SQ_SIDE_MM -> 转 CAR_SQ_TURN_DEG
 *      …… 四条边各走一次，回到起点
 *
 *  ★ 顺时针/逆时针只由 CAR_SQ_CLOCKWISE 决定（见 Car_BuRunSquare 内的 #if）。
 *
 *  ★ 边长 2m × 5 个 × 5 轮 = 50m 行程，按 1000mm/s 纯直行约 0.8 分钟，
 *    加上转向和停顿，总时长以实测为准。要缩短就调 CAR_TST_SPEED_MMPS。
 */
void Car_BuRunSquare(void)
{
	AngleData_t ad;
	float    turn_deg;
	uint32_t round, loop, side;
	uint32_t turn_to = Bu_TurnTimeout(CAR_SQ_TURN_DEG, CAR_SQ_TURN_DPS);
	float    h_target;

	g_car_tst_done = 0U;

	/* 顺时针：Car_RotateInPlaceEx 约定「正 = 左转」，航向增加 = 左转。
	 * 实车验证方向反了，故这里翻转符号：CAR_SQ_CLOCKWISE=1 时用 +90。 */
#if (CAR_SQ_CLOCKWISE != 0)
	turn_deg = CAR_SQ_TURN_DEG;
#else
	turn_deg = -CAR_SQ_TURN_DEG;
#endif

	g_car_bu_seq_stage = 0U;
	osDelay(CAR_BU_SEQ_SETTLE_MS);

	/* ★ 起始航向只读一次。之后每个角的目标 = 起始 + k×90，
	 *   于是每个角都是【绝对角度】，上一个角差的那点、直行段跑偏的那点，
	 *   都会在下一个角被纠回来，不会累积成"扇形"。 */
	Angle_GetData(&ad);
	h_target = ad.angle_deg;              /* 方形测试不加侧偏补偿（往返专用）*/

	for (round = 0U; round < CAR_SQ_REPEAT; round++)
	{
		g_car_sq_round = round;

		for (loop = 0U; loop < CAR_SQ_LOOPS; loop++)
		{
			g_car_sq_loop = loop;

			for (side = 0U; side < 4U; side++)
			{
				g_car_sq_side = side;

				/* ---------- 走一条边 ---------- */
				Bu_ReinitIfMotorRebooted();

				g_car_bu_seq_stage = 1U;
				g_car_bu_seq_ret[0] = (int32_t)Car_MoveStraightHdg(CAR_SQ_SIDE_MM
				                                                  + ((side == 0U) ? CAR_SQ_SIDE_TRIM_MM
				                                                  : ((side == 2U) ? -CAR_SQ_SIDE_TRIM_MM : 0.0f)),
				                                                  CAR_SQ_SPEED_MMPS,
				                                                  CAR_TST_ACCEL_0P1MMPS2,
				                                                  CAR_SQ_HEADING_SEGMENTS,
				                                                  h_target, 1U);
				/* ★ 同上：内部已逐段等到位，不再重等 */
				osDelay(CAR_SQ_GAP_MS);

				/* ---------- 转一个角 ---------- */
				Bu_ReinitIfMotorRebooted();

				g_car_bu_seq_stage = 2U;

				h_target += turn_deg;

#if (CAR_TST_YAW_CLOSE != 0)
				/* 闭环：转到绝对航向，差多少补多少 */
				(void)turn_to;
				g_car_bu_seq_ret[1] = (int32_t)Bu_TurnOnceCorrect(h_target,
				                                                 CAR_SQ_TURN_DPS,
				                                                 CAR_TST_TURN_ACCEL_0P1MMPS2);
#else
				(void)h_target;
				g_car_bu_seq_ret[1] = (int32_t)Car_RotateInPlaceEx(turn_deg,
				                                                  CAR_SQ_TURN_DPS,
				                                                  CAR_TST_TURN_ACCEL_0P1MMPS2);
				Bu_WaitMotionDone(turn_to);
#endif
				osDelay(CAR_SQ_GAP_MS);
			}
		}
	}

	/* ---------- 收尾：停车待机 ---------- */
	g_car_bu_seq_stage = 3U;
	Car_BuStop();
	g_car_tst_done = 1U;

	for (;;)
	{
		osDelay(1000U);
	}
}

/* ==========================================================================
 *              ★★★ 原地 360° 转圈标定（纯诊断）★★★
 * ==========================================================================
 *
 *  原理、为什么正反转各跑一遍、为什么每圈都要记、循环论证警告，
 *  全部写在 car_bringup.h 的 CAR_SPIN_* 那一段里，这里只讲实现要点。
 *
 *  实现要点：
 *    ① 标定前【必须把 g_car_turn_gain 复位成 1.0】
 *       否则上一次标定的补偿会被算进这一次的误差里，越标越偏
 *       （和差速标定要先清零是同一个道理）。
 *    ② 每圈发一条【360°】指令，转完等停稳再读——不在转动中途读，
 *       因为减速段车还在滑，这时的读数包含"还在动"的成分。
 *    ③ 每圈的残余都是【相对该方向起始点】的累积值，不是单圈增量。
 *       这样 Watch 里看到的就是一条直线（纯增益误差）或一条曲线（打滑）。
 *    ④ 用 Wrap180 处理 -180~+180 环绕；超过 CAR_SPIN_ABNORMAL_DEG
 *       就认为可能已经混叠，把结果标成不可信（不自动应用增益）。
 */

/** @brief 单方向连续转 N 圈，每圈记一次累积残余；返回该方向的总残余 ° */
static float Bu_SpinOneDirection(int8_t sign, volatile float *loop_rec)
{
	AngleData_t ad;
	float    h0;
	float    res = 0.0f;
	uint32_t i;
	uint32_t n = CAR_SPIN_LOOPS;

	if (n == 0U)                { return 0.0f; }
	if (n > CAR_SPIN_MAX_LOOPS) { n = CAR_SPIN_MAX_LOOPS; }

	g_car_spin_loop = 0U;

	/* 等停稳、等陀螺仪安静下来，再取基准 */
	osDelay(CAR_TST_YAW_SETTLE_MS * 2U);
	Angle_GetData(&ad);
	h0 = ad.angle_deg;

	for (i = 0U; i < n; i++)
	{
		g_car_spin_loop = i;
		Bu_ReinitIfMotorRebooted();

		/* ★ 一条指令转满 360°，中途不停、不改目标 ——
		 *   中途改目标会把电机搞死（实测只能断电重启）。 */
		(void)Car_RotateInPlaceEx(360.0f * (float)sign,
		                          CAR_SPIN_DPS,
		                          CAR_TST_TURN_ACCEL_0P1MMPS2);
		Bu_WaitMotionDone(Bu_TurnTimeout(360.0f, CAR_SPIN_DPS));

		/* 减速段还在滑，读早了会把"正在滑"当成最终值 */
		osDelay(CAR_TST_YAW_SETTLE_MS * 2U);

		Angle_GetData(&ad);
		res = Bu_Wrap180(ad.angle_deg - h0);       /* 相对本方向起点的累积残余 */

		loop_rec[i] = res;
	}

	/* 把没用到的槽清零，免得 Watch 里看到上一次的旧数据 */
	for (i = n; i < CAR_SPIN_MAX_LOOPS; i++)
	{
		loop_rec[i] = 0.0f;
	}

	return res;
}

/**
  * @brief  ★★★ 开环转角物理验证：转 5 圈 360°，全程不读陀螺仪 ★★★
  *
  *  ★★★ 这个测试存在的唯一理由 ★★★
  *      "陀螺仪不准"和"电机没转到位"在数据上【完全无法区分】——
  *      陀螺仪多读 0.43% 和 车少转 0.43% 会产生一模一样的读数。
  *      要把它们分开，必须有一个【不经过陀螺仪】的基准。
  *
  *      本测试用的 Car_RotateInPlaceEx() 是【纯开环】的：
  *          arc = 轮距 × π × 角度 / 360 × gain
  *      算完弧长直接发指令，全程不读陀螺仪。
  *      所以只要它转不准，问题必然在标尺（轮距/轮径/gain），
  *      与陀螺仪一丁点关系都没有。
  *
  *  ★ 怎么判读（全程只用眼睛 + 一条地面标记线）：
  *      车头对准地面线 -> 跑 -> 每圈停 1.5 秒 -> 5 圈后停住
  *          第 1 圈后偏 X°，第 5 圈后偏 5X°
  *          X ≈ 0      标尺正确
  *          X ≈ +1.55° 弧长多发了 0.43%（gain 该是 99.57% 却按 100% 算）
  *          X ≈ -1.55° 弧长少发了
  *
  *      5 圈后肉眼能明显看出偏（>3°）= 标尺问题；
  *      几乎看不出（<1°）= 标尺正确，之前那 1.4° 就只能是陀螺仪的问题。
  *
  *  ★ 为什么角速度用 60°/s 而不是 90：
  *      转慢一点（每圈 6 秒），加速/减速段占比更小，残余更干净。
  *
  * @param  None（永不返回）
  * @retval None
  */
void Car_BuRunTurnTest(void)
{
	uint8_t i;

	g_car_tst_done     = 0U;
	g_car_bu_abort     = 0U;
	g_car_bu_seq_stage = 0U;

	/* 等电机参数被消化 */
	osDelay(CAR_BU_SEQ_SETTLE_MS);

	for (i = 0U; i < CAR_TT_TURNS; i++)
	{
		Bu_ReinitIfMotorRebooted();

		g_car_bu_turn_i    = (uint8_t)(i + 1U);
		g_car_bu_seq_stage = 1U;

		/* ★ 开环：只按标尺算弧长下发，不参考陀螺仪任何读数 */
		g_car_bu_seq_ret[1] = (int32_t)Car_RotateInPlaceEx(360.0f,
		                                                   CAR_TT_SPS,
		                                                   CAR_TST_TURN_ACCEL_0P1MMPS2);
		if (Bu_WaitMotionDone(Bu_TurnTimeout(360.0f, CAR_TT_SPS)) == 0U)
		{
			g_car_bu_abort = 1U;
			break;
		}

		/* 停 1.5 秒：让人眼看车头停在哪个方向 */
		g_car_bu_seq_stage = 2U;
		osDelay(CAR_TT_DWELL_MS);
	}

	/* ---------- 收尾：停车待机 ---------- */
	g_car_bu_seq_stage = 3U;
	Car_BuStop();
	g_car_tst_done = 1U;

	for (;;)
	{
		osDelay(1000U);
	}
}

/**
  * @brief  ★★★ 方向自检：慢速前进 300mm -> 停 2s -> 后退 300mm -> 停 ★★★
  *
  *  ★ 这个测试存在的唯一理由：
  *      固件没有前向传感器，只有陀螺仪。而陀螺仪分不出"往前走"和"往后走"
  *      —— 两者都是直线、转角都是 0。所以【只有人的眼睛能判断】。
  *
  *      方向表 g_car_dir_sign 填错时，车会整段倒着走（不是转圈，因为
  *      {+1,-1} 和 {-1,+1} 互为整体取反，直行仍然直，只是前后反）。
  *      这个测试把"看方向对不对"的成本降到最低：300mm、150mm/s、2 秒。
  *
  *  ★ 判读：
  *      第一段【远离你】-> 正确
  *      第一段【靠近你】-> 反了，互换 car_cfg.h 里的
  *                          CAR_LEFT_DIR_SIGN / CAR_RIGHT_DIR_SIGN
  *
  * @param  None（永不返回）
  * @retval None
  */
void Car_BuRunDirTest(void)
{
	g_car_tst_done     = 0U;
	g_car_bu_abort     = 0U;
	g_car_bu_seq_stage = 0U;

	osDelay(CAR_BU_SEQ_SETTLE_MS);

	/* ---------- 1. 前进 ---------- */
	Bu_ReinitIfMotorRebooted();
	g_car_bu_seq_stage = 1U;
	g_car_bu_seq_ret[0] = (int32_t)Car_MoveStraightEx(CAR_DT_DIST_MM,
	                                                 CAR_DT_SPEED_MMPS,
	                                                 CAR_TST_ACCEL_0P1MMPS2);
	if (Bu_WaitMotionDone(Bu_MoveTimeout(CAR_DT_DIST_MM, CAR_DT_SPEED_MMPS)) == 0U)
	{
		g_car_bu_abort = 1U;
	}

	if (g_car_bu_abort == 0U)
	{
		osDelay(CAR_DT_DWELL_MS);          /* 停 2s，让人看清是远离还是靠近 */

		/* ---------- 2. 后退回原位 ---------- */
		g_car_bu_seq_stage = 2U;
		g_car_bu_seq_ret[0] = (int32_t)Car_MoveStraightEx(-CAR_DT_DIST_MM,
		                                                 CAR_DT_SPEED_MMPS,
		                                                 CAR_TST_ACCEL_0P1MMPS2);
		if (Bu_WaitMotionDone(Bu_MoveTimeout(CAR_DT_DIST_MM, CAR_DT_SPEED_MMPS)) == 0U)
		{
			g_car_bu_abort = 1U;
		}
	}

	/* ---------- 收尾 ---------- */
	g_car_bu_seq_stage = 3U;
	Car_BuStop();
	g_car_tst_done = 1U;

	for (;;)
	{
		osDelay(1000U);
	}
}

/**
  * @brief  ★ 原地 360° 转圈标定：正转 N 圈 + 反转 N 圈，差分算增益误差
  *
  *         结果：
  *             g_car_turn_gain       自动应用（CAR_SPIN_APPLY_GAIN=1 时）
  *             g_car_spin_wb_suggest 建议的 CAR_WHEEL_BASE_MM（想固化到编译期用）
  *             g_car_spin_bias_drift 陀螺仪零偏漂移（改轮距修不掉，只能报告）
  * @param  None（永不返回）
  * @retval None
  */
void Car_BuRunSpinCal(void)
{
	float    n;
	float    g;                     /* 总有效增益误差（含已生效的 gain）*/
	float    gain_old;
	float    gain_new;
	uint8_t  abnormal;
	float    total_time_s = 0.0f;
	uint32_t t_start;

	g_car_spin_phase      = 0U;
	g_car_spin_loop       = 0U;
	g_car_spin_fwd_total  = 0.0f;
	g_car_spin_rev_total  = 0.0f;
	g_car_spin_verify_per = 0.0f;
	g_car_spin_verified   = 0U;
	{
		uint8_t k;
		for (k = 0U; k < CAR_SPIN_MAX_LOOPS; k++)
		{
			g_car_spin_verify_loop[k] = 0.0f;
		}
	}
	g_car_spin_fwd_per    = 0.0f;
	g_car_spin_rev_per    = 0.0f;
	g_car_spin_gain_err   = 0.0f;
	g_car_spin_bias_drift = 0.0f;
	g_car_spin_wb_suggest = 0.0f;

	/* ★ 标定前先把增益复位成 1.0
	 *   否则上一次的补偿会被算进这一次测到的误差里，越标越偏。 */
	gain_old             = g_car_turn_gain;
	g_car_spin_gain_prev = gain_old;

#if (CAR_SPIN_ENABLE == 0)
	g_car_spin_phase = 3U;
	for (;;) { osDelay(1000U); }
#else
	g_car_turn_gain = 1.0f;

	g_car_bu_seq_stage = 0U;
	osDelay(CAR_BU_SEQ_SETTLE_MS);

	/* ---------------- ① 正转 ---------------- */
	g_car_bu_seq_stage   = 2U;
	g_car_spin_phase     = 1U;
	t_start              = HAL_GetTick();
	g_car_spin_fwd_total = Bu_SpinOneDirection(+1, g_car_spin_fwd_loop);
	total_time_s         = (float)(HAL_GetTick() - t_start) / 1000.0f;

	osDelay(CAR_TST_YAW_SETTLE_MS * 2U);

	/* ---------------- ② 反转 ---------------- */
	g_car_spin_phase     = 2U;
	g_car_spin_rev_total = Bu_SpinOneDirection(-1, g_car_spin_rev_loop);

	/* ---------------- ③ 差分算增益与漂移 ----------------
	 *   正转：res_fwd = +360N·g + b·T
	 *   反转：res_rev = -360N·g + b·T
	 *        g   = (fwd - rev) / (2 × 360N)
	 *        b·T = (fwd + rev) / 2
	 *   —— 一加一减就把「增益误差」和「零偏漂移」彻底分开了。 */
	n = (float)CAR_SPIN_LOOPS;
	if (n < 1.0f) { n = 1.0f; }

	g_car_spin_fwd_per = g_car_spin_fwd_total / n;
	g_car_spin_rev_per = g_car_spin_rev_total / n;

	g = (g_car_spin_fwd_total - g_car_spin_rev_total) / (2.0f * 360.0f * n);
	g_car_spin_gain_err = g;

	/* 零偏漂移：把同号的共模部分换算成 °/分钟 */
	if (total_time_s > 1.0f)
	{
		float bT = (g_car_spin_fwd_total + g_car_spin_rev_total) * 0.5f;
		g_car_spin_bias_drift = bT / (total_time_s / 60.0f);
	}

	/* 超出阈值就认为可能混叠或严重超差，结果不可信 */
	abnormal = 0U;
	if ((g_car_spin_fwd_per >  CAR_SPIN_ABNORMAL_DEG) ||
	    (g_car_spin_fwd_per < -CAR_SPIN_ABNORMAL_DEG) ||
	    (g_car_spin_rev_per >  CAR_SPIN_ABNORMAL_DEG) ||
	    (g_car_spin_rev_per < -CAR_SPIN_ABNORMAL_DEG))
	{
		abnormal = 1U;
	}

	if (abnormal == 0U)
	{
		/* ★ 新增益 = 1 / (1 + g)
		 *
		 *   注意这里【不能】再乘 gain_old：
		 *   上面已经把 g_car_turn_gain 复位成 1.0 才开始测，
		 *   所以测到的 g 就是【原始物理增益误差】，不含任何补偿。
		 *   直接取倒数即为绝对补偿值。
		 *   （若写成 gain_old/(1+g)，第二次标定就会叠加补偿，越标越飞。）
		 *
		 *   g > 0（转多了）-> 新增益 < 1 -> 少发弧长，正确。 */
		gain_new = 1.0f;
		if ((1.0f + g) > 0.1f)
		{
			gain_new = 1.0f / (1.0f + g);
		}
		if (gain_new < 0.5f) { gain_new = 0.5f; }
		if (gain_new > 2.0f) { gain_new = 2.0f; }

#if (CAR_SPIN_APPLY_GAIN != 0)
		g_car_turn_gain = gain_new;
#else
		(void)gain_new;
		g_car_turn_gain = gain_old;
#endif

		/* 建议的轮距（想固化到编译期常量的用这个）
		 *
		 * ★ 推导（别搞反方向）：
		 *      下发弧长 d = 轮距 × π/360 × 角度 × 增益
		 *      实际转角 θ = 2d / 真实轮距
		 *   => θ_实际 / θ_指令 = 增益 × 轮距假定 / 真实轮距
		 *
		 *   g = θ_实际/θ_指令 - 1。标定时增益已复位成 1，所以
		 *      g = 轮距假定/真实轮距 - 1
		 *   => 真实轮距 = 轮距假定 / (1 + g)
		 *
		 *   g > 0（转多了）说明"假定轮距"偏大 -> 必须【调小】，不是调大。
		 *   （早期版本这里写成 ×(1+g)，方向反了，会让误差翻倍。） */
		g_car_spin_wb_suggest = (1.0f + g) > 0.1f
		                        ? ((float)CAR_WHEEL_BASE_MM / (1.0f + g))
		                        : ((float)CAR_WHEEL_BASE_MM);
	}
	else
	{
		/* 数据不可信：把增益还回去，不要拿坏数据污染后续测试 */
		g_car_turn_gain       = gain_old;
		g_car_spin_wb_suggest = (float)CAR_WHEEL_BASE_MM;
	}

	/* ---------------- ④ ★ 补偿后复测（这才是"补偿到底有没有用"的答案）----------------
	 *
	 *   ★ 为什么必须有这一步：
	 *     上面 ①②③ 全程把 g_car_turn_gain 复位成 1.0 在测【原始物理误差】。
	 *     所以跑完 SPIN 永远会看到"每圈偏 1.4°" —— 那是对的、也是必须的，
	 *     但很容易被误读成"补偿没生效"。
	 *
	 *     把补偿真的加上再转一遍，才能证明它到底有效没效。
	 *     （和 ANGLECHK 里的 verify 是同一个思路。）
	 *
	 *   ★ 只做正转：反转会引入零偏的共模项，混淆判断。
	 *     正转复测的残余 = 360N·(g + 补偿) + b·T，b·T 那条腿只有几十毫秒
	 *     到几秒的量级（~0.01°），可以忽略。 */
	if (abnormal == 0U)
	{
		float base = g_car_spin_fwd_per;          /* 补偿前每圈偏差 */
		uint8_t last_idx = 0U;                    /* ★ 用实际生效的圈数索引，防越界 */

		if (n >= 1.0f)
		{
			last_idx = (uint8_t)((uint32_t)n - 1U);
		}
		if (last_idx >= CAR_SPIN_MAX_LOOPS)
		{
			last_idx = (uint8_t)(CAR_SPIN_MAX_LOOPS - 1U);
		}

		g_car_spin_phase = 3U;                    /* 3 = 复测中 */

		(void)Bu_SpinOneDirection(+1, g_car_spin_verify_loop);
		g_car_spin_verify_per = g_car_spin_verify_loop[last_idx] / n;

		/* 判据：残余的【绝对值】缩到原来的 1/4 以内，或者已经很小（<0.15°） */
		{
			float aBase = (base >= 0.0f) ? base : -base;
			float aVer  = (g_car_spin_verify_per >= 0.0f)
			              ? g_car_spin_verify_per : -g_car_spin_verify_per;

			if (aBase > 0.05f)
			{
				g_car_spin_verified = ((aVer <= (aBase * 0.25f)) || (aVer <= 0.15f))
				                      ? 1U : 0U;
			}
			else
			{
				g_car_spin_verified = 1U;         /* 本来就没误差，算通过 */
			}
		}
	}
#endif /* CAR_SPIN_ENABLE */

	/* ---------- 收尾：停车待机 ---------- */
	g_car_bu_seq_stage = 3U;
	Car_BuStop();
	g_car_spin_phase = 4U;                        /* 4 = 全部完成（含复测）*/
	g_car_tst_done   = 1U;

	for (;;)
	{
		osDelay(1000U);
	}
}

/* ==========================================================================
 *                      ★★★ 七、直行测试（纯诊断）★★★
 * ==========================================================================
 *
 *  两个用途：
 *    ① 距离标定（要拿卷尺）：命令走 D，量出实际 A，
 *         新系数 = 旧系数 × D / A
 *    ② 直行跑偏（自动，不用卷尺）：走完读陀螺仪得 Δyaw，
 *         每米跑偏 = Δyaw / D（°/m）
 *
 *  ★ 前进/后退各跑一遍，用「一加一减」分离两种误差：
 *       前进偏 +δ、后退偏 −δ  -> 真实轮差（可修）
 *       前进偏 +δ、后退也 +δ  -> 陀螺仪零偏漂移（改轮距没用）
 *
 *  ★ 每条腿都必须从静止起、走完停稳再读：
 *    还在滑的时候读陀螺仪，读到的是"正在转"的中间值。
 */

/**
  * @brief  走一段直行并返回车头偏了多少度（正 = 左偏/逆时针）
  *
  *         只负责"走 + 读"，不做任何判断，方便前后两个方向复用。
  * @param  distance_mm  位移 mm（负 = 后退）
  * @retval 相对这段起点的车头偏角 °
  */
static float Bu_StraightOneLeg(float distance_mm)
{
	AngleData_t ad;
	float    h0;
	float    d = (distance_mm >= 0.0f) ? distance_mm : -distance_mm;

	/* 先停稳，等陀螺仪安静下来再取基准 */
	osDelay(CAR_TST_YAW_SETTLE_MS);
	Angle_GetData(&ad);
	h0 = ad.angle_deg;

	Bu_ReinitIfMotorRebooted();

	(void)Car_MoveStraightEx(distance_mm, CAR_STR_SPEED_MMPS, CAR_STR_ACCEL_0P1MMPS2);
	Bu_WaitMotionDone(Bu_MoveTimeout(d, CAR_STR_SPEED_MMPS));

	/* 减速段还在滑，读早了会把"正在滑"当成最终朝向 */
	osDelay(CAR_TST_YAW_SETTLE_MS * 2U);
	Angle_GetData(&ad);

	return Bu_Wrap180(ad.angle_deg - h0);
}

/**
  * @brief  ★ 直行测试：重复「前进 D -> 停 -> 后退 D -> 停」，量距离与跑偏
  *
  *         结果全部写进 g_car_str_*，含义见 car_bringup.h 的 CAR_STR_* 一段。
  * @param  None（永不返回）
  * @retval None
  */
void Car_BuRunStraight(void)
{
	uint32_t i;
	uint32_t n = CAR_STR_PAIRS;
	float    D = CAR_STR_DIST_MM;
	float    sum_fwd = 0.0f;
	float    sum_bwd = 0.0f;
	float    yaw_rad;

	g_car_str_phase     = 0U;
	g_car_str_run       = 0U;
	g_car_str_dist_cmd  = D;
	g_car_str_scale_cmd = (float)CAR_BU_DIST_SCALE_PCT;
	g_car_str_yaw_fwd   = 0.0f;
	g_car_str_yaw_bwd   = 0.0f;
	g_car_str_yaw_per_m = 0.0f;
	g_car_str_bias      = 0.0f;
	g_car_str_lat_fwd_mm = 0.0f;
	g_car_str_lat_bwd_mm = 0.0f;
	g_car_str_yaw_drift = 0.0f;

	for (i = 0U; i < CAR_STR_MAX_RUNS; i++)
	{
		g_car_str_fwd_run[i] = 0.0f;
		g_car_str_bwd_run[i] = 0.0f;
	}

#if (CAR_STR_ENABLE == 0)
	g_car_str_phase = 3U;
	for (;;) { osDelay(1000U); }
#else
	if (n == 0U)               { n = 1U; }
	if (n > CAR_STR_MAX_RUNS)  { n = CAR_STR_MAX_RUNS; }
	if (D < 1.0f)              { D = 1.0f; }

	g_car_bu_seq_stage = 0U;
	osDelay(CAR_BU_SEQ_SETTLE_MS);

	for (i = 0U; i < n; i++)
	{
		g_car_str_run = i;

		/* ---------------- ① 前进 ---------------- */
		g_car_bu_seq_stage = 1U;
		g_car_str_phase    = 1U;

		g_car_str_fwd_run[i] = Bu_StraightOneLeg(D);
		sum_fwd += g_car_str_fwd_run[i];
		osDelay(CAR_STR_GAP_MS);

		/* ---------------- ② 后退（回原点）---------------- */
		/* ★ CAR_STR_RETURN=0 时不后退：每条腿都朝前走，
		 *   场地要 n×D 长，但前进/后退的对照就没了，只能看单一方向。 */
		g_car_bu_seq_stage = 2U;
		g_car_str_phase    = 2U;

#if (CAR_STR_RETURN != 0)
		g_car_str_bwd_run[i] = Bu_StraightOneLeg(-D);
		sum_bwd += g_car_str_bwd_run[i];
#else
		g_car_str_bwd_run[i] = 0.0f;
#endif
		osDelay(CAR_STR_GAP_MS);
	}

	/* ---------------- ③ 汇总 ---------------- */
	g_car_str_yaw_fwd = sum_fwd / (float)n;
	g_car_str_yaw_bwd = sum_bwd / (float)n;

	/* 一加一减分离两种误差（★ 这里不能给后退取反，看推导）：
	 *
	 *   设 g = 真实轮差让车头偏的角度（前进时为正 = 逆时针）
	 *      d = 陀螺仪零偏在一条腿时间里累积的角度（和往哪走无关）
	 *
	 *   前进测得： yaw_fwd = +g + d      （前进时轮差让它逆时针偏）
	 *   后退测得： yaw_bwd = -g + d      （后退时轮差让它顺时针偏）
	 *
	 *   所以： g = (yaw_fwd - yaw_bwd) / 2      <- 真实轮差，可修
	 *          d = (yaw_fwd + yaw_bwd) / 2      <- 零偏漂移，改轮距没用
	 *
	 *   ★ 常见错误是先把后退取反再算，那样 g 和 d 会互换。 */
#if (CAR_STR_RETURN != 0)
	{
		float diff   = (g_car_str_yaw_fwd - g_car_str_yaw_bwd) * 0.5f;
		float common = (g_car_str_yaw_fwd + g_car_str_yaw_bwd) * 0.5f;

		yaw_rad = diff * CAR_BU_PI / 180.0f;

		/* 每米跑偏：只用"真实轮差"那部分，零偏不该算进车的性能里 */
		g_car_str_yaw_per_m = diff / (D / 1000.0f);

		/* 等效轮差：两轮路程差 Δ = Δyaw(rad) × 轮距，相对比例 bias = Δ / D */
		g_car_str_bias = (yaw_rad * (float)CAR_WHEEL_BASE_MM) / D;

		/* 零偏贡献（° / m） */
		g_car_str_yaw_drift = common / (D / 1000.0f);
	}
	g_car_str_lat_fwd_mm = D * (g_car_str_yaw_fwd * CAR_BU_PI / 180.0f) * 0.5f;
	g_car_str_lat_bwd_mm = D * (g_car_str_yaw_bwd * CAR_BU_PI / 180.0f) * 0.5f;
#else
	g_car_str_yaw_per_m = g_car_str_yaw_fwd / (D / 1000.0f);
	yaw_rad = g_car_str_yaw_fwd * CAR_BU_PI / 180.0f;
	g_car_str_bias = (yaw_rad * (float)CAR_WHEEL_BASE_MM) / D;
	g_car_str_lat_fwd_mm = D * yaw_rad * 0.5f;
	g_car_str_lat_bwd_mm = 0.0f;
#endif
#endif /* CAR_STR_ENABLE */

	/* ---------- 收尾：停车待机 ---------- */
	g_car_bu_seq_stage = 3U;
	Car_BuStop();
	g_car_str_phase = 3U;
	g_car_tst_done  = 1U;

	for (;;)
	{
		osDelay(1000U);
	}
}

/* ==========================================================================
 *      ★★★ 八、转角误差分辨测试（判「固定偏移」还是「增益误差」）★★★
 * ==========================================================================
 *
 *  核心逻辑只有一句：
 *      在【单一角度】下，"固定偏移"和"增益误差"长得一模一样，
 *      必须用【两个不同角度】才能分开。
 *
 *      增益误差：误差 ∝ 角度   ->  e(90)/e(360) = 0.25
 *      固定偏移：误差与角度无关 ->  e(90)/e(360) = 1.00
 *
 *  实现要点：
 *    ① 测量前把增益复位 1.0、固定弧长补偿清零
 *       否则算出来的"建议值"会是增量而不是绝对值。
 *    ② 每次都用【绝对航向增量】算实际转角（读前一次、读后一次相减），
 *       误差不会带着上一轮的偏差走。
 *    ③ 陀螺仪读数会 ±180 环绕，180° 那一点尤其容易混叠。
 *       所以把读到的增量"对齐"到离指令角度最近的 360° 圈上。
 */

/**
  * @brief  把陀螺仪读到的角度增量对齐到指令角度附近的同一圈
  *
  *         例：指令 180°，陀螺仪因环绕报 -179.5°（其实是 180.5°）
  *             对齐后返回 +180.5°
  * @param  delta_raw   wrap 后的读数差 °
  * @param  angle_cmd   本次指令角度 °
  * @retval 对齐后的实际转角 °
  */
static float Bu_AlignDelta(float delta_raw, float angle_cmd)
{
	float d = delta_raw;

	while ((d - angle_cmd) > 180.0f)  { d -= 360.0f; }
	while ((d - angle_cmd) < -180.0f) { d += 360.0f; }

	return d;
}

/**
  * @brief  原地转一次 angle_deg（开环），返回实际转过的角度 °
  *
  *         ★ 用增量(前后各读一次相减)，不是绝对航向 —— 这样每一轮的
  *           残余误差不会带到下一轮里去。
  * @param  angle_deg  指令角度 °（正 = 左转）
  * @retval 实际转角 °（可能超过 360，已做环绕对齐）
  */
static float Bu_TurnRawOnce(float angle_deg)
{
	AngleData_t ad;
	float    h0;
	float    h1;
	float    d;

	/* 停稳再读基准：还在滑的时候读，读到的是"正在转"的中间值 */
	osDelay(CAR_TST_YAW_SETTLE_MS);
	Angle_GetData(&ad);
	h0 = ad.angle_deg;

	Bu_ReinitIfMotorRebooted();

	/* 开环：只发弧长，不看陀螺仪 */
	(void)Car_RotateInPlaceEx(angle_deg, CAR_ANGCHK_DPS, CAR_TST_TURN_ACCEL_0P1MMPS2);
	Bu_WaitMotionDone(Bu_TurnTimeout(angle_deg, CAR_ANGCHK_DPS));

	osDelay(CAR_TST_YAW_SETTLE_MS * 2U);
	Angle_GetData(&ad);
	h1 = ad.angle_deg;

	d = Bu_Wrap180(h1 - h0);
	return Bu_AlignDelta(d, angle_deg);
}

/**
  * @brief  ★ 转角误差分辨测试：分别转 90/180/360，判是哪一类误差
  *
  *         结果全部写进 g_car_angchk_*，含义见 car_bringup.h 的 CAR_ANGCHK_* 一段。
  * @param  None（永不返回）
  * @retval None
  */
void Car_BuRunAngleCheck(void)
{
	uint8_t  s;
	uint8_t  r;
	float    ang;
	float    actual;
	float    err;
	float    e_small;
	float    e_big;
	float    ratio;

	static const float deg_tab[CAR_ANGCHK_SETS] =
	{
		CAR_ANGCHK_DEG0,
		CAR_ANGCHK_DEG1,
		CAR_ANGCHK_DEG2
	};

	g_car_angchk_phase    = 0U;
	g_car_angchk_set      = 0U;
	g_car_angchk_run      = 0U;
	g_car_angchk_class    = 0U;
	g_car_angchk_ratio    = 0.0f;
	g_car_angchk_arc_sug  = 0.0f;
	g_car_angchk_gain_sug = 100.0f;

	for (s = 0U; s < CAR_ANGCHK_SETS; s++)
	{
		g_car_angchk_deg[s] = deg_tab[s];
		g_car_angchk_err[s] = 0.0f;
		for (r = 0U; r < CAR_ANGCHK_REPS; r++)
		{
			g_car_angchk_run_err[s][r] = 0.0f;
		}
	}

#if (CAR_ANGCHK_ENABLE == 0)
	g_car_angchk_phase = 3U;
	g_car_tst_done     = 1U;
	for (;;) { osDelay(1000U); }
#else
	g_car_bu_seq_stage = 0U;
	osDelay(CAR_BU_SEQ_SETTLE_MS);

	/* ★ 测原始误差：增益复位 1.0、固定补偿清零。
	 *   这样算出来的建议值是绝对值，能直接填进 car_bringup.h。 */
	g_car_turn_gain    = 1.0f;
	g_car_turn_arc_off = 0.0f;

	g_car_angchk_phase = 1U;

	for (s = 0U; s < CAR_ANGCHK_SETS; s++)
	{
		g_car_angchk_set = s;
		ang = deg_tab[s];

		for (r = 0U; r < CAR_ANGCHK_REPS; r++)
		{
			g_car_angchk_run = r;
			Bu_ReinitIfMotorRebooted();

			actual = Bu_TurnRawOnce(ang);
			err    = actual - ang;

			g_car_angchk_run_err[s][r] = err;
			g_car_angchk_err[s] += err;

			osDelay(CAR_ANGCHK_GAP_MS);
		}

		if (CAR_ANGCHK_REPS > 0U)
		{
			g_car_angchk_err[s] /= (float)CAR_ANGCHK_REPS;
		}
	}

	/* ---------------- 判定 ---------------- */
	e_small = g_car_angchk_err[0];                    /* 最小角（90°）*/
	e_big   = g_car_angchk_err[CAR_ANGCHK_SETS - 1U]; /* 最大角（360°）*/

	if ((e_big > 0.5f) || (e_big < -0.5f))
	{
		ratio = (e_small / e_big);
		if (ratio < 0.0f) { ratio = -ratio; }
		g_car_angchk_ratio = ratio;

		if (ratio < 0.5f)
		{
			g_car_angchk_class = 2U;                  /* 增益误差 */
		}
		else
		{
			g_car_angchk_class = 1U;                  /* 固定偏移 */
		}
	}
	else
	{
		/* 误差小到测不出来：既不算增益也不算固定，按"都不用改"处理 */
		g_car_angchk_class = 0U;
		g_car_angchk_ratio = 0.0f;
	}

	/* ---------------- 给建议值 ----------------
	 * 增益：实际/指令 = 1 + g，g = e_big / 360（用最大角，信噪比最好）
	 *       补偿的是"每度少转/多转多少"，所以按比例修。 */
	if ((1.0f + (e_big / 360.0f)) > 0.1f)
	{
		g_car_angchk_gain_sug = 100.0f / (1.0f + (e_big / 360.0f));
	}

	/* 固定弧长：误差(°) × 每度弧长(mm/°) = 要补的弧长
	 * 符号：实际转角 - 指令 = e。e < 0（转不够）就多补，所以建议值 = -e × K。 */
	g_car_angchk_arc_sug = -(e_big * CAR_BU_ARC_K);

	/* ---------------- 验证：把推荐的补偿真的加上，再量一次 360° ----------------
	 * ★ 这一步很重要：前面的 1.39° 是在【增益 = 1.0】下测的原始误差，
	 *   光算出 99.61% 不等于它一定有效。加一次实测闭环验证，
	 *   才能在"改代码烧录之前"就确认这个数是对的。
	 *
	 *   按分类挑补偿方式：
	 *     class=1 固定偏移 -> 加固定弧长
	 *     class=2 增益误差 -> 乘增益
	 *   class=0 就两种都不加，量出来的应该还是原来那个小误差。 */
	if ((g_car_angchk_class == 2U) || (g_car_angchk_class == 0U))
	{
		g_car_turn_gain    = g_car_angchk_gain_sug / 100.0f;
		g_car_turn_arc_off = 0.0f;
	}
	else
	{
		g_car_turn_gain    = 1.0f;
		g_car_turn_arc_off = g_car_angchk_arc_sug;
	}

	g_car_angchk_verified = 0U;

	{
		float verify = Bu_TurnRawOnce(CAR_ANGCHK_DEG2);
		g_car_angchk_verify_err = verify - CAR_ANGCHK_DEG2;

		/* 补偿后误差应该大幅缩小。用"缩小到原来的 1/4 以内"作判据，
		 * 再松就没意义了（原始误差本来就不大）。 */
		if (((g_car_angchk_verify_err > -(e_big * 0.25f)) &&
		     (g_car_angchk_verify_err <  (e_big * 0.25f))) ||
		    ((g_car_angchk_verify_err < 0.3f) && (g_car_angchk_verify_err > -0.3f)))
		{
			g_car_angchk_verified = 1U;
		}
	}

#endif /* CAR_ANGCHK_ENABLE */

	/* ---------- 收尾：停车待机 ---------- */
	g_car_bu_seq_stage = 3U;
	Car_BuStop();
	g_car_angchk_phase = 3U;
	g_car_tst_done     = 1U;

	for (;;)
	{
		osDelay(1000U);
	}
}

/* ==========================================================================
 *              ★★★ 自动识别方向与左右轮（推车法，不写死 SN）★★★
 * ==========================================================================
 *
 *  为什么需要：
 *    原来左右轮靠 car_cfg.h 里写死的两行 SN 区分，换车就得改代码。
 *    真实场景里不可能每台车都写死。
 *
 *  能做到什么（这一点必须说清楚）：
 *    · 能确定：每台电机"正方向"对应车前进的哪个符号   ← ①推车
 *    · 能确定：哪台电机装在车的左边                    ← ②左转
 *    · 不能确定：以上之外的东西（也不需要）
 *
 *  为什么必须由人手推：
 *    让小车自己动就需要先知道方向 —— 鸡生蛋。所以这两步只能人手做。
 *
 *  信号选择：
 *    主用「剩余位移」—— 它直接就是"轮子被拖动了多少"，
 *    电机使能且目标=当前位置时，被推动后 remaining = 目标 - 当前，
 *    符号直接反映拖动方向，物理意义最干净。
 *    同时记录「力矩」积分做交叉验证（力矩方向应为位移方向的反向）。
 *
 *  时序：每台电机各采一次，两台同时采（推车时两轮一起转）。
 */

/** @brief 读某台电机的剩余位移（0.1mm），设备号 0/1 */
static int32_t Bu_GetRemain(uint8_t dev)
{
	return (dev == 0U) ? g_motor_remain_left : g_motor_remain_right;
}

/** @brief 读某台电机的力矩 */
static int16_t Bu_GetTorque(uint8_t dev)
{
	return Motor_GetTorque(dev);
}

/** @brief 一个采样窗口：把剩余位移/力矩对时间积分，判断"有没有被推动" */
static uint8_t Bu_CaptureWindow(uint32_t wait_ms, int32_t *remain_out, int32_t *torque_out)
{
	uint32_t waited = 0U;
	uint32_t elapsed;
	int32_t  sum_r[2] = { 0, 0 };
	int32_t  sum_t[2] = { 0, 0 };
	uint8_t  moving   = 0U;
	uint8_t  dev;

	/* --- 第一段：等"开始动"（最多等 wait_ms）--- */
	while (waited < wait_ms)
	{
		uint8_t active = 0U;

		for (dev = 0U; dev < 2U; dev++)
		{
			int32_t r = Bu_GetRemain(dev);
			int16_t t = Bu_GetTorque(dev);

			if ((r >= CAR_BU_DIRDET_REMAIN_MIN) || (r <= -CAR_BU_DIRDET_REMAIN_MIN) ||
			    (t >= CAR_BU_DIRDET_TORQUE_MIN)  || (t <= -CAR_BU_DIRDET_TORQUE_MIN))
			{
				active = 1U;
			}
		}

		if (active != 0U)
		{
			moving = 1U;
			break;
		}
		osDelay(CAR_BU_DIRDET_PERIOD_MS);
		waited += CAR_BU_DIRDET_PERIOD_MS;
	}

	/* --- 第二段：连续采样一个固定窗口，做积分 --- */
	for (elapsed = 0U; elapsed < CAR_BU_DIRDET_SAMPLE_MS; elapsed += CAR_BU_DIRDET_PERIOD_MS)
	{
		for (dev = 0U; dev < 2U; dev++)
		{
			sum_r[dev] += Bu_GetRemain(dev);
			sum_t[dev] += (int32_t)Bu_GetTorque(dev);
		}
		osDelay(CAR_BU_DIRDET_PERIOD_MS);
	}

	/* --- 第三段：等人松手，让轮子回到原位 --- */
	osDelay(CAR_BU_DIRDET_SETTLE_MS);

	for (dev = 0U; dev < 2U; dev++)
	{
		remain_out[dev] = sum_r[dev];
		torque_out[dev] = sum_t[dev];
	}
	return moving;
}

/**
  * @brief  ★ 自动识别每台电机的方向 + 哪台是左轮（人手推车两步）
  *
  *         前提： Car_Motor_Init() 已经跑完（电机已使能、目标=当前位置）。
  *         结果： 写入 g_car_dir_sign[] / g_car_dir_left_dev，运动函数会自动使用。
  *
  *         进度看 g_car_dirdet_state：
  *             1 = 请把车向前推
  *             2 = 正在采样推车
  *             3 = 请把车原地向左转 90°
  *             4 = 正在采样转向
  *             5 = 识别完成
  *             9 = 识别失败（原因见 g_car_dirdet_fail）
  *
  * @retval 1 成功，0 失败（失败时保留原有默认方向，不会把车搞坏）
  */
uint8_t Car_AutoDetectDirections(void)
{
	int32_t  push_r[2], push_t[2];
#if (CAR_BU_DIRDET_ROT_MODE != 2)
	int32_t  rot_r[2],  rot_t[2];
#endif
	int8_t   fwd[2];
	uint8_t  dev;
	int8_t   left_dev = -1;

	g_car_dirdet_ok   = 0U;
	g_car_dirdet_fail = 0U;

#if (CAR_BU_DIRDET_ENABLE == 0)
	return 0U;                                  /* 未启用识别 */
#endif

	/* ---------- ① 推车：确定每台电机的"前进方向" ---------- */
	g_car_dirdet_state = 1U;                    /* 亮"请推车"阶段 */
	osDelay(1000U);

	if (Bu_CaptureWindow(CAR_BU_DIRDET_WAIT_MS, push_r, push_t) == 0U)
	{
		g_car_dirdet_fail  = 1U;                /* 等了 60s 车没动 */
		g_car_dirdet_state = 9U;
		return 0U;
	}
	g_car_dirdet_state = 2U;

	for (dev = 0U; dev < 2U; dev++)
	{
		g_car_dirdet_push[dev]   = push_r[dev];
		g_car_dirdet_torque[dev] = push_t[dev];
	}

	/* 两台都必须有明显信号，否则判失败（可能推得不直/只推动了一个轮子）*/
	for (dev = 0U; dev < 2U; dev++)
	{
		if ((push_r[dev] > -CAR_BU_DIRDET_REMAIN_MIN) && (push_r[dev] < CAR_BU_DIRDET_REMAIN_MIN))
		{
			g_car_dirdet_fail  = 2U;            /* 位移信号太小 */
			g_car_dirdet_state = 9U;
			return 0U;
		}
	}

	/* ★ 两台镜像安装，向前推时剩余的符号必然相反。相同说明装法不对或只推动了一侧 */
	if (((push_r[0] > 0) && (push_r[1] > 0)) || ((push_r[0] < 0) && (push_r[1] < 0)))
	{
		g_car_dirdet_fail  = 3U;                /* 两轮同号，不像镜像安装 */
		g_car_dirdet_state = 9U;
		return 0U;
	}

	/* remain = 目标 - 当前。被推着"前进"时当前值变大 -> remain 变负。
	 * 所以"前进 = 正方向"的电机，其 remain 积分为负 -> dir_sign = +1 */
	for (dev = 0U; dev < 2U; dev++)
	{
		fwd[dev] = (push_r[dev] < 0) ? (int8_t)1 : (int8_t)-1;
#if (CAR_BU_DIRDET_INVERT != 0)
		fwd[dev] = (int8_t)(-fwd[dev]);
#endif
	}

	/* ---------- ② 定出哪台装在左边 ---------- */
#if (CAR_BU_DIRDET_ROT_MODE == 2)
	/* ★ 让车自己扭一下，用陀螺仪读转角符号 —— 不用人手，也不依赖电机型号约定
	 *
	 *   给两台电机发【同号的原始位移】（同一个 raw 值，不套方向表）：
	 *     镜像安装 -> 两轮转向相反 -> 车原地转
	 *     同向安装 -> 两轮转向相同 -> 车直行，陀螺仪读不到角度（判失败并回退）
	 *
	 *   车往左转（俯视逆时针，Δyaw > 0）：左轮往后、右轮往前
	 *       发的是同一个 raw，所以"往后"的那台 raw 正方向就是车的后方
	 *         -> 该台 fwd = -1，另一台 fwd = +1，且往后那台 = 左轮
	 *   车往右转（俯视顺时针，Δyaw < 0）：反过来
	 *
	 *   全部只用到「左转时左轮往后」这一个纯几何事实。 */
	{
		AngleData_t ad;
		float  dyaw;
		int8_t raw_fwd0;

		g_car_dirdet_state = 4U;

		Angle_RequestZero();
		osDelay(CAR_BU_DIRDET_ROT_GYRO_MS);

		/* 同号原始位移：两轮转向相反 -> 原地转 */
		(void)Bu_EmitMotion((int32_t)CAR_BU_DIRDET_ROT_ARC,
		                    (int32_t)CAR_BU_DIRDET_ROT_ARC,
		                    (uint32_t)CAR_BU_DIRDET_ROT_MMPS, 0U);
		Bu_WaitMotionDone(CAR_BU_DIRDET_ROT_DONE_MS);
		osDelay(CAR_BU_DIRDET_SETTLE_MS);

		Angle_GetData(&ad);
		dyaw = ad.angle_deg;

		/* 转回原位，不停在歪的地方（失败也照做，尽量别让车歪着） */
		(void)Bu_EmitMotion(-(int32_t)CAR_BU_DIRDET_ROT_ARC,
		                    -(int32_t)CAR_BU_DIRDET_ROT_ARC,
		                    (uint32_t)CAR_BU_DIRDET_ROT_MMPS, 0U);
		Bu_WaitMotionDone(CAR_BU_DIRDET_ROT_BACK_MS);
		osDelay(CAR_BU_DIRDET_SETTLE_MS);

		if ((dyaw > CAR_BU_DIRDET_ROT_DEG) || (dyaw < -CAR_BU_DIRDET_ROT_DEG))
		{
			/* 左转(Δyaw>0)：左轮往后 -> raw+ 是往后的那台 = 左轮
			 * 右转(Δyaw<0)：左轮往前 -> raw+ 是往前的那台  = 右轮 */
			if (dyaw > 0.0f)
			{
				raw_fwd0 = (int8_t)-1;          /* raw+ 让设备0的轮子往后 */
				left_dev = 0;                   /* 设备0 = 左轮 */
			}
			else
			{
				raw_fwd0 = (int8_t)1;           /* raw+ 让设备0的轮子往前 */
				left_dev = 1;                   /* 设备0 = 右轮 -> 左轮是设备1 */
			}

			/* 陀螺仪自转的结果比人手推车的积分可靠得多，直接采信；
			 * 推力方向（fwd）一并由它定死，只做一致性提示。 */
			fwd[0] = raw_fwd0;
			fwd[1] = (int8_t)(-raw_fwd0);

			/* 若和推车得到的结果冲突，以陀螺仪为准，但记一次失败码便于排查 */
			if ((push_r[0] < 0) && (fwd[0] != 1))  { g_car_dirdet_fail = 4U; }
			if ((push_r[0] > 0) && (fwd[0] != -1)) { g_car_dirdet_fail = 4U; }
		}
		else
		{
			/* 陀螺仪没读到转角：两轮可能同向安装（不镜像），
			 * 或者电机没真正执行。保留推车结果，左右退回默认（设备0 = 左）。 */
			left_dev = 0;
			g_car_dirdet_fail = 5U;
		}
	}

#else
	/* 方式 1：人手把车原地向左转 90°，靠剩余位移积分判定 */
	g_car_dirdet_state = 3U;
	osDelay(1000U);

	if (Bu_CaptureWindow(CAR_BU_DIRDET_WAIT_MS, rot_r, rot_t) == 0U)
	{
		/* 第二步失败不致命：方向已经测好了，左右只是命名问题。
		 * 默认把设备 0 当左轮（和原来写死的行为一致）。 */
		left_dev = 0;
	}
	else
	{
		g_car_dirdet_state = 4U;
		for (dev = 0U; dev < 2U; dev++)
		{
			g_car_dirdet_rot[dev] = rot_r[dev];
		}

		/* 车向左转（俯视逆时针）时：左轮往后滚、右轮往前滚。
		 * 所以"转向时的符号 == 前进时的符号"的那台是右轮，
		 *     相反的则是左轮。 */
		for (dev = 0U; dev < 2U; dev++)
		{
			int8_t rot_fwd = (rot_r[dev] < 0) ? (int8_t)1 : (int8_t)-1;
#if (CAR_BU_DIRDET_INVERT != 0)
			rot_fwd = (int8_t)(-rot_fwd);
#endif
			if (rot_fwd != fwd[dev])            /* 转向时方向与前进相反 -> 左轮 */
			{
				left_dev = (int8_t)dev;
			}
		}
		if (left_dev < 0)
		{
			left_dev = 0;                       /* 判不出来就退回默认 */
		}
	}
#endif

	/* ---------- 写入运行时表 ---------- */
	g_car_dir_left_dev = left_dev;
	for (dev = 0U; dev < 2U; dev++)
	{
		g_car_dir_sign[dev] = fwd[dev];
	}

	g_car_dirdet_ok    = 1U;
	g_car_dirdet_state = 5U;
	return 1U;
}


/**
  * @brief  只跑轮径诊断（不含初始化、不含后续动作），返回后不会退出
  *         2s 等待 -> 两轮各走 1 整圈 -> 待机
  * @param  None
  * @retval None
  */
void Car_BuDiagOnly(void)
{
	g_car_bu_seq_stage = 0U;
	osDelay(CAR_BU_SEQ_SETTLE_MS);

	g_car_bu_seq_stage = 1U;
	Car_BuDiagWheelOneRev();

	g_car_bu_seq_stage = 5U;
	for (;;)
	{
		osDelay(1000U);
	}
}

/**
  * @brief  等电机控制器启动完成：轮询 0x0312 上电广播
  *
  *         电机控制器复位时会主动广播 0x0312（7 字节 SN），这是它"我起来了"的信号。
  *         固定 osDelay(500) 在 STM32 与电机同时上电时经常踩空 —— 帧发出去了，
  *         但电机还在启动，于是绑定全部落空，后面所有帧无人应答（上电完全不动）。
  *
  *         策略：
  *           ① 轮询等第一条广播（最多 CAR_BU_MOTOR_BOOT_WAIT_MS）
  *           ② ★ 再多等剩余时间，直到收到【两条不同】SN 或总超时
  *              —— 两台电机复位时刻不一定同步，只等第一条会导致第二台
  *                 拿到 car_cfg.h 的兜底 SN，绑不上（换车就完全不动的根源）
  *           ③ 一直凑不齐就超时返回，绝不死等（心跳帧能证明 STM32 是活的）
  * @param  None
  * @retval None
  */
static void Bu_WaitMotorBoot(void)
{
	uint32_t waited = 0U;

	g_car_bu_boot_sn_count = 0U;

	while (waited < CAR_BU_MOTOR_BOOT_WAIT_MS)
	{
		if (g_motor_rx_sn_count > 0U)
		{
			break;
		}
		osDelay(50U);
		waited += 50U;
	}

	if (g_motor_rx_sn_count > 0U)
	{
		/* 第一台已经起来了：给第二台留时间广播 */
		osDelay(CAR_BU_MOTOR_BOOT_SETTLE_MS);

		/* ★ 继续等第二条不同的 SN（最多再花 SETTLE_MS × 4）*/
		{
			uint32_t extra = 0U;
			while ((Motor_GetBootSnUsed() < 2U) &&
			       (extra < (CAR_BU_MOTOR_BOOT_SETTLE_MS * 4U)))
			{
				osDelay(50U);
				extra += 50U;
			}
		}
	}

	g_car_bu_boot_sn_count = g_motor_rx_sn_count;
}

/**
  * @brief  发一帧 STM32 心跳（0x7FE），每 500ms 一次
  *         ID 0x7FE 在电机协议里没有占用，不会和任何电机帧冲突
  *
  *         Data[1] 的 bit0~4 轮流携带 SN 的 7 个字节，方便在分析仪上核对
  *         「固件里烧的 SN」到底和电机广播的 SN 一不一致（SN 对不上是
  *         「上位机能开、STM32 不动」的头号原因）
  * @param  None
  * @retval None
  */
void Car_BuSendHeartbeat(void)
{
	static uint8_t seq = 0U;
	uint8_t d[8];

	d[0] = 0xA5U;                                    /* 魔数，一眼认出是 STM32 */
	d[1] = seq++;                                    /* 序号在变 = 任务没被饿死 */
	d[2] = g_car_bu_seq_stage;
	/* Data[3]：低 4 位 = 初始化完成标志，高 4 位 = 上电时收到的 0x0312 广播条数
	 * 广播条数 = 0 说明上电时一台电机都没起来（没电 / CAN 断开 / 电机没上电） */
	d[3] = (uint8_t)((g_car_bu_inited & 0x0FU) |
	                 ((g_car_bu_boot_sn_count & 0x0FU) << 4));
	d[4] = (uint8_t)(g_car_bu_tx_fail  & 0xFFU);
	d[5] = (uint8_t)(g_car_bu_tx_count & 0xFFU);
	d[6] = (uint8_t)((g_car_bu_tx_fail  >> 8) & 0xFFU);
	d[7] = (uint8_t)((g_car_bu_tx_count >> 8) & 0xFFU);

	(void)Can_SendStd(CAR_BU_HEARTBEAT_ID, d, 8U);
}

/**
  * @brief  发一帧「固件打算绑的 SN」（0x7FD），左右轮交替，每 2 秒一次
  *
  *         ★ 用途：不接上位机、不开日志，只看 CAN 分析仪就能核对
  *           「STM32 打算绑哪个 SN」vs「电机上电广播 0x0312 的 SN」。
  *           两者不一致时，0x0313 会认领失败，电机永远不动 —— 这是最坑的一个坑。
  *         ★ 现在 SN 是运行时决定的（优先用上电广播），所以这一帧
  *           直接反映 Bu_ResolveSn() 的结果，也就是真正下发 0x0313 的内容。
  *
  *         Data[0..6] = SN（7 字节，与 0x0312 广播同格式，可直接逐字节对比）
  *         Data[7]    = 0 左轮 / 1 右轮
  * @param  dev  0 = 左轮，1 = 右轮
  * @retval None
  */
void Car_BuSendSnInfo(uint8_t dev)
{
	uint8_t d[8];

	Bu_ResolveSn(dev, d);
	d[7] = (dev != 0U) ? 1U : 0U;

	(void)Can_SendStd(CAR_BU_SN_INFO_ID, d, 8U);
}

/** @brief  把一个有小数的量转成 int16 定点，带饱和（防止溢出回绕成相反的符号） */
static int16_t Bu_ToI16(float v)
{
	float r = (v >= 0.0f) ? (v + 0.5f) : (v - 0.5f);

	if (r > 32767.0f)  { r = 32767.0f; }
	if (r < -32768.0f) { r = -32768.0f; }
	return (int16_t)r;
}

/** @brief  小端写 int16 */
static void Bu_PutI16(uint8_t *b, int16_t v)
{
	uint16_t u = (uint16_t)v;

	b[0] = (uint8_t)(u & 0xFFU);
	b[1] = (uint8_t)(u >> 8);
}

/**
  * @brief  ★ 把诊断结果广播到 CAN（两帧，格式见 car_bringup.h 的 CAR_DIAG_ANGLE_ID）
  *
  *         ★ 为什么要把这些值发出去：
  *           Watch 只能在本机调试器里看，口头转述容易出错。
  *           把它们编码成 CAN 帧，任何分析仪都能独立核对，
  *           也让"谁测的、测到多少"这件事有可追溯的记录。
  *
  *         定点格式（全部小端 int16，避免浮点在 CAN 上无法直接表达）：
  *             角度     0.01°    （1.40° -> 140）
  *             弧长     0.01mm   （-4.74mm -> -474）
  *             百分比   0.01%    （99.61% -> 9961）
  * @param  None
  * @retval None
  */
void Car_BuSendDiag(void)
{
	uint8_t d[8];

	/* ---- 帧 1：0x7FC 分类 + 90°/180° 的误差 ---- */
	d[0] = 0xA6U;                                  /* 魔数 */
	d[1] = g_car_angchk_phase;
	d[2] = g_car_angchk_class;
	d[3] = (uint8_t)((g_car_angchk_ratio > 2.55f) ? 255 :
	                 ((g_car_angchk_ratio < 0.0f)  ? 0   :
	                  (uint8_t)(g_car_angchk_ratio * 100.0f + 0.5f)));
	Bu_PutI16(&d[4], Bu_ToI16(g_car_angchk_err[0] * 100.0f));
	Bu_PutI16(&d[6], Bu_ToI16(g_car_angchk_err[1] * 100.0f));
	(void)Can_SendStd(CAR_DIAG_ANGLE_ID, d, 8U);

	/* ---- 帧 2：0x7FB 360° 的误差 + 两个建议值 ---- */
	d[0] = 0xA6U;
	d[1] = 0x01U;                                  /* 续帧标记 */
	Bu_PutI16(&d[2], Bu_ToI16(g_car_angchk_err[2] * 100.0f));
	Bu_PutI16(&d[4], Bu_ToI16(g_car_angchk_arc_sug * 100.0f));
	Bu_PutI16(&d[6], Bu_ToI16(g_car_angchk_gain_sug * 100.0f));
	(void)Can_SendStd(CAR_DIAG_ANGLE_ID2, d, 8U);

	/* ---- 帧 3：0x7FA 验证结果（把建议补偿真的加上后，360° 的残余）---- */
	d[0] = 0xA6U;
	d[1] = 0x02U;                                  /* 验证帧标记 */
	d[2] = g_car_angchk_verified;
	d[3] = 0x00U;
	Bu_PutI16(&d[4], Bu_ToI16(g_car_angchk_verify_err * 100.0f));
	Bu_PutI16(&d[6], Bu_ToI16(g_car_turn_gain * 10000.0f));
	(void)Can_SendStd(CAR_DIAG_ANGLE_ID3, d, 8U);

	/* ---- 帧 4：0x7F9 SPIN 结果（补偿前后的每圈偏差对比）----
	 *   ★ 这一帧专门解释"为什么 SPIN 跑完还是显示 1.4°"：
	 *     fwd_per 是【增益=1.0】下测的原始误差（应该 ≈1.4°，正常）；
	 *     verify_per 是【加上补偿后】复测的残余（应该 ≈0）。
	 *     两者一起看才知道补偿到底有没有用。 */
	d[0] = 0xA6U;
	d[1] = 0x03U;                                  /* SPIN 帧标记 */
	d[2] = g_car_spin_phase;                       /* 0/1/2/3=复测中/4=完成 */
	d[3] = g_car_spin_verified;
	/* ★ 先把两个 int16 都算好再写，不要写 d[4]/d[5] 再覆盖（容易看错） */
	Bu_PutI16(&d[4], Bu_ToI16(g_car_spin_fwd_per * 100.0f));     /* 补偿前每圈偏差 */
	Bu_PutI16(&d[6], Bu_ToI16(g_car_spin_verify_per * 100.0f));  /* 补偿后每圈残余 */
	(void)Can_SendStd(CAR_DIAG_SPIN_ID, d, 8U);

	/* ---- 帧 4：0x7F3 转角停止点系统偏差（治「每轮固定偏一点、越偏越大」）----
	 *   ★ 这是"往右偏"的直接指标：
	 *       bias 稳定在正数  -> 车每次都"转不够"
	 *       bias 稳定在负数  -> 每次都"转过头"
	 *       bias 收敛到 0    -> 固定残余被补掉了，只剩随机项
	 *   判读：跑完一轮后看 bias 是否稳定；若还在单调增长说明偏差大于限幅。
	 */
	d[0] = 0xA6U;
	d[1] = 0x09U;                                     /* 停止点帧标记 */
	d[2] = (uint8_t)((g_car_turn_stop_n > 255U) ? 255U : g_car_turn_stop_n);
	d[3] = (uint8_t)((g_car_turn_corr_cnt > 255U) ? 255U : g_car_turn_corr_cnt);
	Bu_PutI16(&d[4], Bu_ToI16(g_car_turn_stop_bias * 1000.0f));               /* °，0.001 */
	Bu_PutI16(&d[6], Bu_ToI16(g_car_turn_stop_bias * CAR_BU_ARC_K * 100.0f)); /* mm，0.01 */
	(void)Can_SendStd(CAR_DIAG_STOP_ID, d, 8U);

	/* ---- 帧 5：0x7F2 航向补偿状态（连续直行 + 在线学习）----
	 *   ★ 这是唯一能确认"补偿方向对不对"的窗口：
	 *       trim  -> 当前生效的两轮目标位移差（mm，0.01）
	 *       yaw   -> 走完后的航向残余（°）
	 *   判读：
	 *       trim 稳定在某个值且 yaw 不再下降  -> 已收敛（该值就是打滑补偿量）
	 *       trim 单调增长直到 ±CAR_HC_MAX_MM  -> ★ 学习方向反了，
	 *                                            把 CAR_HC_LEARN_K 改负号
	 *       trim 在符号间来回跳               -> K 太大，减半
	 *       trim=0 且 yaw=0                   -> 车本身就很直，无需补偿
	 */
	d[0] = 0xA6U;
	d[1] = 0x0AU;                                     /* 航向补偿帧标记 */
	d[2] = g_car_hdg_trim_n;                          /* 学了几次 */
	d[3] = (uint8_t)((g_car_bu_nofb_cnt > 255U) ? 255U : g_car_bu_nofb_cnt); /* 反馈缺失 */
	Bu_PutI16(&d[4], Bu_ToI16(g_car_hdg_alpha * 100.0f));    /* 段转角 °，0.01 */
	Bu_PutI16(&d[6], Bu_ToI16(g_car_hdg_trim_mm * 100.0f));  /* 位置差 mm，0.01 */
	(void)Can_SendStd(CAR_DIAG_HDG_ID, d, 8U);

	/* ---- 帧 6：0x7F1 测试状态（看"测试是不是被安全机制中止了"）----
	 *   ★ 这一帧解决"车不走了，不知道为什么"的问题：
	 *       abort=1  -> 因无法确认停车而【主动】终止整轮测试（安全保护生效）
	 *                  这时 nofb 一定 > 0，说明某台电机不报剩余位移
	 *       stage    -> 停在哪个阶段（1=直行 2=掉头 3=收尾）
	 *       waite    -> 最后一次等待实际等了多久 ms
	 *   判读：测试跑完 5 轮 = done 变 1；提前停住且 abort=1 = 反馈出问题。
	 */
	d[0] = 0xA6U;
	d[1] = 0x0BU;                                     /* 测试状态帧标记 */
	d[2] = g_car_bu_seq_stage;
	d[3] = (uint8_t)(((g_car_bu_abort != 0U)       ? 0x01U : 0x00U) |
	                 ((g_car_tst_done  != 0U)      ? 0x02U : 0x00U) |
	                 ((g_car_hdg_ref_mode != 0U)   ? 0x04U : 0x00U) |
	                 (((g_car_bu_sn_match & 0x03U) == 0x03U) ? 0x08U : 0x00U));
	/* ★ bit3 = 1 表示配置的左/右轮 SN 都在这台车的总线上找到了。
	 *   若为 0，说明 SN 抄错或电机没上电 —— 这时绑定会落空，
	 *   表现就是「车完全不动」。 */
	Bu_PutI16(&d[4], Bu_ToI16((float)g_car_bu_seq_waited_ms));            /* ms */
	Bu_PutI16(&d[6], Bu_ToI16((float)g_car_bu_turn_i));                   /* 开环转角：第几圈 */
	(void)Can_SendStd(CAR_DIAG_RUNST_ID, d, 8U);

	/* ---- 帧 7：0x7F0 方向符号 / 轮距 / 增益（关掉方向识别后的安全网）----
	 *   ★ 关掉 CAR_BU_DIRDET_ENABLE 后，方向就固定用 g_car_dir_sign 默认值。
	 *     万一默认值错了，车会【画圈】而不是走直线。这一帧让问题一眼可见：
	 *       dir_l / dir_r : 左右轮方向符号（+1 / -1，实际按 0x01/0xFF 编码）
	 *       W             : CAR_WHEEL_BASE_MM（×10，单位 0.1mm）
	 *       gain          : 转角增益（×10000）
	 *   判读：
	 *       dir_l=+1、dir_r=-1  -> 镜像安装，正常
	 *       dir_l=-1、dir_r=+1  -> 也正常（另一侧镜像）
	 *       两个都是 +1 或都是 -1 -> ★ 车会原地转圈，必须开启方向识别
	 */
	d[0] = 0xA6U;
	d[1] = 0x0CU;                                     /* 方向帧标记 */
	d[2] = (uint8_t)((g_car_dir_sign[0] > 0) ? 0x01U : 0xFFU);
	d[3] = (uint8_t)((g_car_dir_sign[1] > 0) ? 0x01U : 0xFFU);
	Bu_PutI16(&d[4], Bu_ToI16((float)CAR_WHEEL_BASE_MM * 10.0f));   /* 0.1mm */
	Bu_PutI16(&d[6], Bu_ToI16(g_car_turn_gain * 10000.0f));          /* 0.0001 */
	(void)Can_SendStd(CAR_DIAG_DIR_ID, d, 8U);
}

/**
  * @brief  等运动真正走完：轮询两个通道的剩余位移归零（带超时）
  *
  *         ★ 绝不能在轴还在动的时候下发反向指令 —— 实测会把电机搞死
  *           （位移/力矩/心跳/ACK 全 0，只能断电重启）。
  *         ★ 剩余位移来自 0x0420 反馈：motor.c 把主通道解析到
  *           g_motor_remain_left、从通道解析到 g_motor_remain_right。
  * @param  timeout_ms 最长等待时间 ms
  * @retval 1 两个通道都真的到位了（可以安全下发下一条 0x0416）
  *         0 超时且剩余量仍然很大（轴可能还在动 —— 【绝对不能】再发 0x0416，
  *           会搞死电机，只能断电重启）
  */
static uint8_t Bu_WaitMotionDone(uint32_t timeout_ms)
{
	uint32_t waited = 0U;
	uint8_t  seen   = 0U;
	uint8_t  ok     = 0U;

	/* ★ 第一段必须等「新的剩余量被报上来」。
	 *
	 *   0x0416 刚发出去的那一刻，0x0420 反馈里还是上一次的 0（已经走完了），
	 *   这时如果直接判"到位"，函数会在运动刚开始的几毫秒内就返回，
	 *   随后下一条指令（下一条边 / 下一次转向）会把目标改掉 ——
	 *   表现就是「这一段的位移或转角只走了一小部分」。
	 *
	 *   所以判据改成：必须先看到过"忙"，之后才认"闲"；
	 *   万一整个行程都在 EPS 以内（指令本来就很小），最多等
	 *   CAR_BU_REMAIN_SEEN_MS 就走下面正常流程。 */
	while (waited < timeout_ms)
	{
		int32_t l    = g_motor_remain_left;
		int32_t r    = g_motor_remain_right;
		uint8_t busy = ((Bu_AbsI32(l) > (int32_t)CAR_BU_REMAIN_EPS_0P1MM) ||
		                (Bu_AbsI32(r) > (int32_t)CAR_BU_REMAIN_EPS_0P1MM)) ? 1U : 0U;

		if (busy != 0U)
		{
			seen = 1U;
			s_motion_seen_busy = 1U;
		}
		else if (seen != 0U)
		{
			ok = 1U;
			break;
		}
		else if (waited >= CAR_BU_REMAIN_SEEN_MS)
		{
			/* ★★★ 这里原来是直接 break，那才是「中途改目标搞死电机」的真凶 ★★★
			 *
			 *   一次都没见过 busy，只可能有两种情况：
			 *     ① 指令本来就极小（< 3mm），车几乎没动 —— 确实可以走
			 *     ② 根本没收到 0x0420 反馈（电机 Byte6 bit5 的
			 *        「自动返回剩余位移」配置丢了 / 反馈帧被丢）
			 *   情况 ② 下 g_motor_remain_* 会【冻结为 0】，
			 *   于是 400ms 就返回"到位"，而车其实还在跑 ——
			 *   600mm/s 下 400ms 只走了 240mm，5m 还差 4760mm！
			 *   紧接着下一条 0x0416 就是「运动中途改目标」，
			 *   电机当场瘫痪，只能断电重启。
			 *
			 *   所以改成：必须等过「位移 ÷ 速度」这个物理下限才有可能走。 */
			if (waited >= s_expect_ms)
			{
				break;
			}
		}

		osDelay(20U);
		waited += 20U;
	}

	g_car_bu_seq_waited_ms = waited;
	osDelay(CAR_BU_MOTION_SETTLE_MS);

	/* 车已停稳，恢复零偏学习（学习窗口里只要出现过"关"，那个窗口整窗作废）*/
#if (CAR_BU_FREEZE_GYRO_BIAS != 0)
	Angle_SetBiasLearning(1U);
#endif

	/* ★ 停稳延时之后再确认一次：超时出来但轴其实已经停了 -> 仍算到位；
	 *   剩余量还很大 -> 轴可能还在动，必须让调用方知道。 */
	if ((Bu_AbsI32(g_motor_remain_left)  > (int32_t)CAR_BU_REMAIN_EPS_0P1MM) ||
	    (Bu_AbsI32(g_motor_remain_right) > (int32_t)CAR_BU_REMAIN_EPS_0P1MM))
	{
		ok = 0U;
	}

	/* ★ 整条指令期间一次没见过 busy，就是"没反馈"而不是"走完了"。
	 *   这种状态下【无法确认】轴已经停 —— 一律按未到位上报，
	 *   让调用方放弃后续 0x0416（宁可停一段，也不能搞死电机）。
	 *   注意用 s_motion_seen_busy（指令级）而不是 seen（本次等待级）：
	 *   同一条指令可能被等两次，第二次必然 seen==0。 */
	if ((s_motion_seen_busy == 0U) && (s_expect_ms >= CAR_BU_REMAIN_SEEN_MS))
	{
		ok = 0U;
		if (g_car_bu_nofb_cnt < 0xFFFFU)
		{
			g_car_bu_nofb_cnt++;          /* 计入 0x7F2 诊断帧 */
		}
	}

	return ok;
}

/**
  * @brief  电机重启自愈：一旦再收到 0x0312 上电广播，就重做一遍初始化
  *
  *         ★ 电机控制器复位后，设备号 / 参数 / 轮径全部丢失，
  *           表现是「只回 ACK、不转也不报位移」，必须重新绑定才能恢复。
  *           实测：重新绑 SN ×2 + 0x0316 之后，位移/力矩/心跳立刻回来
  *           （0 -> 位移126 力矩126 心跳13）。
  * @param  None
  * @retval None
  */
static void Bu_ReinitIfMotorRebooted(void)
{
	static uint32_t last_sn_count = 0U;
	static uint8_t  first_call    = 1U;

	if (first_call != 0U)
	{
		/* 第一次进来只记基线：上电时那两条 0x0312 是正常启动广播，不算重启 */
		first_call    = 0U;
		last_sn_count = g_motor_rx_sn_count;
		return;
	}

	if (g_motor_rx_sn_count == last_sn_count)
	{
		return;
	}
	last_sn_count = g_motor_rx_sn_count;

	if (g_car_bu_inited == 0U)
	{
		return;                          /* 第一次初始化还没跑完 */
	}

	Car_BuStop();
	osDelay(200U);

	/* 电机真的重新上电了（0x0312 广播），参数已经丢，允许再写一次 0x0316 */
	s_param_written = 0U;
	s_force_param   = 1U;
	Car_Motor_Init();
	s_force_param   = 0U;
	osDelay(500U);
}

/**
  * @brief  脱机总入口：上电自检 + 动作测试序列（不会返回）
  *
  *         行为由 car_bringup.h 的 CAR_BU_RUN_MODE 决定：
  *             CAR_BU_RUN_HEARTBEAT_ONLY : 只发心跳，不碰电机（验证 CAN 发送链路用）
  *             CAR_BU_RUN_DIAG_ONLY      : 初始化 + 轮径诊断
  *             CAR_BU_RUN_FULL           : 诊断 + 直行 + 转向（默认）
  *             CAR_BU_RUN_ROUNDTRIP      : 课题四·直线往返 5m × 5 次 × 3 轮
  *             CAR_BU_RUN_SQUARE         : 课题五·方形 2m 边长 × 5 个 × 1 轮
  * @param  None
  * @retval None
  */
void Car_BuBootSequence(void)
{
	/* 等 300ms：CAN 收发器上电 + 总线稳定 */
	osDelay(300U);

	/* ★ 等电机控制器启动完成（靠它自己广播 0x0312 来判定），而不是盲等 500ms */
	Bu_WaitMotorBoot();

#if (CAR_BU_RUN_MODE == CAR_BU_RUN_HEARTBEAT_ONLY)
	/* 纯心跳模式：一帧都不往电机发，纯粹验证 STM32 的 CAN 能不能出去 */
	g_car_bu_seq_stage = 5U;
	for (;;)
	{
		osDelay(1000U);
	}
#else
	/* 绑定 + 配参数 + 配轮径 + 使能 */
	Car_Motor_Init();

	/* ★★★ 自动识别方向与左右轮（推车法）★★★
	 *   必须在 Car_Motor_Init 之后：只有电机使能了、目标=当前位置，
	 *   推车才会产生"剩余位移"和抵抗力矩，识别才有信号可用。
	 *   识别失败不中断，回退到默认方向（设备0=+1，设备1=-1）。 */
#if (CAR_BU_DIRDET_ENABLE != 0)
	(void)Car_AutoDetectDirections();
#endif

/* ★★★ 注意：每一条 #if / #elif / #else / #endif 都必须【独占一行】★★★
 *
 *   曾经这里写成：
 *        Car_BuDiagOnly();  #elif (CAR_BU_RUN_MODE == CAR_BU_RUN_ROUNDTRIP)
 *   预处理器只认【行首】的 #elif，同一行后面的 #elif 会被当成普通文本。
 *   于是 #if 找不到 #elif，一路跳到最后的 #else，
 *   实际执行的变成了 Car_BuRunTestSequence()（前进500/后退500/转90/转回90），
 *   而编译器一声不吭。用户看到的就是"前进后退到原点然后转个90度继续"。
 *
 *   现在末尾的 #else 改成 #error：所有合法模式都必须有明确分支，
 *   一旦再出现这种"跳到 else"的错位，编译期就会报错，不会静默跑错。
 */
#if (CAR_BU_RUN_MODE == CAR_BU_RUN_DIAG_ONLY)
	Car_BuDiagOnly();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_ROUNDTRIP)
	/* 课题四：直线往返测试 */
	Car_BuRunRoundTrip();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_SQUARE)
	/* 课题五：方形测试 */
	Car_BuRunSquare();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_SPIN)
	/* 纯诊断：原地 360° 转圈标定（量转角增益误差）*/
	Car_BuRunSpinCal();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_STRAIGHT)
	/* 纯诊断：直行测试（量距离精度与跑偏 °/m）*/
	Car_BuRunStraight();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_ANGLECHK)
	/* 纯诊断：转角误差分辨（判固定偏移还是增益误差）*/
	Car_BuRunAngleCheck();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_TURNTEST)
	/* ★ 开环转角物理验证：转 5 圈 360°，全程不读陀螺仪 */
	Car_BuRunTurnTest();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_DIRTEST)
	/* ★ 方向自检：慢速前进 300mm 再后退，肉眼确认前进/后退没搞反 */
	Car_BuRunDirTest();
#elif (CAR_BU_RUN_MODE == CAR_BU_RUN_FULL)
	/* 动作测试序列（前进/后退/左转/右转，内含 2s 稳定等待，最后死循环）*/
	Car_BuRunTestSequence();
#else
#error "CAR_BU_RUN_MODE 落到了 #else —— 说明上面的 #elif 链断了（多半是某条预处理指令没独占一行）"
#endif
#endif
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  四舍五入（远离 0）
  *         float -> int32 直接截断会把 0.7 变成 0，位移就凭空少了
  */
static int32_t Bu_Round(float v)
{
	return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/** @brief 取绝对值（不引 math.h，省 flash） */
static float Bu_AbsF(float v)
{
	return (v >= 0.0f) ? v : -v;
}

/** @brief 把角度差归一化到 -180 ~ +180（陀螺仪读数是循环的） */
static float Bu_Wrap180(float deg)
{
	while (deg > 180.0f)   { deg -= 360.0f; }
	while (deg < -180.0f)  { deg += 360.0f; }
	return deg;
}

/** @brief 取绝对值（int32） */
static int32_t Bu_AbsI32(int32_t v)
{
	return (v >= 0) ? v : -v;
}

/**
  * @brief  距离标定系数（1.0 = 不补偿）
  *         见 car_bringup.h 里 CAR_BU_DIST_SCALE_PCT 的推导
  */
static float Bu_Scale(void)
{
	float s = (float)CAR_BU_DIST_SCALE_PCT / 100.0f;

	return (s > 0.0f) ? s : 1.0f;       /* 防止误填 0 / 负数把车变成不动 */
}

/**
  * @brief  mm -> 0.1mm（协议位移单位），★ 全文件唯一乘 10 的地方
  */
static int32_t Bu_MmTo0p1mm(float mm)
{
	return Bu_Round(mm * 10.0f * Bu_Scale());
}

/**
  * @brief  mm/s -> 0.1mm/s（协议速度单位），★ 全文件唯一乘 10 的地方
  *         与位移同乘一个系数，加速时间不变（速度和距离等比例放大）
  */
static uint32_t Bu_MmpsTo0p1mmps(float mm_s)
{
	float v = (mm_s >= 0.0f) ? mm_s : -mm_s;
	float c = v * 10.0f * Bu_Scale();

	if (c < 1.0f)
	{
		c = 1.0f;                       /* 速度 0 电机不动，给最小 1 (0.1mm/s) */
	}
	if (c > 4000000000.0f)
	{
		c = 4000000000.0f;              /* 防止超出 u32 */
	}
	return (uint32_t)Bu_Round(c);
}

/**
  * @brief  加速度（0.1mm/s^2），同样按标定系数放大
  */
static uint32_t Bu_AccScaled(void)
{
	float a = (float)CAR_BU_ACC_0P1MMPS2 * Bu_Scale();

	if (a < 1.0f)
	{
		a = 1.0f;
	}
	return (uint32_t)Bu_Round(a);
}

/**
  * @brief  把调用者传进来的加速度换算成协议值（同样乘标定系数）
  *         ★ 传 0 表示"用默认值"，不传 0 就用传进来的
  * @param  acc_0p1mmps2  加速度 0.1mm/s²（未标定），0 = 默认
  * @retval 协议用的加速度 0.1mm/s²（已标定）
  */
static uint32_t Bu_AccTo0p1mmps2(uint32_t acc_0p1mmps2)
{
	float a;

	if (acc_0p1mmps2 == 0U)
	{
		return Bu_AccScaled();              /* 0 = 用默认值 */
	}

	a = (float)acc_0p1mmps2 * Bu_Scale();
	if (a < 1.0f)
	{
		a = 1.0f;
	}
	return (uint32_t)Bu_Round(a);
}

/**
  * @brief  按「距离 ÷ 速度」估算这一步最多要等多久（含加速/减速余量）
  *
  *         ★ 为什么必须动态算：固定 20s 的超时对 500mm 够用，
  *           但 5m 就要 33s（150mm/s），固定超时会在半路把运动掐断，
  *           车只走了一半就收到下一条指令 —— 实测这样会把电机搞死。
  * @param  distance_mm  位移 mm（取绝对值）
  * @param  speed_mm_s   速度 mm/s（取绝对值）
  * @retval 超时 ms（不低于 CAR_BU_SEQ_MOVE_TIMEOUT_MS）
  */
static uint32_t Bu_MoveTimeout(float distance_mm, float speed_mm_s)
{
	float    d = (distance_mm >= 0.0f) ? distance_mm : -distance_mm;
	float    v = (speed_mm_s  >= 0.0f) ? speed_mm_s  : -speed_mm_s;
	float    ms;
	uint32_t t;

	if (v < 1.0f)
	{
		v = 1.0f;                           /* 防止除零 */
	}

	ms = (d / v) * 1000.0f * CAR_TST_TIMEOUT_FACTOR;
	if (ms > 4000000.0f)
	{
		ms = 4000000.0f;                    /* 别算出个天文数字 */
	}

	t = (uint32_t)ms + CAR_TST_TIMEOUT_EXTRA_MS;

	if (t < CAR_BU_SEQ_MOVE_TIMEOUT_MS)
	{
		t = CAR_BU_SEQ_MOVE_TIMEOUT_MS;     /* 太短就没意义了 */
	}
	return t;
}

/**
  * @brief  原地转向的超时：按 角度 ÷ 角速度 估
  */
static uint32_t Bu_TurnTimeout(float angle_deg, float speed_deg_s)
{
	float    a = (angle_deg   >= 0.0f) ? angle_deg   : -angle_deg;
	float    v = (speed_deg_s >= 0.0f) ? speed_deg_s : -speed_deg_s;
	float    ms;
	uint32_t t;

	if (v < 1.0f)
	{
		v = 1.0f;
	}

	ms = (a / v) * 1000.0f * CAR_TST_TIMEOUT_FACTOR;
	if (ms > 4000000.0f)
	{
		ms = 4000000.0f;
	}

	t = (uint32_t)ms + CAR_TST_TIMEOUT_EXTRA_MS;
	if (t < CAR_BU_SEQ_MOVE_TIMEOUT_MS)
	{
		t = CAR_BU_SEQ_MOVE_TIMEOUT_MS;
	}
	return t;
}

/**
  * @brief  统一发送入口：顺手统计成功/失败次数，方便 Watch 观察
  * @retval 0 成功，1 失败
  */
static uint8_t Bu_Send(uint16_t id, const uint8_t *data, uint8_t dlc)
{
	uint8_t r = Can_SendStd(id, data, dlc);

	if (r == 0U)
	{
		g_car_bu_tx_count++;
	}
	else
	{
		g_car_bu_tx_fail++;
	}
	return r;
}

/**
  * @brief  绑定 SN：0x0313，DLC=8，数据 = [SN0..SN6, 设备号]
  */
static uint8_t Bu_SendBind(uint8_t dev, const uint8_t *sn7)
{
	uint8_t d[8];
	uint8_t i;

	for (i = 0U; i < 7U; i++)
	{
		d[i] = sn7[i];
	}
	d[7] = dev;

	return Bu_Send(CAN_ID_MOTOR_BIND_SN, d, 8U);
}

/**
  * @brief  配置电机参数：0x0316，DLC=8
  *         B4 BF CF AA 00 5F [方向位Bit7 | 回传位 | 马达号bit3..0] [设备号]
  * @param  dev        设备号（0x0313 绑定时的槽位），决定这一帧被哪台电机接收
  * @param  motor_no   要写进电机的马达号（0 = 左轮，1 = 右轮）
  * @param  dir_reversed 1 = Byte6 bit7 置位（电机正方向反装）
  * @retval 0 成功，1 失败
  *
  * ★★ 这是整个课题二的关键：Byte6 的低 4 位才是"马达号"，也就是每条运动帧
  *    Data[4] 用来寻址的那个号。协议文档把它和设备号写成了同一个名字，
  *    所以之前两台电机的马达号一直是 0，命令永远被两台同时执行。
  */
static uint8_t Bu_SendParam(uint8_t dev, uint8_t motor_no, uint8_t dir_reversed)
{
	uint8_t d[8];

	d[0] = (uint8_t)(((uint8_t)CAR_MOTOR_BYTE0_HI << 4) | (uint8_t)CAR_MOTOR_POLE_PAIRS);
	d[1] = (uint8_t)(((uint8_t)CAR_MOTOR_SPEED_P << 4) | (uint8_t)CAR_MOTOR_SPEED_I);
	d[2] = (uint8_t)(((uint8_t)CAR_MOTOR_TORQUE_P << 4) | (uint8_t)CAR_MOTOR_TORQUE_I);
	d[3] = (uint8_t)(((uint8_t)CAR_MOTOR_CURRENT_CW << 4) | (uint8_t)CAR_MOTOR_CURRENT_CCW);
	d[4] = (uint8_t)((((uint8_t)CAR_MOTOR_BRAKE != 0U) ? 0x02U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_DAMPING != 0U) ? 0x01U : 0x00U));
	d[5] = (uint8_t)(((uint8_t)CAR_MOTOR_SPEED_D << 4) | (uint8_t)CAR_MOTOR_TORQUE_D);
	d[6] = (uint8_t)(((dir_reversed != 0U) ? 0x80U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_AUTO_UNLOCK != 0U) ? 0x40U : 0x00U) |
	                 (((uint8_t)CAR_MOTOR_RETURN_REMAIN != 0U) ? 0x00U : 0x20U) |
	                 (((uint8_t)CAR_MOTOR_RETURN_TORQUE != 0U) ? 0x00U : 0x10U) |
	                 (uint8_t)(motor_no & 0x0FU));
	d[7] = dev;

	return Bu_Send(CAN_ID_MOTOR_CFG_PARAM, d, 8U);
}

/**
  * @brief  配置轮径：0x0408，DLC=4，数据 = [设备号, 0x24, 轮径低字节, 轮径高字节]
  *         7500 (0x1D4C) -> 4C 1D
  */
static uint8_t Bu_SendWheel(uint8_t dev, uint16_t wheel_0p01mm)
{
	uint8_t d[4];

	d[0] = dev;
	d[1] = MOTOR_CFG_SUB_WHEEL;
	d[2] = (uint8_t)(wheel_0p01mm & 0xFFU);         /* 低字节：0x4C */
	d[3] = (uint8_t)(wheel_0p01mm >> 8);            /* 高字节：0x1D */

	return Bu_Send(CAN_ID_MOTOR_CFG_WHEEL, d, 4U);
}

/**
  * @brief  上电使能 / 下电释放：0x0413，DLC=5
  *         数据 = [0x5A 上电使能 / 0x4A 下电释放, 0, 0, 0, 设备号]
  *         ★ 不发这条，电机只更新目标位置但不出力（实测速度只有 28mm/s，
  *           力矩≈0，状态位 Bit0 锁定 = 0）。发了才真正驱动。
  */
static uint8_t Bu_SendLock(uint8_t dev, uint8_t code)
{
	uint8_t d[5];

	d[0] = code;
	d[1] = 0U;
	d[2] = 0U;
	d[3] = 0U;
	d[4] = dev;

	return Bu_Send(CAN_ID_MOTOR_LOCK, d, 5U);
}

/**
  * @brief  设置速度：0x0415（主帧组），DLC=5，数据 = [速度 u32 小端, 马达号]
  * @param  motor_no 0 = 左轮，1 = 右轮
  */
static uint8_t Bu_SendSpeed(uint8_t motor_no, uint32_t speed_0p1mmps)
{
	uint8_t  d[5];

	d[0] = (uint8_t)(speed_0p1mmps);
	d[1] = (uint8_t)(speed_0p1mmps >> 8);
	d[2] = (uint8_t)(speed_0p1mmps >> 16);
	d[3] = (uint8_t)(speed_0p1mmps >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);

	return Bu_Send(CAN_ID_MOTOR_SET_SPD_MAIN, d, 5U);
}

/**
  * @brief  设置加速度：0x0414（主帧组），DLC=5，数据 = [加速度 u32 小端, 马达号]
  * @param  motor_no 0 = 左轮，1 = 右轮
  */
static uint8_t Bu_SendAccel(uint8_t motor_no, uint32_t acc_0p1mmps2)
{
	uint8_t  d[5];

	d[0] = (uint8_t)(acc_0p1mmps2);
	d[1] = (uint8_t)(acc_0p1mmps2 >> 8);
	d[2] = (uint8_t)(acc_0p1mmps2 >> 16);
	d[3] = (uint8_t)(acc_0p1mmps2 >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);

	return Bu_Send(CAN_ID_MOTOR_SET_ACC_MAIN, d, 5U);
}

/**
  * @brief  设置相对位置：0x0416（主帧组），DLC=6，数据 = [s32 0.1mm, 马达号, 0x00]
  *         ★ 负位移不用手工处理：(uint32_t)int32 就是二进制补码
  *            例 -3047 -> 0xFFFFF419 -> 字节 19 F4 FF FF
  * @param  motor_no 0 = 左轮，1 = 右轮
  */
static uint8_t Bu_SendPosRel(uint8_t motor_no, int32_t dist_0p1mm)
{
	uint8_t  d[6];
	uint32_t u = (uint32_t)dist_0p1mm;

	d[0] = (uint8_t)(u);
	d[1] = (uint8_t)(u >> 8);
	d[2] = (uint8_t)(u >> 16);
	d[3] = (uint8_t)(u >> 24);
	d[4] = (uint8_t)(motor_no & 0x0FU);
	d[5] = MOTOR_TARGET_RELATIVE;                   /* 0x00 = 相对位置 */

	return Bu_Send(CAN_ID_MOTOR_SET_POS_MAIN, d, 6U);
}

/**
  * @brief  统一起步：加速度 -> 速度 -> 两轮相对位置（主帧组，靠马达号 0/1 区分左右）
  *
  *         ★ 顺序不能反：先发位置会先用上一次的旧速度走一小段
  *         ★ 帧之间必须留间隙，否则电机来不及处理会把后面的帧丢掉
  *
  *         ★★ 差速就在这里：
  *              左轮 = 0x0414 / 0x0415 / 0x0416，Data[4] = 0
  *              右轮 = 0x0414 / 0x0415 / 0x0416，Data[4] = 1
  *            两个位移取反（异号）-> 车直行（实测转角 0.00°）
  *            两个位移相同（同号）-> 车原地转（实测转角 51.95°）
  *
  * @param  left_0p1mm        左轮位移 0.1mm（正 = 车前进）
  * @param  right_0p1mm       右轮位移 0.1mm（正 = 车前进）
  * @param  speed_0p1mmps     两轮速度 0.1mm/s
  * @retval 0 成功，1 参数非法或发送失败
  */
static uint8_t Bu_StartMotion(int32_t left_0p1mm, int32_t right_0p1mm, uint32_t speed_0p1mmps,
                              uint32_t accel_0p1mmps2)
{
	/* ★ 两台电机镜像安装：同一条「+位移」指令，一台往前、一台往后。
	 *   原来这里用编译期常量 CAR_BU_RIGHT_SIGN 写死"右轮取反"，
	 *   现在改用运行时方向表 g_car_dir_sign[]（由 Car_AutoDetectDirections
	 *   推车测出来），所以换车换电机都不用改代码。
	 *   表里 +1 = 该电机正方向就是车前进，-1 = 需要取反。 */
	return Bu_EmitMotion(left_0p1mm  * g_car_dir_sign[0],
	                     right_0p1mm * g_car_dir_sign[1],
	                     speed_0p1mmps, accel_0p1mmps2);
}

/**
  * @brief  直接按原始位移下发（★ 不套 g_car_dir_sign[]）
  *
  *         只在「自动识别方向」阶段用：那时方向还不知道，
  *         必须发原始值才能从车的实际动作反推方向。
  * @param  l_raw/r_raw      原始位移 0.1mm
  * @param  speed_0p1mmps    两轮速度 0.1mm/s
  * @retval 0 成功，1 参数非法或发送失败
  */
static uint8_t Bu_EmitMotion(int32_t left_0p1mm, int32_t right_0p1mm, uint32_t speed_0p1mmps,
                             uint32_t accel_0p1mmps2)
{
	uint32_t acc;

	/* 不动就不发：避免参数算错时车突然窜一下 */
	if ((speed_0p1mmps == 0U) || ((left_0p1mm == 0) && (right_0p1mm == 0)))
	{
		return 1U;
	}

	g_car_bu_last_delta = left_0p1mm;
	g_car_bu_last_speed = speed_0p1mmps;

	/* ★ 记下这条指令的物理最短运行时间 = 最大轮位移 ÷ 速度。
	 *   Bu_WaitMotionDone 用它做兜底：一次都没见到 0x0420 报"忙"时，
	 *   至少等这么久才能认为车可能停了（否则会在运动中改目标）。 */
	{
		int32_t lm = (left_0p1mm  >= 0) ? left_0p1mm  : -left_0p1mm;
		int32_t rm = (right_0p1mm >= 0) ? right_0p1mm : -right_0p1mm;
		int32_t dm = (lm > rm) ? lm : rm;

		s_expect_ms = (uint32_t)(((uint32_t)dm * 1000U) / speed_0p1mmps);
	}

	/* 新指令：重新开始判"反馈是否活着" */
	s_motion_seen_busy = 0U;

	/* ★ 运动期间冻结陀螺仪零偏学习（见 car_bringup.h 的 CAR_BU_FREEZE_GYRO_BIAS）。
	 *   在 Bu_WaitMotionDone() 里车停稳后再打开。 */
#if (CAR_BU_FREEZE_GYRO_BIAS != 0)
	Angle_SetBiasLearning(0U);
#endif

	acc = Bu_AccTo0p1mmps2(accel_0p1mmps2);

#if (CAR_BU_SEND_ACCEL != 0)
	if (Bu_SendAccel(CAR_BU_LEFT_MOTOR_NO,  acc) != 0U) { return 1U; }
	osDelay(CAR_BU_MOTION_GAP_MS);
	if (Bu_SendAccel(CAR_BU_RIGHT_MOTOR_NO, acc) != 0U) { return 1U; }
	osDelay(CAR_BU_MOTION_GAP_MS);
#endif

	if (Bu_SendSpeed(CAR_BU_LEFT_MOTOR_NO,  speed_0p1mmps) != 0U) { return 1U; }
	osDelay(CAR_BU_MOTION_GAP_MS);
	if (Bu_SendSpeed(CAR_BU_RIGHT_MOTOR_NO, speed_0p1mmps) != 0U) { return 1U; }
	osDelay(CAR_BU_MOTION_GAP_MS);

	/* ★★★ 这两帧【紧挨着发，中间不留延时】★★★
	 *
	 *   原来的写法两轮之间插 20ms，于是右轮的目标位置比左轮晚 20ms 生效：
	 *   起步瞬间左轮先动、右轮后动，车头先别一下，之后整段都带着这个倾角。
	 *   这一段能靠下一个角纠回来（绝对航向锚定），但
	 *   【每段都往同一侧别一下】本身就是一种系统性横向偏移，会累积。
	 *
	 *   两帧 CAN 传输只差 ~0.3ms，压到一起后左右起步基本同步。
	 *   加速度/速度帧仍然留间隔（电机来不及处理连续帧），
	 *   但位置目标是"决定什么时候开始走"的那一帧，必须对齐。 */
	if (Bu_SendPosRel(CAR_BU_LEFT_MOTOR_NO,  left_0p1mm) != 0U)  { return 1U; }
	if (Bu_SendPosRel(CAR_BU_RIGHT_MOTOR_NO, right_0p1mm) != 0U) { return 1U; }

	return 0U;
}
