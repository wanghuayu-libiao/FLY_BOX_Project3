/**
  ******************************************************************************
  * @file    main.c
  * @brief   程序入口：FreeRTOS(CMSIS-RTOS v1) 外设测试框架
  *          用文件顶部的 CURRENT_TEST_MODE 宏切换当前测试对象：
  *          TEST_MODE_LED  ：任务 Task_Led   每 500ms 翻转一次 LED
  *          TEST_MODE_CAN  ：任务 Task_CanTxTest 每 500ms 发送一字节 0xAA（标准帧 ID 0x123）
  *          TEST_MODE_GYRO ：任务 Task_Gyro  每 500ms 逐接口自检：读状态(0x04)/角速度/温度，
  *                            并对 0x01 寄存器做写-读回校验（结果都在 debug_* 全局变量里）
  *          TEST_MODE_SPI  ：任务 Task_SpiTest 连续调用 Spi_ReadWriteByte(0xAA)（不加延时，
  *                            纯 SPI 底层测试，配合示波器抓 SPI2 波形）
  *          TEST_MODE_APP  ：最终应用（第六步）：Task_Angle 每 10ms 算角度（含零偏处理），
  *                            Task_CanTx 把角度/温度发给上位机，Task_CanRx 处理上位机命令
  *                            （报文格式见 inc/protocol.h）
  *          TEST_MODE_MOTOR：课题二·电机自检：Task_Motor 非阻塞跑完两台电机的初始化序列
  *                            （绑 SN / 配参数 / 配轮径 / 设零位 / 使能 / 查位置），
  *                            Task_CanRx 把 0x0420+ / 0x0409 / 0x041A / 0x0312 等
  *                            电机反馈交给 Motor_OnFrame 解析，结果看 g_motor_* 变量
  *          TEST_MODE_BRINGUP：课题二·脱机自检：Task_Bringup 调 Car_BuBootSequence()，
  *                            上电自动绑两台电机 -> 前进 500mm -> 后退 500mm
  *                            -> 原地左转 90° -> 原地右转 90° -> 停车，无限循环；
  *                            进度看 g_car_bu_seq_stage / g_car_bu_seq_ret[]，
  *                            发送计数看 g_car_bu_tx_count / g_car_bu_tx_fail；
  *                            同时保留陀螺仪上报（Task_Angle + Task_CanTx，0x101/0x102），
  *                            所以上位机仍能看到角度曲线，可用它验证转向角度/方向
  *          TEST_MODE_HOST   ：课题二·上位机控车：固件只发心跳(0x7FE) + 陀螺仪(0x101/0x102)，
  *                            **不创建 Task_Bringup，一帧都不往电机发**；
  *                            电机交给上位机（USB-CAN 适配器）通过 canable2 全权控制。
  *                            ★ 为什么必须分两个模式：STM32 和 USB-CAN 适配器是两个
  *                              独立主站，同一 CAN 总线上谁后发谁生效。固件只要还在
  *                              跑 BRINGUP 序列，上位机点的直行/转向就会被它下一帧覆盖，
  *                              现象就是"点了没反应，只有两台停止有反应"。
  *          （Task_Led 在所有模式下都会创建：LED(PB9) 每 500ms 翻转一次，
  *            作"心跳灯"指示程序在运行——灯闪即运行中）
  *          ★ 任务里延时必须用 osDelay（让出 CPU）；HAL_Delay 是忙等，
  *            只允许在 osKernelStart() 之前的外设初始化阶段使用
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"                   /* FreeRTOS(CMSIS-RTOS v1)：osThread/osDelay/osKernelStart */
#include "led.h"
#include "can.h"
#include "spi.h"
#include "xv7001bb.h"                   /* XV7001BB 陀螺仪驱动（SPI2） */
#include "angle.h"                      /* 角度计算 + 零偏处理 */
#include "protocol.h"                   /* 上位机 CAN 报文定义 */
#include "motor.h"                      /* 电机 CAN 协议层 + 非阻塞初始化（课题二） */
#include "car_bringup.h"                /* 直发版双电机绑定 + 差速运动学（课题二 分步调机） */
#include "tof8.h"                       /* 课题三·TOF8 激光测距（CAN 协议见 tof8通讯协议.docx） */
#include "car_cfg.h"                    /* TOF8 设备号数量/距离上报周期等配置 */
#include "flybox.h"                     /* 课题三·飞箱推拉箱：5 台电机 + 语义动作 */

/* Private define ------------------------------------------------------------*/
/* 测试模式宏开关 */
#define TEST_MODE_LED       1
#define TEST_MODE_CAN       2
#define TEST_MODE_GYRO      3
#define TEST_MODE_SPI       4               /* 纯 SPI 底层波形测试（不碰其它外设，方便示波器抓波形） */
#define TEST_MODE_APP       5               /* 最终应用：角度计算 + 零偏处理 + CAN 收发 */
#define TEST_MODE_MOTOR     6               /* 课题二·电机自检：绑定 SN / 配参数 / 配轮径 /
                                               设零位 / 使能 / 查位置，全程非阻塞，Watch 看结果 */
#define TEST_MODE_BRINGUP   7               /* 课题二·脱机整车：Car_BuBootSequence() 上电自动
                                               绑定两台电机 -> 前进 500mm -> 后退 500mm
                                               -> 原地左/右转 90° -> 停车，无限循环
                                               参数全在 car_bringup.h（含距离标定系数）*/
#define TEST_MODE_HOST      8               /* 课题二·上位机控车：固件只发心跳(0x7FE) + 陀螺仪
                                               (0x101/0x102)，**一帧都不往电机发**，
                                               电机全部交给上位机（USB-CAN 适配器）控制。
                                               ★ 为什么必须单独一个模式：STM32 和 USB-CAN
                                                 适配器是两个独立主站，同一 CAN 总线上谁后发
                                                 谁生效。只要固件还在跑 BRINGUP 序列，
                                                 上位机点的直行/转向就会被固件下一帧覆盖。*/
#define TEST_MODE_TOF8      9               /* ★ 课题三·TOF8 测距自检：只发 0x0316 配置 +
                                               0x0408 通道 10/12/15/16 请求，不碰电机、
                                               不碰陀螺仪。距离每 100ms 用 0x7F8 上报，
                                               用 CAN 分析仪就能看到毫米值。
                                               ★ 电机一帧都不发：本模式只验证 TOF8 通信
                                                 和测距，避免电机干扰排查。*/
#define TEST_MODE_FLYBOX    10              /* ★ 课题三·飞箱机构调试：扫描 5 台电机设备号
                                               -> 配参数/使能 -> 单轴点动 -> 语义动作。
                                               ★ 上位机用 0x111 下发点动命令，
                                                 不看调试器也能调；
                                                 状态看 0x7F5/0x7F6/0x7F4 */

/* ★★★ 课题四/五（往返测试、方形测试）不用在这里加新模式 ★★★
 *
 *   它们是 TEST_MODE_BRINGUP 的「子模式」，由 car_bringup.h 里的
 *   CAR_BU_RUN_MODE 选择跑哪一段：
 *       CAR_BU_RUN_FULL       动作序列（前进/后退/左转/右转）
 *       CAR_BU_RUN_ROUNDTRIP  课题四·往返测试
 *       CAR_BU_RUN_SQUARE     课题五·方形测试
 *
 *   为什么不在这里加 TEST_MODE_XXX：
 *     那个宏只在 main.c 里可见，而「跑哪段序列」的判断在 car_bringup.c 里，
 *     两处开关意味着要改两个文件，极易漏改（然后就出现"改了没反应"）。
 *     所以只留 CAR_BU_RUN_MODE 一个开关，main.c 这边保持 TEST_MODE_BRINGUP 不动。
 */

/* BRINGUP 模式的测试参数已全部改到 car_bringup.h（CAR_BU_SEQ_* 系列）*/

#ifndef CURRENT_TEST_MODE
/* ★★★ 烧录前必须确认这一行！★★★
 *
 *  这是整个工程唯一的模式开关。改错了，看到的永远是"完全没反应"：
 *  固件在跑，但跑的是另一个课题的逻辑。
 *
 *  常见误判：
 *    以为在跑飞箱，实际还是 TEST_MODE_BRINGUP
 *      -> 走的是课题二小车逻辑，拿 car_cfg.h 里小车的 SN 去绑飞箱电机
 *      -> SN 对不上，绑不上，0x0413 不生效 -> 电机不锁死
 *      -> 方形测试发运动帧给设备号 0/1（飞箱是 7~11）-> 一帧都不动
 *      -> 现象和「固件没跑」一模一样
 *
 *  当前应为 TEST_MODE_FLYBOX（课题三）。
 */
#define CURRENT_TEST_MODE   TEST_MODE_FLYBOX    /* ★ 课题三·飞箱：5 台电机 + TOF 闭环
                                              *
                                              *   其它可选：
                                              *     TEST_MODE_HOST    = 上位机控车
                                              *     TEST_MODE_BRINGUP = 课题二脱机整车
                                              *     TEST_MODE_TOF8    = 只验 TOF，不碰电机
                                              *
                                              *   ★ 三者不能同时用：固件一旦自己发运动帧，
                                              *     上位机的指令就会被覆盖（谁后发谁生效）。*/
#endif

/* ★★★ 拼写防护（别删）★★★
 *
 * 这个宏名字打错一个字母，后果是"整个程序什么都不做"，而且编译器一声不吭：
 * 预处理器把 #if 里未定义的标识符当作 0，于是下面所有
 *     #if (CURRENT_TEST_MODE == TEST_MODE_XXX)
 * 全部为 false —— 外设一个都不初始化、任务一个都不创建。
 * 唯一还在跑的是 osKernelStart() 和 idle 任务，现象就是「灯不闪、车不动」，
 * 看起来像"烧录失败"或"单片机坏了"，极难往"宏拼错了"上想。
 *
 * 下面这段在编译期就把它拦下来：#if 里引用未定义的宏会被当成 0，
 * 所以只要 CURRENT_TEST_MODE 不是任何一个合法模式，就直接报错停编。
 */
#if !defined(CURRENT_TEST_MODE) \
    || ((CURRENT_TEST_MODE != TEST_MODE_LED) \
        && (CURRENT_TEST_MODE != TEST_MODE_CAN) \
        && (CURRENT_TEST_MODE != TEST_MODE_GYRO) \
        && (CURRENT_TEST_MODE != TEST_MODE_SPI) \
        && (CURRENT_TEST_MODE != TEST_MODE_APP) \
        && (CURRENT_TEST_MODE != TEST_MODE_MOTOR) \
        && (CURRENT_TEST_MODE != TEST_MODE_BRINGUP) \
        && (CURRENT_TEST_MODE != TEST_MODE_HOST) \
        && (CURRENT_TEST_MODE != TEST_MODE_TOF8) \
        && (CURRENT_TEST_MODE != TEST_MODE_FLYBOX))
#error "CURRENT_TEST_MODE 不是合法的测试模式（很可能宏名拼错了，检查 TEST_MODE_HOST / TEST_MODE_BRINGUP 的拼写）"
#endif

/* 任务间通知信号（CMSIS-RTOS v1 osSignal） */
#define SIG_ANGLE_READY     0x01            /* Task_Angle -> Task_CanTx：新一帧角度算好了 */
#define SIG_CAN_RX          0x01            /* CAN 接收中断 -> Task_CanRx：队列里有新帧 */

/* 课题三·TOF8 自检任务参数 */
#define TOF8_BOOT_WAIT_MS       800U        /* 上电等模块稳定（连续模式首次出数需要 >100ms）*/
#define TOF8_CFG_GAP_MS         20U         /* 配置/开关帧之间的间隔 */
#define TOF8_DIAG_PERIOD_MS     100U        /* 0x7F8 距离上报周期 */
#define TOF8_RECHECK_PERIOD    20U          /* 每 N 个上报周期（=2s）补查一次设备号/分辨率/频率 */

/* Global variables ----------------------------------------------------------*/
/* Watch 变量（加入调试器 Watch 观察） */
volatile uint8_t debug_status = 0;      /* XV7001BB 状态寄存器(0x04)读回值 */
volatile int32_t debug_angle  = 0;      /* XV7001BB 角速度原始值（16 位补码） */
volatile float   debug_temp     = 0.0f; /* XV7001BB 温度值（℃，浮点） */
volatile int32_t debug_temp_int = 0;    /* 温度 ×100 的整数（例：25℃ -> 2500，方便 Watch 观察） */
volatile uint8_t debug_wrback   = 0;    /* 写-读回校验：1=成功（读回值已改变），0=失败（没写进去） */

/* FreeRTOS 任务句柄（调试用）：TaskLed 在所有模式都创建（心跳灯），其余任务只创建当前模式对应的 */
osThreadId TaskLedHandle  = NULL;
osThreadId TaskCanHandle  = NULL;
osThreadId TaskGyroHandle = NULL;
osThreadId TaskSpiHandle  = NULL;
osThreadId TaskAngleHandle = NULL;
osThreadId TaskCanTxHandle = NULL;
osThreadId TaskCanRxHandle = NULL;
osThreadId TaskMotorHandle = NULL;
osThreadId TaskBringupHandle = NULL;
osThreadId TaskHeartbeatHandle = NULL;
osThreadId TaskTof8Handle = NULL;          /* 课题三·TOF8 测距轮询 */
osThreadId TaskFlyboxHandle = NULL;        /* 课题三·飞箱机构调试 */

/* BRINGUP 测试的进度已由 car_bringup.c 的 g_car_bu_seq_stage 提供：
 *   0=等待 1=直行 2=转向 3=停车 4=完成
 * 另有 g_car_bu_seq_ret[0]=直行返回值、[1]=转向返回值（0=成功）、
 *      g_car_bu_last_arc（未标定弧长 0.1mm）、g_car_bu_scale_pct（标定系数）、
 *      g_car_bu_last_delta / g_car_bu_last_speed（最终下发的值，含标定）*/

/* Private function prototypes -----------------------------------------------*/
/* 本文件内部函数（static：避免与本工程 bsp.c 中的同名函数冲突） */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* FreeRTOS 任务（CMSIS-RTOS v1） ---------------------------------------------*/
/* ★ 规则：任务里的延时必须用 osDelay —— 它会挂起当前任务、把 CPU 让给其它任务；
 *   HAL_Delay 是"忙等"（死循环查 SysTick 计数），在任务里会阻塞调度、浪费 CPU，禁止使用 */

/**
  * @brief  LED 测试任务：每 500 ms 翻转一次 LED
  * @param  argument 未使用（CMSIS-RTOS v1 任务函数固定为 void const *）
  * @retval None（死循环，永不返回）
  */
static void Task_Led(void const *argument)
{
	(void)argument;

	for (;;)
	{
		Led_Toggle();                   /* LED 翻转 */
		osDelay(500);                   /* 替代 HAL_Delay(500)：让出 CPU */
	}
}

/**
  * @brief  CAN 测试任务：每 500 ms 通过 CAN 发送一字节 0xAA（标准帧 ID 0x123）
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_CanTxTest(void const *argument)
{
	(void)argument;

	for (;;)
	{
		Can_SendByte(0xAA);             /* CAN 发送 0xAA */
		osDelay(500);                   /* 替代 HAL_Delay(500)：让出 CPU */
	}
}

/**
  * @brief  陀螺仪逐接口自检任务：每 500 ms 把驱动所有接口跑一遍
  *         1) 读状态寄存器 0x04 -> debug_status
  *            （判据：读回值非 0x00 且非 0xFF = SPI 通信正常）
  *         2) 读角速度 0x0A     -> debug_angle（16 位补码原始值）
  *         3) 读温度   0x08     -> debug_temp（℃，浮点）
  *                               + debug_temp_int（×100 整数，例：25℃ -> 2500）
  *         4) 写-读回校验 0x01（DspCtl1，HPF 配置寄存器，写错不损坏芯片）：
  *            读原值 -> 写测试值 -> 读回 -> 恢复原值
  *            -> debug_wrback（1 = 成功：读回值已改变；0 = 失败：读回值没变/没写进去）
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_Gyro(void const *argument)
{
	uint8_t old_val;
	uint8_t test_val;
	uint8_t new_val;

	(void)argument;

	for (;;)
	{
		/* --- 1. 读状态寄存器 0x04（非 0x00 且非 0xFF = 通信正常） --- */
		debug_status = XV7001bb_ReadReg(0x04);

		/* --- 2. 读角速度 0x0A（16 位补码原始值） --- */
		debug_angle  = XV7001bb_ReadAngle();

		/* --- 3. 读温度 0x08；同时乘 100 存成整数，方便 Watch（例：25℃ -> 2500） --- */
		debug_temp     = XV7001bb_ReadTmp();
		debug_temp_int = (int32_t)(debug_temp * 100.0f);

		/* --- 4. 写-读回校验：0x01 = DspCtl1（HPF 配置寄存器，写错不损坏芯片） ---
		 *     测试值故意取"与原值不同"的值（原值是 0x00 就写 0xFF，否则写 0x00）：
		 *     这样"读回没变"=写失败，"读回变了"=写成功；若固定写 0x00，
		 *     恰好原值本就是 0x00 时会把"没变化"误判成写失败 */
		old_val  = XV7001bb_ReadReg(0x01);                  /* 保存原值 */
		test_val = (old_val == 0x00) ? 0xFF : 0x00;         /* 选一个与原值不同的测试值 */
		XV7001bb_WriteData(0x01, test_val);                 /* 写入测试值 */
		new_val  = XV7001bb_ReadReg(0x01);                  /* 读回 */
		debug_wrback = (old_val == new_val) ? 0 : 1;        /* 0=校验失败（没写进去），1=成功 */
		XV7001bb_WriteData(0x01, old_val);                  /* 恢复原值 */

		osDelay(500);                   /* 替代 HAL_Delay(500)：让出 CPU */
	}
}

/**
  * @brief  SPI 底层波形测试任务：连续调用 Spi_ReadWriteByte(0xAA)，不加任何延时
  *         用途：示波器抓 SPI2 波形（SCK=PB13 / MOSI=PB15），验证寄存器级通信
  *         ★ 优先级(osPriorityBelowNormal)比心跳灯低一档：本任务不主动让出 CPU，
  *           靠心跳灯任务（普通优先级）到期抢占来保证"灯闪 = 程序在跑"；对波形无影响
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_SpiTest(void const *argument)
{
	(void)argument;

	for (;;)
	{
		/* 纯粹测试 SPI 底层通信（不涉及其它外设，不加延时，方便示波器抓波形） */
		Spi_ReadWriteByte(0xAA);
	}
}

/**
  * @brief  角度计算任务：每 1 ms 采样一次陀螺仪（过采样），每 10 ms 输出一个角度块
  *         用 osDelayUntil 保证严格 1 ms 周期（osDelay 会把每轮计算耗时累加进周期，积分会偏）。
  *         优先级高于其它任务，保证 1 ms 响应。每算完一个 10ms 块就通知 Task_CanTx 发送。
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_Angle(void const *argument)
{
	uint32_t wake_time;

	(void)argument;
	wake_time = osKernelSysTick();

	for (;;)
	{
		if (Angle_Step() != 0U)
		{
			if (TaskCanTxHandle != NULL)
			{
				(void)osSignalSet(TaskCanTxHandle, SIG_ANGLE_READY);
			}
		}

		(void)osDelayUntil(&wake_time, ANGLE_SAMPLE_PERIOD_MS);
	}
}

/**
  * @brief  CAN 发送任务（task_can_tx）：每收到一帧新角度就发 0x101（每 10ms），
  *         每 10 帧额外发一次 0x102（温度/状态/标志/零偏，每 100ms）。报文格式见 protocol.h
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_CanTx(void const *argument)
{
	AngleData_t d;
	uint8_t buf[8];
	uint8_t div = 0U;

	(void)argument;

	for (;;)
	{
		(void)osSignalWait(SIG_ANGLE_READY, osWaitForever);
		Angle_GetData(&d);

		Proto_PackAngle(buf, d.angle_deg, d.rate_dps, d.seq);
		(void)Can_SendStd(PROTO_ID_ANGLE, buf, 8U);

		div++;
		if (div >= PROTO_INFO_DIV)
		{
			div = 0U;
			Proto_PackInfo(buf, d.temp_c, d.status, d.flags, d.bias_dps, d.temp_coef);
			(void)Can_SendStd(PROTO_ID_INFO, buf, 8U);

			/* ★ 0x103：标度因子/时间基自检
			 *   [0..3] int32  不回绕累计角度，0.01°  （÷100 = 度）
			 *   [4..5] int16  时间基误差 ppm
			 *   [6..7] uint16 进入 RUN 态后的秒数
			 */
			{
				float    at   = 0.0f;
				int16_t  ppm  = 0;
				uint16_t secs = 0U;

				Angle_GetTotal(&at, &ppm, &secs, NULL);

				{
					int32_t q = (int32_t)(at * 100.0f);
					buf[0] = (uint8_t)((uint32_t)q & 0xFFU);
					buf[1] = (uint8_t)(((uint32_t)q >> 8) & 0xFFU);
					buf[2] = (uint8_t)(((uint32_t)q >> 16) & 0xFFU);
					buf[3] = (uint8_t)(((uint32_t)q >> 24) & 0xFFU);
					buf[4] = (uint8_t)((uint16_t)ppm & 0xFFU);
					buf[5] = (uint8_t)(((uint16_t)ppm >> 8) & 0xFFU);
					buf[6] = (uint8_t)(secs & 0xFFU);
					buf[7] = (uint8_t)(secs >> 8);
				}
				(void)Can_SendStd(PROTO_ID_TOTAL, buf, 8U);
			}
		}
	}
}

/**
  * @brief  CAN 接收任务（task_can_rx）：由接收中断唤醒，把队列里的帧取空并处理命令（帧 0x110）
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_CanRx(void const *argument)
{
	CanFrame_t f;

	(void)argument;

	for (;;)
	{
		(void)osSignalWait(SIG_CAN_RX, osWaitForever);

		while (Can_RxPop(&f) != 0U)
		{
			/* ★ 顺序绝对不能反：TOF8 和电机共用 0x0409 应答 ID，
			 *   TOF8 的通道 8（复位 VL53Lx）又正好等于 MOTOR_QUERY_POS。
			 *   必须先让 Tof8_OnFrame() 按设备号「认领」自己的帧，返回 1 就不要再往下交，
			 *   否则 TOF8 的复位应答会被 Motor_OnFrame() 当成「当前位置」写进缓存。*/
			if (Tof8_OnFrame(&f) == 0U)
			{
				/* 电机反馈/应答帧（0x0420+/0x0440+/0x0409/0x041A/0x0312/错误帧）
				 * 非电机 ID 会被内部直接忽略，与下面的陀螺仪命令帧互不影响 */
				Motor_OnFrame(&f);
			}

			if ((f.id == PROTO_ID_CMD) && (f.dlc >= 1U))
			{
				switch (f.data[0])
				{
				case PROTO_CMD_ZERO_ANGLE:
					Angle_RequestZero();
					break;
				case PROTO_CMD_RECALIBRATE:
					Angle_RequestRecalibrate();
					break;
				default:
					break;
				}
			}

			/* ★ 飞箱调试命令 0x111（上位机下发：点动 / 查位置 / 重扫设备号 / 语义动作）
			 *   与陀螺仪命令 0x110 是不同 ID，互不影响。 */
			if (f.id == FLY_ID_CMD)
			{
				Fly_OnCommand(f.data, f.dlc);
			}
		}
	}
}

/**
  * @brief  电机初始化任务（课题二 第二步）：非阻塞地跑完两台电机的初始化序列
  *         顺序：绑定 SN(0x0313) -> 配参数(0x0316) -> 配轮径(0x0408 ×3)
  *               -> 设当前位置=0(0x0416 类型 0x0B) -> 使能(0x0413 0x5A)
  *               -> 设加速度(0x0414) -> 查位置(0x0408/0x08)，先左后右
  *         全部完成后 g_motor_ready = 1；结果在 g_motor_* 一组 Watch 变量里
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_Motor(void const *argument)
{
	uint32_t wake_time;

	(void)argument;
	wake_time = osKernelSysTick();

	Motor_StartInit();

	for (;;)
	{
		(void)Motor_Tick();             /* 到时间才发下一帧，立刻返回 */
		(void)osDelayUntil(&wake_time, MOTOR_TICK_PERIOD_MS);
	}
}

/**
  * @brief  STM32 心跳任务：每 500ms 发一帧 0x7FE，用来证明「STM32 到底有没有在总线上说话」
  *
  *         ★ 为什么需要它：上位机的 USB-CAN 适配器和 STM32 是**两个独立主站**，
  *           电机不动时无法区分是「STM32 没发出去」还是「发了但电机没理」。
  *           0x7FE 是电机协议里完全没占用的 ID，任何 CAN 分析仪/上位机日志都能看到。
  *
  *         数据布局：
  *             [0] 0xA5 固定魔数    [1] 序号（0~255 循环，一直在变 = STM32 活着）
  *             [2] 序列阶段         [3] 初始化完成标志
  *             [4] tx_fail 低字节   [5] tx_count 低字节
  *             [6] tx_fail 高字节   [7] tx_count 高字节
  *
  *         判读方法：
  *             看得到 0x7FE 且 [1] 在变  -> STM32 的 CAN 发送链路完全正常，问题在报文内容/设备号
  *             看不到 0x7FE              -> STM32 的 CAN 收发器、接线或波特率有问题
  *             看得到但 [5][7] 一直 0    -> 心跳能发但电机帧全失败（极端情况）
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_CanHeartbeat(void const *argument)
{
	uint32_t sn_div = 0U;

	(void)argument;

	/* 上电先等 300ms，避免和 Can_Init 抢在同一个时刻 */
	osDelay(300U);

	for (;;)
	{
		Car_BuSendHeartbeat();

		/* ★ 诊断结果（每 500ms）：转角误差分辨测试的结论 + 建议值。
		 *   不接调试器也能用 CAN 分析仪读到，便于外部独立核对。 */
		Car_BuSendDiag();

		/* 每 4 次心跳（2s）发一对 SN 自证帧（左 / 右） */
		if (++sn_div >= (CAR_BU_SN_INFO_MS / CAR_BU_HEARTBEAT_MS))
		{
			sn_div = 0U;
			Car_BuSendSnInfo(0U);
			Car_BuSendSnInfo(1U);
		}

		osDelay(CAR_BU_HEARTBEAT_MS);
	}
}

/**
  * @brief  BRINGUP 测试任务（课题二）：脱机整车自检
  *
  *         整个流程都在 Car_BuBootSequence() 里，本任务只是它的入口：
  *             300ms（等 CAN 收发器上电）
  *             -> 等电机 0x0312 上电广播（最多 6s）
  *             -> Car_Motor_Init()：绑 SN×2 -> 各写一条 0x0316（马达号 0/1）
  *                                   -> 配轮径×3×2 -> 使能×2 -> 加速度/速度×2
  *             -> 2s（让电机消化参数）
  *             -> 前进 500mm @ 60mm/s   （两轮异号 -> 直行）
  *             -> 后退 500mm @ 60mm/s
  *             -> 原地左转 90° @ 30°/s  （两轮同号 -> 反向 -> 原地转）
  *             -> 原地右转 90° @ 30°/s
  *             -> 停车 1s，然后回到第一步无限循环
  *         每步之间停 2s，且都要等剩余位移归零才走下一步
  *         （中途下反向指令会把电机搞死，必须等停稳）。
  *
  *         ★ Car_Motor_Init() 内部用了 osDelay()，而 osDelay() 依赖调度器已运行，
  *           所以必须在本任务里调用，不能放到 main() 的 osKernelStart() 之前
  *           （那样会一直阻塞，等不到调度器启动）。
  *
  *         进度看 g_car_bu_seq_stage / g_car_bu_seq_ret[0..1]，
  *         发送计数看 g_car_bu_tx_count / g_car_bu_tx_fail
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_Bringup(void const *argument)
{
	(void)argument;

	/* 内部含初始化 + 测试序列 + 死循环，永不返回 */
	Car_BuBootSequence();
}

/**
  * @brief  课题三·TOF8 测距任务：配置 -> 开连续测量 -> 循环轮询 -> 上报
  *
  *         流程完全按《tof8通讯协议.docx》「二、流程」：
  *             1) 开机发一次配置帧 0x0316（4x4 + 持续测量）
  *             2) 开连续测距（通道 10，Data[2]=1）
  *             3) 循环取数（通道 12）
  *         本任务不关连续测量：自检模式要的是"一直有数看"，
  *         真要"用完就关"的场合（比如推拉箱动作期间怕干扰）自己调
  *         Tof8_ContinuousSet(slot, 0)。
  *
  *         上报帧（任何 CAN 分析仪/上位机都能直接读）：
  *         ┌─ 0x7F8（距离）──────────── DLC=8 ──────────────────────┐
  *         │ [0] 0xA6 魔数      [1] 序号（循环，在变 = 任务在跑）     │
  *         │ [2][3] 槽0 距离 mm u16 小端   [4][5] 槽1 距离 mm u16    │
  *         │ [6] 槽0 应答帧数低字节        [7] 槽1 应答帧数低字节    │
  *         └──────────────────────────────────────────────────────┘
  *         ┌─ 0x7F7（设备信息）─────── DLC=8 ──────────────────────┐
  *         │ [0] 0xA7 魔数      [1] 槽0 设备号   [2] 槽1 设备号      │
  *         │ [3] 槽0 分辨率(16/64)  [4] 槽1 分辨率                   │
  *         │ [5] 槽0 频率 Hz        [6] 槽1 频率 Hz   [7] 槽数        │
  *         └──────────────────────────────────────────────────────┘
  *
  *         ★ 怎么判故障（这是本模式的全部价值）：
  *            [6][7] 一直 0          -> 请求帧根本没被应答：设备号填错 / 没接线 / 波特率不对
  *            [2][3] = 9999(0x270F)  -> 应答正常，但模块报"无效"（无回波/贴死/超量程）
  *            [3][5] 分辨率/频率为 0 -> 设备号不对（应答被别的设备吃了）
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_Tof8(void const *argument)
{
	uint8_t  i;
	uint8_t  buf[8];
	uint8_t  seq = 0U;
	uint8_t  diag_div = 0U;
	uint8_t  recheck_div = 0U;
	uint16_t d0;
	uint16_t d1;
	uint32_t wake;

	(void)argument;

	Tof8_Init();                        /* 只清内部状态，不发报文 */

	/* 等模块上电稳定：连续模式首次出数可能要 >100ms，这里给 800ms 富余 */
	osDelay(TOF8_BOOT_WAIT_MS);

	/* ---- 1) 配置（开机一次即可）---- */
	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		(void)Tof8_Config(i);
		osDelay(TOF8_CFG_GAP_MS);
	}

	/* ---- 2) 开连续测量 ---- */
	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		(void)Tof8_ContinuousSet(i, 1U);
		osDelay(TOF8_CFG_GAP_MS);
	}

	osDelay(TOF8_FIRST_DATA_TIMEOUT_MS);

	/* ---- 3) 查一次分辨率/频率：这是"设备号对不对"的唯一硬证据 ---- */
	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		(void)Tof8_RequestResolution(i);
		osDelay(TOF8_CFG_GAP_MS);
		(void)Tof8_RequestFrequency(i);
		osDelay(TOF8_CFG_GAP_MS);
	}

	wake = osKernelSysTick();

	for (;;)
	{
		/* ---- 4) 轮询每路距离（通道 12）----
		 *      两路之间留 2ms，让第一路的应答有时间回来再发第二路，
		 *      否则总线上请求挤在一起，拿到的可能全是「旧数据」*/
		for (i = 0U; i < TOF8_DEV_NUM; i++)
		{
			(void)Tof8_RequestDistance(i);
			osDelay(2U);
		}

		/* ---- 5) 每 100ms 上报一帧 0x7F8 ---- */
		if (++diag_div >= (uint8_t)(TOF8_DIAG_PERIOD_MS / TOF8_POLL_PERIOD_MS))
		{
			diag_div = 0U;

			d0 = Tof8_GetDistanceMm(0U);
			d1 = Tof8_GetDistanceMm(1U);        /* 槽位不存在时返回 9999，正好是无效值 */

			buf[0] = 0xA6U;
			buf[1] = seq++;
			buf[2] = (uint8_t)(d0 & 0xFFU);
			buf[3] = (uint8_t)(d0 >> 8);
			buf[4] = (uint8_t)(d1 & 0xFFU);
			buf[5] = (uint8_t)(d1 >> 8);
			buf[6] = (uint8_t)(Tof8_GetRespCount(0U) & 0xFFU);
			buf[7] = (uint8_t)(Tof8_GetRespCount(1U) & 0xFFU);
			(void)Can_SendStd(TOF8_DIAG_ID, buf, 8U);

			/* ---- 6) 每 2s 补查一次设备号/分辨率/频率 ---- */
			if (++recheck_div >= TOF8_RECHECK_PERIOD)
			{
				recheck_div = 0U;

				for (i = 0U; i < TOF8_DEV_NUM; i++)
				{
					(void)Tof8_RequestResolution(i);
					osDelay(TOF8_CFG_GAP_MS);
					(void)Tof8_RequestFrequency(i);
					osDelay(TOF8_CFG_GAP_MS);
				}

				buf[0] = 0xA7U;
				buf[1] = Tof8_GetDev(0U);
				buf[2] = Tof8_GetDev(1U);
				buf[3] = Tof8_GetResolution(0U);
				buf[4] = Tof8_GetResolution(1U);
				buf[5] = Tof8_GetFrequency(0U);
				buf[6] = Tof8_GetFrequency(1U);
				buf[7] = TOF8_DEV_NUM;
				(void)Can_SendStd(TOF8_DIAG_ID2, buf, 8U);
			}
		}

		(void)osDelayUntil(&wake, TOF8_POLL_PERIOD_MS);
	}
}

/**
  * @brief  课题三·飞箱机构调试任务：扫描设备号 -> 配参数 -> 上报状态
  *
  *         ★ 本任务【不主动动电机】。所有运动都由上位机发 0x111 命令触发，
  *           原因：没有极限限位挡块，自动跑序列一旦设备号映射错了，
  *           就是让一个不知道是什么的关节自己转起来 —— 这个风险不能冒。
  *           先人工点动确认，再谈自动化。
  *
  *         上电流程：
  *             1) Fly_Init()            按 flybox.h 的宏填设备号（0xFF = 未指定）
  *             2) Fly_VerifyAxes()      逐个设备号查状态（★ 纯读，不动电机）
  *                                       —— 这是「5 个号是否都在」的唯一证据
  *             3) Fly_ScanDevices()     扫全总线 0..15，看实际有哪几个号在线
  *             4) Fly_SendScanReport()  上报扫描结果（0x7F6）
  *             5) Fly_ConfigAxes()      0x0316(保持原号) -> 轮径 -> 使能 -> 加速度/速度
  *                                      ★ 闸门：应答数 != 需配置轴数时一帧都不发
  *             6) Fly_RequestPositions() 拿绝对位置基准
  *             7) 循环：每 200ms 上报 0x7F5 状态 + 每 400ms 上报 0x7F4 位置
  *
  *         ★ FLY_USE_SN_BIND == 0（默认）：不做 SN 绑定，
  *           因为设备号出厂已定（M1~M5 = 7/8/9/10/11），SN 绑定的
  *           唯一作用就是写设备号，重复写只是多一次把号搞乱的风险。
  *
  *         ★ 怎么看「能不能安全地动」：
  *             g_fly_verify_cnt == 5   5 个设备号全部在线
  *             g_fly_cfg_cnt    == 5   5 个轴都配置成功
  *             各轴 pos_valid   == 1   上电读到了绝对位置（软限位才可信）
  *           对应 0x7F5 的 [7] = 0x55。
  * @param  argument 未使用
  * @retval None（死循环，永不返回）
  */
static void Task_Flybox(void const *argument)
{
	uint8_t  div_state = 0U;
	uint8_t  div_pos   = 0U;
	uint8_t  div_health = 0U;
	uint32_t wake;

	(void)argument;

	Fly_Init();

	/* ★ 第一帧必须尽快发出去：这是「固件到底有没有在跑」的唯一证据。
	 *   如果 CAN 上看不到 0x7F1（魔数 0xB2），就不是配置问题，
	 *   而是固件没跑 / 烧的是旧固件 / CAN 发不出去 —— 那要先解决那个。 */
	Fly_SetBootStage(0U);

	/* ---- 0) TOF 初始化（飞箱的定位靠它，所以比电机还早一点）----
	 * 配置帧 0x0316 + 开连续测量（通道 10），内部含约 900ms 的等待，
	 * 正好当电机的启动等待用。
	 * ★ TOF 一帧都读不到时后面的 TOF 闭环会直接拒绝运动（FLY_TOF_ERR_NO_DATA），
	 *   这是故意的：没有定位手段就不该动。 */
	Fly_SetBootStage(1U);
	Fly_TofInit();

	/* ★ 上电就把 TOF 健康状态倒出来一次。
	 *   意义：如果这里「应答计数 = 0」，那后面的激光靠近必然失败，
	 *         不要再去猜流程逻辑 —— 先解决 TOF 通不通。
	 *   ★ 想看最新状态随时发 0x111 [0x0F]。 */
	Fly_SendTofHealth();

	/* 等电机内部启动完成 */
	Fly_SetBootStage(2U);
	osDelay(1000U);

#if (FLY_USE_SN_BIND != 0)
	/* ---- SN 绑定（0x0313，SN 寻址）----
	 *
	 * ★★★ 这一步【不能省】★★★
	 *   实测（课题二）：电机上电后不绑 SN，只会回 ACK、不转也不报位移。
	 *   0x0313 靠 SN 匹配认领电机，绑完之后电机才认「设备号」这个地址，
	 *   后面 0x0316 / 0x0408 / 0x0413 / 0x0416 才有意义。
	 *
	 *   不绑的现象就是飞箱第一次上电那样：电机不锁死、一帧都不动。
	 *
	 * ★ 绑定填的设备号 = FLY_DEV_*（7/8/9/10/11），
	 *   所以这一步不改变出厂号，只是把「SN -> 设备号」确立下来。
	 */
	Fly_SetBootStage(3U);
	(void)Fly_BindAxes();

	/* ★ 绑定后必须等够：电机要把设备号写进内部存储（Flash），
	 *   写完可能还会自己重启一次。等 1 秒再校验，否则会误判成「没应答」。 */
	Fly_SetBootStage(4U);
	osDelay(1000U);
#endif

	/* ---- 1) 校验 5 个设备号是否都在线（纯读，不动电机）----
	 * 判据：发 0x0408 [dev, 0x25] 查状态，看有几个 dev 回了 0x0409。
	 * ★ 这一步是后面所有运动的安全前提：只有确认 5 个号都在，
	 *   才能确定「发 Data[4]=7 时只有 M1 会动」，而不是 5 台一起动。 */
	Fly_SetBootStage(5U);
	g_fly_verify_cnt = Fly_VerifyAxes(FLY_SCAN_GAP_MS);
	Fly_SendBootStage();

	/* ---- 2) 扫描 + 上报（即使校验没过，也能看到总线上到底有什么）---- */
	Fly_SetBootStage(6U);
	(void)Fly_ScanDevices(FLY_SCAN_GAP_MS);
	g_fly_mapped_cnt = Fly_AutoMapDevices();
	Fly_SendScanReport();

	/* ---- 3) 配置（★ 内部有硬闸门：应答数 != 需配置轴数 时一帧 0x0316 都不发）---- */
	Fly_SetBootStage(7U);
	(void)Fly_ConfigAxes();
	Fly_SendBootStage();

	osDelay(500U);

	/* ---- 4) 拿绝对位置基准（软限位的前提）----
	 * ★ 两步都不能省：
	 *     RequestPositions() 只是「发问」，RefreshPositions() 才是「收答案」。
	 *   少了 Refresh，pos_valid 永远是 0 -> 软限位失效 + 语义动作全部拒绝执行。 */
	Fly_SetBootStage(8U);
	Fly_RequestPositions();
	osDelay(300U);
	(void)Fly_RefreshPositions();

#if (FLY_JOGTEST_ON_BOOT != 0)
	/* ---- 5) 5 轴点动自检（默认关闭，换电机/查方向时才开）---- */
	Fly_SetBootStage(9U);
	Fly_RunJogTest();
	osDelay(500U);
#endif

#if (FLY_RUN_FLOW_ON_BOOT != 0)
	/* ---- 6) ★ 上电自动开始业务流程 ----
	 * ★ 不在这里直接跑，而是「置一个请求」交给下面的主循环。
	 *   理由：主循环里才有循环 / 交替 / 失败停机的逻辑，
	 *   在这里跑一遍会绕开那些逻辑。
	 *
	 * ★★★ M5 归零必须在 SetZeroAll 【之前】★★★
	 *
	 *   SetZeroAll 把「当前姿态」当成 0°。如果上次流程中途停了、
	 *   抓钩还开着（比如 90°），那 90° 就被当成了 0°，
	 *   接下来「抬钩 100°」就变成从 90° 再抬 100° = 190° -> 顶结构。
	 *
	 *   所以先反向转回垂下位置，再置零。
	 *   由 FLY_HOOK_BOOT_HOME 控制（1 = 做）。 */
	Fly_SetBootStage(9U);

#if (FLY_HOOK_BOOT_HOME != 0)
	(void)Fly_HookHomeNow();
	osDelay(200U);
#endif

	/* ★ 现在把当前位置设为零点。
	 *   流程用的是相对位移，本身不需要零点；
	 *   但置零之后软限位才生效，能在出错时提前拦住。
	 *   ★ 前提：上电时机构处于「你觉得的基准姿态」。 */
	Fly_SetZeroAll();
	osDelay(200U);

#if (FLY_BOOT_FLOW == FLY_FLOW_PULL) || (FLY_BOOT_FLOW == FLY_FLOW_PUSH)
	g_fly_flow_which = FLY_BOOT_FLOW;
	g_fly_flow_req   = 1U;
#else
#error "FLY_BOOT_FLOW 必须是 FLY_FLOW_PULL 或 FLY_FLOW_PUSH"
#endif

	osDelay(100U);
#endif

	Fly_SetBootStage(10U);

	wake = osKernelSysTick();

	for (;;)
	{
		/* ---- 业务流程请求（0x111 [0x0D, 1=拉箱 / 2=推箱]）----
		 * ★ 必须在任务里执行：流程内部每步都要等到位。
		 *
		 * ★★★ 每次只跑【一轮】，循环靠外层 for(;;) 重新置请求 ★★★
		 *
		 *   原来是 do{...}while() 在里面一直转，结果有两个毛病：
		 *     ① 其它请求（置零 / 学习抓取距离 / 点动 / TOF 诊断）
		 *        全都被饿死 —— 流程一跑起来就再也收不到命令。
		 *     ② 发 0x111 [0x02] 停止 也没用，因为没人检查它。
		 *   改成每轮返回外层，两个问题一起解决。 */
		if ((g_fly_flow_req != 0U) && (g_fly_flow_running == 0U))
		{
			uint8_t failed;

			g_fly_flow_req = 0U;

			if (g_fly_flow_which == FLY_FLOW_PULL)
			{
				failed = Fly_RunPullOnce();
			}
			else
			{
				failed = Fly_RunPushOnce();
			}

			/* ★ 失败处理按 FLY_FLOW_NEVER_STOP 走：
			 *   1 = 盲走模式，失败也继续下一轮（调试期用）
			 *   0 = 任一步失败就停下，不再自动开始下一轮
			 *
			 * ★ 这里写成普通 if 而不是条件编译拼语句：
			 *   拼接写法在 (NEVER_STOP=0, LOOP=0) 的组合下会留下一个
			 *   没有主体的 else，直接编译失败 —— 这种坑不留。 */
			{
				uint8_t keep_going = 1U;

#if (FLY_FLOW_NEVER_STOP == 0)
				if (failed != 0U) { keep_going = 0U; }
#else
				(void)failed;
#endif

#if (FLY_FLOW_LOOP != 0)
				if (keep_going != 0U)
				{
	#if (FLY_FLOW_LOOP_ALT != 0)
					/* 拉箱 / 推箱 交替：拉完接着推，推完再拉 */
					g_fly_flow_which = (g_fly_flow_which == FLY_FLOW_PULL)
					                 ? FLY_FLOW_PUSH
					                 : FLY_FLOW_PULL;
	#endif
					osDelay(FLY_FLOW_STEP_GAP_MS);
					g_fly_flow_req = 1U;    /* ★ 重新置请求 = 下一轮 */
				}
#else
				(void)keep_going;           /* 不循环：跑一轮就回待机 */
#endif
			}

			osDelay(50U);
		}

		/* --- 分隔：下面是需要被服务到的其它请求 --- */

		/* ---- 学习抓取距离（0x111 [0x0E] 不带参数）----
		 * ★ 必须在任务里执行：要读 TOF，不能在 CAN 接收任务里阻塞。
		 * ★ 前提：人已经手动把平台点到「钩子刚好能勾住箱子」的位置。
		 *   固件只是把此刻的 TOF 读数记下来当停止点。 */
		if (g_fly_learn_req != 0U)
		{
			g_fly_learn_req = 0U;
			(void)Fly_LearnGrabDist();
		}

		/* ---- TOF 健康诊断（0x111 [0x0F]）----
		 * ★ 把两路 TOF 的「设备号 / 应答计数 / 是否有数据 / 当前距离」全倒出来。
		 *   这是判断「激光到底在不在总线上」最直接的一帧。 */
		if (g_fly_tofhealth_req != 0U)
		{
			g_fly_tofhealth_req = 0U;
			Fly_SendTofHealth();
		}

		/* ---- TOF 设备号扫描（0x111 [0x10]）----
		 * ★ 对 0..15 每个设备号发一次取距离请求，看谁应答。
		 *   这是排查「激光填的设备号对不对」的唯一手段 ——
		 *   电机侧的扫描发的是电机通道，TOF8 不认。 */
		if (g_fly_tofscan_req != 0U)
		{
			g_fly_tofscan_req = 0U;
			(void)Tof8_ScanDevices(20U);
			Fly_SendTofScanReport();
		}

		/* ---- M5 抓钩归零（0x111 [0x12]）----
		 * ★ 流程中途停掉后抓钩会停在抬起位置，用这条把转回垂下。
		 *   内部会等到位并置零，所以必须在任务里跑。 */
		if (g_fly_hookhome_req != 0U)
		{
			g_fly_hookhome_req = 0U;
			(void)Fly_HookHomeNow();
			osDelay(200U);
		}

		/* ---- 置零请求（0x111 [0x0A, 轴号]）----
		 * ★ 前提：人已经把机构摆到该轴的零点姿态。
		 *   置零只是「把当前位置记为 0」，不会让机构动。 */
		if (g_fly_zero_req != 0U)
		{
			g_fly_zero_req = 0U;

			if (g_fly_zero_axis == 0xFFU)
			{
				Fly_SetZeroAll();
			}
			else if (g_fly_zero_axis < FLY_AXIS_COUNT)
			{
				(void)Fly_SetZeroAxis((FlyAxis_t)g_fly_zero_axis);
			}
			g_fly_zero_axis = 0xFFU;
		}

		/* ---- 上位机发 0x111 [0x09] 可随时重跑点动自检 ----
		 * ★ 必须在任务里执行，不能在 Task_CanRx 里：
		 *   点动要等到位（内部 osDelay 几百毫秒），
		 *   在接收任务里跑会把整个 CAN 接收链路堵住。 */
		if (g_fly_jogtest_req != 0U)
		{
			g_fly_jogtest_req = 0U;
			Fly_RunJogTest();
			osDelay(500U);
		}

		/* 每 200ms：5 轴设备号 + 校验/配置状态 */
		if (++div_state >= 4U)
		{
			div_state = 0U;
			Fly_SendStateReport();

			/* 同频上报两路 TOF 距离（mm）+ 最近一次闭环结果，
			 * 让你在 CAN 分析仪上直接看到「距离在变」。 */
			Fly_SendTofReport();
		}

		/* ★ 每 1s 上报系统健康（0x7E3 魔数 0xBA / 0xBB）
		 *   堆余量 / 栈水位 / CAN 错误计数 ——
		 *   这是区分「MCU 死了」「堆炸了」「总线挂了」「电机堵了」的关键。 */
		if (++div_health >= 20U)
		{
			div_health = 0U;
			Fly_SendHealthReport();
		}

		/* ---- 皮带同步点动（0x111 [0x0C, 方向, 距离L, 距离H]）----
		 * ★ 验证「两条皮带能不能一起动」。和逐轴点动不同，
		 *   这里两帧位置指令紧挨着发，中间不留延时。 */
		if (g_fly_beltjog_req != 0U)
		{
			g_fly_beltjog_req = 0U;
			(void)Fly_BeltBothJog((int32_t)(int16_t)g_fly_beltjog_mm * 10);
			osDelay(300U);
		}

		/* ---- TOF 联动诊断（0x111 [0x0B, 轴, 步长L, 步长H, 步数]）----
		 * 让某个轴慢走几步，每步报一次两路 TOF，用来确定
		 * 「哪个传感器在测哪个轴的运动方向」。
		 * ★ 必须在任务里跑：内部要等到位。 */
		if (g_fly_tofprobe_req != 0U)
		{
			g_fly_tofprobe_req = 0U;
			(void)Fly_TofProbe(g_fly_tofprobe_belt,
			                   g_fly_tofprobe_step,
			                   g_fly_tofprobe_cnt);
			osDelay(300U);
		}

		/* 每 400ms：轮流上报一个轴的位置 / 剩余位移 / 反馈帧数 */
		if (++div_pos >= 8U)
		{
			div_pos = 0U;
			Fly_SendPosReport();
		}

		(void)osDelayUntil(&wake, 50U);
	}
}

/**
  * @brief  CAN 收到帧入队后的通知（在中断里调用，只做唤醒任务）
  */
static void OnCanRx(void)
{
	if (TaskCanRxHandle != NULL)
	{
		(void)osSignalSet(TaskCanRxHandle, SIG_CAN_RX);
	}
}

/* =========================================================================
 *          ★★★ 死机取证钩子（2026-10-30 新增）★★★
 *
 *  ★ 为什么必须加：
 *    现场出现「往回走到一半整个系统不动了」，但 CAN 上什么都看不到 ——
 *    因为原来 configCHECK_FOR_STACK_OVERFLOW = 0，栈溢出是完全隐形的。
 *    溢出的任务会踩坏相邻任务的数据，最终表现就是「跑到某步突然全停」，
 *    而且怎么查代码都查不出来。
 *
 *  ★ 现在两个钩子都会：
 *      1) 发一帧 0x7EF（魔数 0xEE），把「是谁挂了 / 还剩多少栈」发到总线上
 *      2) 停在那里死循环，不再破坏其它数据
 *
 *  ★ 判读：看到 0x7EF 就是【MCU 侧问题】；
 *         看不到 0x7EF 但动作也停了，那是【电机侧问题】（堵转/保护）。
 *    这一条就能把「软件死机」和「机械卡死」区分开，不用再猜。
 * ========================================================================= */

/**
  * @brief  死机上报：0x7EF [0xEE, 原因, 任务名前6字节]
  * @param  why  1 = 栈溢出，2 = 堆分配失败
  * @note   钩子是在异常上下文里跑的，这里只做「发一帧 + 死循环」，
  *         不调用任何 FreeRTOS API，避免二次崩溃。
  */
static void PanicReport(uint8_t why, const char *name)
{
	uint8_t buf[8];
	uint8_t i;

	buf[0] = 0xEEU;
	buf[1] = why;

	for (i = 0U; i < 6U; i++)
	{
		buf[2U + i] = (name != NULL) ? (uint8_t)name[i] : 0U;
		if ((name != NULL) && (name[i] == '\0')) { break; }
	}

	(void)Can_SendStd(0x7EFU, buf, 8U);
}

/**
  * @brief  FreeRTOS 栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW = 2 时会被调用）
  * @param  xTask       溢出的任务句柄
  * @param  pcTaskName  任务名
  * @note   正常运行时【绝不会】进这里。一旦进来，说明某个任务栈给小了。
  */
void vApplicationStackOverflowHook(void *xTask, char *pcTaskName)
{
	(void)xTask;

	PanicReport(1U, pcTaskName);

	/* 死在这里：让 0x7FE 心跳消失，从外部也能看出 MCU 挂了 */
	for (;;)
	{
		/* 空转，不喂任何东西 */
	}
}

/**
  * @brief  FreeRTOS 堆分配失败钩子（configUSE_MALLOC_FAILED_HOOK = 1）
  * @note   任务/队列创建时堆不够会进这里。
  *         典型症状：某个任务根本没跑起来，但编译一切正常。
  */
void vApplicationMallocFailedHook(void)
{
	PanicReport(2U, NULL);

	for (;;)
	{
	}
}

/* CMSIS-RTOS v1 线程定义：osThreadDef(名字, 函数, 优先级, 实例数, 栈大小[单位:字]) */
osThreadDef(Task_Led,  Task_Led,  osPriorityNormal, 0, 256);
osThreadDef(Task_CanTxTest, Task_CanTxTest, osPriorityNormal, 0, 256);
osThreadDef(Task_Gyro,  Task_Gyro,  osPriorityNormal, 0, 256);
osThreadDef(Task_SpiTest, Task_SpiTest, osPriorityBelowNormal, 0, 256);
/* ★ Task_Angle 从 AboveNormal 降为 BelowNormal：
 *   它要 1ms 严格周期，一旦 SPI 读不动（超时/没接陀螺），osDelayUntil 会立刻返回，
 *   于是在整个 1ms 里反复空转，把普通优先级任务（Task_Bringup 的电机序列）饿死。
 *   降到 BelowNormal 后，即使它空转，调度器也会把 CPU 分给更高优先级的任务。
 *   （SPI_TIMEOUT_MS 已同时从 100 降到 2，双重保险） */
osThreadDef(Task_Angle, Task_Angle, osPriorityBelowNormal, 0, 384);
osThreadDef(Task_CanTx, Task_CanTx, osPriorityNormal, 0, 256);
osThreadDef(Task_CanRx, Task_CanRx, osPriorityNormal, 0, 256);
osThreadDef(Task_Motor, Task_Motor, osPriorityNormal, 0, 256);
osThreadDef(Task_CanHeartbeat, Task_CanHeartbeat, osPriorityAboveNormal, 0, 256);
/* ★ Task_Bringup 提到 AboveNormal：脱机主控一旦被饿死，车就完全不动，
 *   必须保证它拿到 CPU 把初始化帧和运动帧发出去 */
osThreadDef(Task_Bringup, Task_Bringup, osPriorityAboveNormal, 0, 320);
/* 课题三·TOF8 测距：Normal 优先级即可，它只是每 10ms 发一帧请求 */
osThreadDef(Task_Tof8, Task_Tof8, osPriorityNormal, 0, 320);
/* 课题三·飞箱：AboveNormal —— 它要在等待到位的循环里保持节奇准，
 * 且一旦被饿死，运动中的轴就没人确认到位了。
 * ★ 栈从 384 提到 512 字（2KB）：这条线程的调用链最深
 *   （Fly_RunJogTest -> Fly_MoveRelEx -> Fly_SendFrame -> Can_SendStd
 *     -> HAL_CAN_AddTxMessage 里有 CAN_TxHeaderTypeDef 等局部变量），
 *   再加上 Fly_TofInit 那一路，1.5KB 附近有溢出风险。
 *   栈溢出在 FreeRTOS 里表现为 HardFault 或莫名卡死，
 *   现象就是「上电什么都不动、灯也不闪」—— 和最开始的症状一样。 */
/* ★ 栈从 512 提到 768：
 *   业务流程的调用链变深了（流程 -> 靠近 -> TOF 闭环 -> 增量 -> 运动 -> CAN 发帧），
 *   而每次 CAN 发帧都要在栈上放 CAN_TxHeaderTypeDef + 数据缓冲。
 *   512 字（2KB）在调试期还够，但已经贴着边界 —— 栈溢出会直接 HardFault，
 *   表现就是「跑到某一步突然全停、CAN 上一片死寂」，很难和逻辑 bug 区分。
 *   多给 1KB 换取不用怀疑这一项。 */
osThreadDef(Task_Flybox, Task_Flybox, osPriorityAboveNormal, 0, 768);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  程序入口
  * @param  None
  * @retval int（实际不会返回）
  */
int main(void)
{
	/* 1. HAL 初始化（含 1ms SysTick 时基，HAL_Delay 依赖它） */
	HAL_Init();

	/* 2. 系统时钟配置：HSE 8MHz 晶振 x PLL9 -> 72MHz */
	SystemClock_Config();

	/* 3. 按当前测试模式初始化对应外设
	 *    此时调度器尚未启动（osKernelStart() 在后面），初始化里的 HAL_Delay 可以正常使用 */
#if (CURRENT_TEST_MODE == TEST_MODE_LED)
	Led_Init();
#elif (CURRENT_TEST_MODE == TEST_MODE_CAN)
	Can_Init();
#elif (CURRENT_TEST_MODE == TEST_MODE_GYRO)
	XV7001bb_Init();                    /* 内部：Spi_Init() + HAL_Delay(100) 上电等待 + 读 0x04 验证 */
#elif (CURRENT_TEST_MODE == TEST_MODE_SPI)
	Spi_Init();                         /* 只初始化 SPI2（GPIO + 外设）；不发任何数据、不碰芯片 */
#elif (CURRENT_TEST_MODE == TEST_MODE_APP)
	XV7001bb_Init();                    /* SPI2 + 等芯片上电稳定（100ms） */
	Can_Init();                         /* CAN1 1Mbps + 接收中断 */
	Angle_Init();                       /* 必须在 XV7001bb_Init 之后：先读一次状态/温度 */
	Can_SetRxCallback(OnCanRx);         /* 收到帧后唤醒 Task_CanRx */
#elif (CURRENT_TEST_MODE == TEST_MODE_MOTOR)
	Can_Init();                         /* CAN1 1Mbps + 接收中断 */
	Motor_Init();                       /* 填配置、清状态；不发报文 */
	Can_SetRxCallback(OnCanRx);         /* 收到帧后唤醒 Task_CanRx（电机反馈也走这里） */
#elif ((CURRENT_TEST_MODE == TEST_MODE_BRINGUP) || (CURRENT_TEST_MODE == TEST_MODE_HOST))
	/* 脱机自检(BRINGUP) 和 上位机控车(HOST) 的外设完全一样，
	 * 差别只在「要不要创建 Task_Bringup 去自己发运动帧」，见下面的任务创建段。
	 * ★ 课题四/五（往返测试、方形测试）也走 TEST_MODE_BRINGUP 这条路，
	 *   靠 car_bringup.h 的 CAR_BU_RUN_MODE 选择跑哪段序列。 */
	XV7001bb_Init();                    /* 陀螺仪：SPI2 + 上电等待（100ms）*/
	Can_Init();                         /* CAN1 1Mbps + 接收中断 */
	Angle_Init();                       /* 必须在 XV7001bb_Init 之后：先读一次状态/温度 */
	Motor_Init();                       /* 只为让 Motor_OnFrame 能解析反馈（本模式不发初始化序列）*/
	Can_SetRxCallback(OnCanRx);         /* 收到帧后唤醒 Task_CanRx */
#elif (CURRENT_TEST_MODE == TEST_MODE_TOF8)
	/* 课题三·TOF8 测距自检：只开 CAN，不碰陀螺仪、不碰电机。
	 * ★ 为什么刻意不初始化电机：本模式要回答的问题是
	 *   「TOF8 到底能不能测到距离」，这跟电机、陀螺仪一点关系都没有。
	 *   一旦同时跑，电机反馈帧会淹没 0x7F8 的排查视野。 */
	Can_Init();                         /* CAN1 1Mbps + 接收中断 */
	Can_SetRxCallback(OnCanRx);         /* 收到帧后唤醒 Task_CanRx */
#elif (CURRENT_TEST_MODE == TEST_MODE_FLYBOX)
	/* 课题三·飞箱机构调试：只开 CAN。
	 * ★ 不初始化陀螺仪、不初始化 TOF8 —— 本阶段只解决「5 台电机是什么号」
	 *   和「每个轴能不能安全地点动」，多开一路就多一份排查干扰。
	 * ★ Motor_Init() 必须调：Motor_OnFrame() 靠它清缓存 + 填配置表，
	 *   而 0x0420 剩余位移（到位判据的唯一来源）就在里面解析。 */
	Can_Init();
	Motor_Init();
	Can_SetRxCallback(OnCanRx);
#endif

	/* 3.5 LED 心跳指示灯：这里只做 GPIO 初始化，
	 *     真正每 500ms 的翻转由下方统一创建的 Task_Led 完成（灯闪 = 程序在运行） */
#if (CURRENT_TEST_MODE != TEST_MODE_LED)
	Led_Init();                         /* PB9 推挽输出（初始输出高 = 熄灭） */
#endif

	/* 4. 创建任务：Task_Led 在所有模式下都创建（500ms 心跳灯），
	 *    再创建当前测试模式对应的功能任务 */
#if (CURRENT_TEST_MODE == TEST_MODE_LED)
	TaskLedHandle  = osThreadCreate(osThread(Task_Led),  NULL);
#elif (CURRENT_TEST_MODE == TEST_MODE_CAN)
	TaskLedHandle  = osThreadCreate(osThread(Task_Led),  NULL);   /* 心跳灯 */
	TaskCanHandle  = osThreadCreate(osThread(Task_CanTxTest), NULL);
#elif (CURRENT_TEST_MODE == TEST_MODE_GYRO)
	TaskLedHandle  = osThreadCreate(osThread(Task_Led),  NULL);   /* 心跳灯 */
	TaskGyroHandle = osThreadCreate(osThread(Task_Gyro), NULL);
#elif (CURRENT_TEST_MODE == TEST_MODE_SPI)
	TaskLedHandle  = osThreadCreate(osThread(Task_Led),  NULL);       /* 心跳灯 */
	TaskSpiHandle  = osThreadCreate(osThread(Task_SpiTest), NULL);    /* 纯 SPI 波形测试（无延时连续发） */
#elif (CURRENT_TEST_MODE == TEST_MODE_APP)
	TaskLedHandle   = osThreadCreate(osThread(Task_Led),   NULL);     /* 心跳灯 */
	TaskCanRxHandle = osThreadCreate(osThread(Task_CanRx), NULL);
	TaskCanTxHandle = osThreadCreate(osThread(Task_CanTx), NULL);
	TaskAngleHandle = osThreadCreate(osThread(Task_Angle), NULL);
#elif (CURRENT_TEST_MODE == TEST_MODE_MOTOR)
	TaskLedHandle   = osThreadCreate(osThread(Task_Led),   NULL);     /* 心跳灯 */
	TaskCanRxHandle = osThreadCreate(osThread(Task_CanRx), NULL);
	TaskMotorHandle = osThreadCreate(osThread(Task_Motor), NULL);     /* 电机初始化状态机 */
#elif (CURRENT_TEST_MODE == TEST_MODE_BRINGUP)
	/* ---- 脱机自检：固件自己跑完整序列 ---- */
	TaskLedHandle   = osThreadCreate(osThread(Task_Led),     NULL);   /* 心跳灯 */
	TaskCanRxHandle = osThreadCreate(osThread(Task_CanRx),   NULL);   /* 解析电机反馈，Watch 可见 */
	TaskHeartbeatHandle = osThreadCreate(osThread(Task_CanHeartbeat), NULL); /* ★ 0x7FE 心跳，证明 STM32 在发帧 */
	TaskAngleHandle = osThreadCreate(osThread(Task_Angle),   NULL);   /* 陀螺仪：1ms 采样 / 10ms 输出 */
	TaskCanTxHandle = osThreadCreate(osThread(Task_CanTx),   NULL);   /* 发 0x101/0x102 给上位机 */
	TaskBringupHandle = osThreadCreate(osThread(Task_Bringup), NULL); /* 绑定 + 点动测试 */
#elif (CURRENT_TEST_MODE == TEST_MODE_HOST)
	/* ---- 上位机控车：★ 故意不创建 Task_Bringup ----
	 * Task_Bringup 一跑起来就会自己发运动帧，把上位机的指令覆盖掉。
	 * 本模式固件只做三件事：心跳、陀螺仪上报、被动解析电机反馈（给 Watch 看）。*/
	TaskLedHandle   = osThreadCreate(osThread(Task_Led),     NULL);   /* 心跳灯 */
	TaskCanRxHandle = osThreadCreate(osThread(Task_CanRx),   NULL);   /* 只解析反馈，不发任何帧 */
	TaskHeartbeatHandle = osThreadCreate(osThread(Task_CanHeartbeat), NULL); /* 0x7FE 心跳（也可当"固件没抢电机"的证据）*/
	TaskAngleHandle = osThreadCreate(osThread(Task_Angle),   NULL);   /* 陀螺仪：1ms 采样 / 10ms 输出 */
	TaskCanTxHandle = osThreadCreate(osThread(Task_CanTx),   NULL);   /* 发 0x101/0x102 给上位机 */
#elif (CURRENT_TEST_MODE == TEST_MODE_TOF8)
	/* ---- 课题三·TOF8 测距自检：只跑这一条链路 ---- */
	TaskLedHandle   = osThreadCreate(osThread(Task_Led),     NULL);   /* 心跳灯：灯闪 = 固件在跑 */
	TaskCanRxHandle = osThreadCreate(osThread(Task_CanRx),   NULL);   /* 解析 0x0409/0x0793/0x0794 */
	TaskTof8Handle  = osThreadCreate(osThread(Task_Tof8),    NULL);   /* 配置 + 轮询 + 0x7F8 上报 */
#elif (CURRENT_TEST_MODE == TEST_MODE_FLYBOX)
	/* ---- 课题三·飞箱机构调试 ---- */
	TaskLedHandle   = osThreadCreate(osThread(Task_Led),     NULL);   /* 心跳灯 */
	TaskCanRxHandle = osThreadCreate(osThread(Task_CanRx),   NULL);   /* 0x111 命令 + 电机反馈 */
	TaskFlyboxHandle = osThreadCreate(osThread(Task_Flybox), NULL);   /* 扫描 + 配置 + 上报 */
	/* ★ 独立心跳（0x7FE，500ms）：
	 *   它和 flybox 模块完全无关，只证明「STM32 在跑并且能发 CAN」。
	 *   如果连 0x7FE 都看不到，问题就不在飞箱逻辑里，而在固件/收发器/接线。 */
	TaskHeartbeatHandle = osThreadCreate(osThread(Task_CanHeartbeat), NULL);
#endif

	/* 5. 启动 FreeRTOS 调度器：任务从此刻开始运行。
	 *    任务里延时必须用 osDelay（让出 CPU）；调度器启动后严禁再调用 HAL_Delay */
	osKernelStart();

	while (1)
	{
		/* 只有任务创建失败或调度器启动失败才会走到这里 */
	}
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  System Clock Configuration
  *         HSE 8 MHz crystal (X1) -> PLL x9 -> SYSCLK 72 MHz
  *           SYSCLK = 72 MHz (PLL, HSE x 9)
  *           HCLK   = 72 MHz (AHB / 1)
  *           PCLK1  = 36 MHz (APB1 / 2, 36 MHz is the maximum for APB1)
  *           PCLK2  = 72 MHz (APB2 / 1)
  *         2 wait states are required for the Flash above 48 MHz.
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/* Turn on the HSE (8 MHz crystal on OSC_IN/OSC_OUT) and the PLL */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/* Select the PLL output as system clock and set the bus prescalers */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
	                              RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}

	/* 时基(SysTick)再校准：按最终 72 MHz 重新装载 1 ms 时基，并保持中断优先级
	 * ★ 说明：HAL_InitTick() 在 STM32F1 HAL 里配置的仍然是 SysTick（不会切换到 TIM！），
	 *   作用只是"时钟切换后按新时钟重新装载 SysTick"。HAL_RCC_ClockConfig() 内部已调用过
	 *   一次，这里按需求追加一次幂等调用，无副作用。
	 *   本工程 bsp.c 的 SysTick_Handler 同时调用 HAL_IncTick() + osSystickHandler()，
	 *   HAL 时基与 FreeRTOS 节拍共用 SysTick（configTICK_RATE_HZ=1000，即 1 ms = 1 tick） */
	(void)HAL_InitTick(TICK_INT_PRIORITY);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
	/* Disable the interrupts and stay here */
	__disable_irq();
	while (1)
	{
	}
}
