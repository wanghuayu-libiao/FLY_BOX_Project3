/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN1 驱动实现（HAL 版，STM32F103C8T6，PA11=CAN_RX，PA12=CAN_TX，1Mbps）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "can.h"

/* Global variables ----------------------------------------------------------*/
/* CAN 接收结果（调试器 Watch 观察用；在中断回调里更新，加 volatile 防止被优化） */
volatile uint8_t  g_can_rx_data[8] = {0};   /* 最近一次收到报文的 8 个数据字节（完整报文载荷） */
volatile uint32_t g_can_rx_id   = 0;        /* 最近一次收到报文的标准帧 ID（StdId） */
volatile uint32_t g_can_rx_drop = 0;        /* 接收队列满而丢弃的帧数 */
volatile uint32_t g_can_tx_flush = 0;       /* 邮箱全满时被丢弃的旧帧次数（总线无应答时会增加） */

/* Private define ------------------------------------------------------------*/
/* 发送使用的标准帧 ID */
#define CAN_TX_STD_ID   0x123U

/* 接收队列深度（中断里入队、任务里出队） */
#define CAN_RX_QUEUE_LEN    16U

/* Private variables ---------------------------------------------------------*/
static CAN_HandleTypeDef hcan;

static CanFrame_t s_rx_queue[CAN_RX_QUEUE_LEN];
static volatile uint8_t s_rx_head = 0U;     /* 只由中断写 */
static volatile uint8_t s_rx_tail = 0U;     /* 只由任务写 */
static void (*s_rx_callback)(void) = NULL;

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化 CAN1（正常模式，波特率 1Mbps）
  *         引脚分配：
  *             PA11 = CAN_RX（输入，内部上拉）
  *             PA12 = CAN_TX（复用推挽输出）
  *         时钟：CAN1 挂在 APB1（36 MHz）上
  *             波特率 = PCLK1 / (Prescaler x (1 + BS1 + BS2))
  *                    = 36MHz  / (2 x (1 + 15 + 2))
  *                    = 36MHz  / 36 = 1Mbps
  *         过滤器：接收所有标准帧，映射到 FIFO0
  * @param  None
  * @retval None
  */
void Can_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	CAN_FilterTypeDef sFilterConfig = {0};

	/* 1. 开启 CAN1 与 GPIOA 时钟 */
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* 2. PA11(CAN_RX)：输入（CAN 总线空闲为隐性高电平，启用内部上拉） */
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* 3. PA12(CAN_TX)：复用推挽输出 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* 4. CAN1 参数配置：1Mbps
	 *    36MHz / (2 x (1 + 15 + 2)) = 1MHz
	 *    采样点 = (1 + BS1) / (1 + BS1 + BS2) = 16/18 ≈ 88.9%（接近推荐的 87.5%）
	 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 2U;               /* 36MHz / 2 = 18MHz 时间量子时钟 */
	hcan.Init.Mode = CAN_MODE_NORMAL;       /* 正常模式（需要外部 CAN 收发器） */
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;          /* 总线关闭后自动恢复，连续发送时不会永久下线 */
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = ENABLE;  /* 发送失败自动重传 */
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	(void)HAL_CAN_Init(&hcan);

	/* 5. 过滤器配置：ID 与掩码全为 0 => 不关心任何 ID 位，接收所有帧，映射到 FIFO0 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000U;
	sFilterConfig.FilterIdLow = 0x0000U;
	sFilterConfig.FilterMaskIdHigh = 0x0000U;
	sFilterConfig.FilterMaskIdLow = 0x0000U;
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;/* 单 CAN 器件不使用该参数 */
	(void)HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

	/* 6. 启动 CAN（必须调用，否则外设停留在初始化模式，无法收发） */
	(void)HAL_CAN_Start(&hcan);

	/* 7. 使能 FIFO0 "报文挂起"中断（收到新帧后置中断标志并触发中断） */
	(void)HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* 8. 打开 NVIC 里 CAN1_RX0 的中断通道
	 *    ★ 必须！只使能外设中断而 NVIC 没开，回调永远不会执行。
	 *    优先级数值取 6（>= FreeRTOS 的 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5），
	 *    以后想在回调里调用 FreeRTOS 的 ...FromISR API 也不会踩优先级限制。
	 *    ★ F1 命名注意：RX0 向量与 USB 低优先级中断共用，
	 *    枚举名是 USB_LP_CAN1_RX0_IRQn（F1 头文件里没有 CAN1_RX0_IRQn！） */
	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 6U, 0U);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

/**
  * @brief  发送 1 个字节（标准帧，ID = 0x123，DLC = 1）
  * @param  data 要发送的字节
  * @retval 0 表示成功，1 表示失败（无空闲邮箱等）
  */
uint8_t Can_SendByte(uint8_t data)
{
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox = 0;

	/* 标准帧头配置 */
	txHeader.StdId = CAN_TX_STD_ID;
	txHeader.ExtId = 0U;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = 1U;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, &data, &txMailbox) != HAL_OK)
	{
		return 1U;                          /* 失败 */
	}

	return 0U;                              /* 成功 */
}

/**
  * @brief  发送一帧标准数据帧
  *         邮箱全满（通常是总线上无人应答，帧卡在邮箱里重发）时，先放弃旧帧再发，
  *         保证总线恢复后发出去的是最新数据，而不是一堆过期的角度
  * @param  id   标准帧 ID（11 位）
  * @param  data 数据指针
  * @param  dlc  数据长度 0~8
  * @retval 0 成功，1 失败
  */
uint8_t Can_SendStd(uint16_t id, const uint8_t *data, uint8_t dlc)
{
	CAN_TxHeaderTypeDef txHeader;
	uint32_t txMailbox = 0;

	if (dlc > 8U)
	{
		dlc = 8U;
	}

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
	{
		/* ★★ 不能直接 AbortTxRequest 把 3 个邮箱全清空！
		 *    运动帧是「加速度 -> 速度 -> 位置」连着发 3~6 帧的突发，
		 *    1Mbps 下一帧 8 字节约 110μs，连发时第 4 帧必然看到 0 个空邮箱。
		 *    原来的做法是把还没发出去的帧**全部中止**，反复中止会累积 CAN 发送
		 *    错误计数，最终把控制器打进 bus-off —— 表现就是 STM32 的 0x7FE 心跳、
		 *    0x0101 角度帧、0x0414/15/16 运动帧全部从总线上消失，
		 *    只剩电机自己在发反馈帧。
		 *    正确做法：短等待空邮箱（正常几十微秒就腾出来），超时才丢弃。 */
		uint32_t start = HAL_GetTick();

		while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
		{
			if ((HAL_GetTick() - start) >= CAN_TX_WAIT_MS)
			{
				(void)HAL_CAN_AbortTxRequest(&hcan,
				                             CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
				g_can_tx_flush++;
				break;
			}
		}
	}

	txHeader.StdId = id;
	txHeader.ExtId = 0U;
	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.DLC = dlc;
	txHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, (uint8_t *)data, &txMailbox) != HAL_OK)
	{
		return 1U;
	}

	return 0U;
}

/**
  * @brief  ★ 取 CAN 错误状态寄存器 ESR 原值（供系统健康诊断帧 0xBB 使用）
  *
  *   ESR 位定义（参考手册）：
  *     bit4      EWGF  总线关闭（bus-off）标志
  *     bit16..23 TEC   发送错误计数（★ 到 255 就进 bus-off）
  *     bit24..31 REC   接收错误计数
  *
  *   ★ 为什么要暴露它：
  *     现场「跑到一半全停」有一种原因是 CAN 总线进了 bus-off，
  *     这时 STM32 的帧全部发不出去，看起来就像「程序死了」。
  *     有了 TEC/REC 就能一眼分清是软件问题还是总线问题。
  */
uint32_t Can_GetEsr(void)
{
	return hcan.Instance->ESR;
}

/** @brief 取「邮箱全满时丢弃旧帧」的累计次数 */
uint32_t Can_GetTxFlushCount(void)
{
	return g_can_tx_flush;
}

/**
  * @brief  从接收队列取出一帧（在任务里调用）
  * @retval 1 取到一帧，0 队列为空
  */
uint8_t Can_RxPop(CanFrame_t *frame)
{
	uint8_t tail = s_rx_tail;

	if (s_rx_head == tail)
	{
		return 0U;
	}

	*frame = s_rx_queue[tail];
	s_rx_tail = (uint8_t)((tail + 1U) % CAN_RX_QUEUE_LEN);

	return 1U;
}

/**
  * @brief  设置接收通知回调：每收到一帧入队后在中断上下文调用（用来唤醒接收任务）
  */
void Can_SetRxCallback(void (*callback)(void))
{
	s_rx_callback = callback;
}

/* Interrupt handlers & callbacks --------------------------------------------*/

/**
  * @brief  CAN1 RX0 中断服务函数（FIFO0 收到新报文时由硬件触发）
  *         ★ 函数名必须是 USB_LP_CAN1_RX0_IRQHandler：
  *           STM32F1 把该向量与 USB 低优先级中断共用，芯片头文件的枚举名是
  *           USB_LP_CAN1_RX0_IRQn，启动文件 startup_stm32f103xb.c 里的弱函数同名。
  *           若写成 CAN1_RX0_IRQHandler：编译链接都能过，但中断永远进不来！
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan);              /* 交给 HAL 统一分发到下面的回调 */
}

/**
  * @brief  CAN FIFO0 收到新报文的回调（HAL 弱函数重写）
  *         HAL_CAN_IRQHandler() 检测到 RX_FIFO0_MSG_PENDING 标志后调用本函数
  * @param  hcan CAN 句柄（形参遮蔽本文件顶部的 static hcan，HAL 惯例命名，无副作用）
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	/* 把报文从 FIFO0 取走（必须取，否则 FIFO 满后该中断不再触发） */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
	{
		return;
	}

	/* 存入全局变量，方便调试器 Watch 观察 */
	g_can_rx_id = rx_header.StdId;          /* 保存 ID */

	/* 循环复制 8 个字节数据到全局数组（完整报文载荷） */
	for (uint8_t i = 0; i < 8U; i++)
	{
		g_can_rx_data[i] = rx_data[i];
	}

	/* 标准数据帧入队供任务处理 */
	if ((rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA))
	{
		uint8_t head = s_rx_head;
		uint8_t next = (uint8_t)((head + 1U) % CAN_RX_QUEUE_LEN);

		if (next == s_rx_tail)
		{
			g_can_rx_drop++;                /* 队列满：丢弃这一帧 */
		}
		else
		{
			s_rx_queue[head].id = (uint16_t)rx_header.StdId;
			s_rx_queue[head].dlc = (uint8_t)rx_header.DLC;
			for (uint8_t i = 0; i < 8U; i++)
			{
				s_rx_queue[head].data[i] = rx_data[i];
			}
			s_rx_head = next;

			if (s_rx_callback != NULL)
			{
				s_rx_callback();
			}
		}
	}
}
