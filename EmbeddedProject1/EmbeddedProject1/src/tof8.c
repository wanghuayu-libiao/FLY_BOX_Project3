/**
  ******************************************************************************
  * @file    tof8.c
  * @brief   课题三·TOF8 激光测距模块 CAN 驱动（协议见《tof8通讯协议.docx》）
  *
  *  ★★★ 为什么这个文件里有一堆「只认自己的设备号」的判断 ★★★
  *
  *    TOF8 用的是 0x0408 / 0x0409 / 0x0316 三个 ID，
  *    这三个 ID 在电机协议 V1.1 里分别对应「配轮径+查询 / 查询应答 / 参数配置」。
  *    两者共用一根总线时，靠报文里的 **设备号** 区分，所以：
  *
  *      1) 发送方向没风险 —— 请求帧 Data[0]=设备号，只有目标设备会应答。
  *      2) 接收方向有风险 —— 总线上所有设备都能听到，必须靠设备号筛。
  *         而且电机模块的 0x0409 解析里 `dev = Data[0] & 0x0F`，
  *         只要 TOF8 设备号 & 0x0F 落进 0/1，就会污染左/右轮的位置缓存。
  *
  *    所以本驱动提供 Tof8_OnFrame()，调用方必须在 Motor_OnFrame() 之前调用，
  *    由本函数「认领」属于自己的帧，剩下的才交给电机模块。
  *
  *  ★★★ 通道号 8 的坑 ★★★
  *     TOF8_CH_RESET_VL53 = 8，而 MOTOR_QUERY_POS = 0x08 也是 8。
  *     如果不认领，一次复位应答就会被电机模块当成「当前位置」写进缓存。
  *     这是本驱动必须在 Motor_OnFrame() 之前调用的另一个原因。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "can.h"
#include "car_cfg.h"
#include "tof8.h"

/* =========================================================================
 *                      设备号合法性自检（编译期）
 * ========================================================================= */

#if (TOF8_DEV_NUM < 1U) || (TOF8_DEV_NUM > TOF8_MAX_SLOTS)
#error "TOF8_DEV_NUM 必须在 1..TOF8_MAX_SLOTS 之间"
#endif

#if (TOF8_DEV0 == CAR_LEFT_DEV) || (TOF8_DEV0 == CAR_RIGHT_DEV)
#error "TOF8_DEV0 与电机设备号冲突：TOF8 和电机共用 0x0408/0x0409/0x0316，设备号必须不同"
#endif

#if (TOF8_DEV_NUM >= 2U)
  #if (TOF8_DEV1 == CAR_LEFT_DEV) || (TOF8_DEV1 == CAR_RIGHT_DEV)
  #error "TOF8_DEV1 与电机设备号冲突：设备号必须不同于 CAR_LEFT_DEV / CAR_RIGHT_DEV"
  #endif
  #if (TOF8_DEV1 == TOF8_DEV0)
  #error "TOF8_DEV0 与 TOF8_DEV1 不能相同"
  #endif
#endif

/* Private define ------------------------------------------------------------*/

/* 请求 -> 应答之间的最少留白。
 * 模块是「收到请求才回」，回得很快（< 2 ms），但 FreeRTOS 里任务切换最细 1 tick。
 * 留 3 ms 足够覆盖一次应答，同时不会让取数太慢。*/
#define TOF8_REQ_GAP_MS             3U

/* Private types -------------------------------------------------------------*/

/* 单路 TOF8 的运行状态 */
typedef struct
{
	uint16_t dist_raw;          /* 最近一次原始距离，0.1mm（未做无效替换）*/
	uint16_t dist_mm;           /* 最近一次有效距离 mm；无效 = TOF8_DIST_MM_INVALID */
	uint8_t  has_data;          /* 收到过至少一次有效距离 */
	uint8_t  fresh;             /* 最近一帧应答的「新数据标志」*/
	uint8_t  resolution;        /* 分辨率 16/64，0 = 未查到 */
	uint8_t  frequency;         /* 频率 Hz，0 = 未查到 */
	uint32_t resp_count;        /* 收到的 0x0409 应答帧数 */
	uint32_t req_count;         /* 发出的请求帧数 */
} Tof8Slot_t;

/* Private variables ---------------------------------------------------------*/

/* 槽位 -> 物理设备号 映射表（顺序来自 car_cfg.h）*/
static const uint8_t s_dev_list[TOF8_DEV_NUM] =
{
	TOF8_DEV0,
#if (TOF8_DEV_NUM >= 2U)
	TOF8_DEV1,
#endif
#if (TOF8_DEV_NUM >= 3U)
	TOF8_DEV2,
#endif
#if (TOF8_DEV_NUM >= 4U)
	TOF8_DEV3,
#endif
};

static Tof8Slot_t s_slot[TOF8_DEV_NUM];

/* =========================================================================
 *        ★ 设备号扫描缓存（2026-10-30 新增）
 *
 *  ★ 为什么不复用 s_slot：
 *    s_slot 只有 TOF8_DEV_NUM 个槽位，而且按 s_dev_list 里【配置好的】
 *    设备号索引。扫描的目的恰恰是「不知道设备号是多少」，
 *    所以需要一张按「任意设备号 0..15」索引的表。
 *
 *  ★ 扫描期间的帧捕获：
 *    Tof8_OnFrame() 开头会按 Tof8_FindSlot() 过滤，设备号不在配置表里的
 *    帧会被放行给电机模块 —— 那样就扫不到了。
 *    所以扫描期在过滤【之前】先记录一笔。
 * ========================================================================= */

volatile uint8_t  g_tof8_scan_cnt = 0U;
volatile uint8_t  g_tof8_scan_list[16];

static volatile uint8_t  s_scanning = 0U;         /* 1 = 正在扫描 */
static volatile uint8_t  s_scan_hit[16];          /* 该设备号有应答 */
static volatile uint16_t s_scan_dist[16];         /* 该设备号最新距离 mm */

/* Private function prototypes -----------------------------------------------*/

static uint8_t  Tof8_SendReq(uint8_t slot, uint8_t ch, uint8_t b2);
static uint16_t Tof8_DecodeDist(uint16_t raw);

/* =========================================================================
 *                              内部工具
 * ========================================================================= */

/**
  * @brief  0.1mm 原始值 -> mm，同时做无效值替换
  *         文档：原始值 <= 10，或 == 8888 / 9999 视为无效
  */
static uint16_t Tof8_DecodeDist(uint16_t raw)
{
	if (raw <= TOF8_DIST_RAW_MIN_VALID)
	{
		return TOF8_DIST_MM_INVALID;
	}
	if ((raw == TOF8_DIST_RAW_INVALID_A) || (raw == TOF8_DIST_RAW_INVALID_B))
	{
		return TOF8_DIST_MM_INVALID;
	}
	/* 0.1mm -> mm，四舍五入，避免 0.4mm 的分辨率被截断丢精度 */
	return (uint16_t)((raw + 5U) / 10U);
}

/**
  * @brief  发一帧 8 字节请求：0x0408 [设备号, 通道号, B2, 0,0,0,0,0]
  * @retval 1 = 已发出；0 = 槽位非法或发送失败
  */
static uint8_t Tof8_SendReq(uint8_t slot, uint8_t ch, uint8_t b2)
{
	uint8_t buf[8];

	if (slot >= TOF8_DEV_NUM)
	{
		return 0U;
	}

	buf[0] = s_dev_list[slot];
	buf[1] = ch;
	buf[2] = b2;
	buf[3] = 0U;
	buf[4] = 0U;
	buf[5] = 0U;
	buf[6] = 0U;
	buf[7] = 0U;

	/* ★ Can_SendStd() 的返回值语义是 0=成功、1=失败，别写反 */
	if (Can_SendStd(TOF8_ID_REQ, buf, 8U) != 0U)
	{
		return 0U;
	}

	s_slot[slot].req_count++;
	return 1U;
}

/* =========================================================================
 *                              槽位查询
 * ========================================================================= */

void Tof8_Init(void)
{
	uint8_t i;

	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		s_slot[i].dist_raw   = 0U;
		s_slot[i].dist_mm    = TOF8_DIST_MM_INVALID;
		s_slot[i].has_data   = 0U;
		s_slot[i].fresh      = 0U;
		s_slot[i].resolution = 0U;
		s_slot[i].frequency  = 0U;
		s_slot[i].resp_count = 0U;
		s_slot[i].req_count  = 0U;
	}
}

uint8_t Tof8_GetDev(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_dev_list[slot] : 0xFFU;
}

uint8_t Tof8_FindSlot(uint8_t dev)
{
	uint8_t i;

	for (i = 0U; i < TOF8_DEV_NUM; i++)
	{
		if (s_dev_list[i] == dev)
		{
			return i;
		}
	}
	return 0xFFU;
}

/* =========================================================================
 *                              发送指令
 * ========================================================================= */

uint8_t Tof8_Config(uint8_t slot)
{
	uint8_t buf[8];

	if (slot >= TOF8_DEV_NUM)
	{
		return 0U;
	}

	/* 配置帧格式（文档「发送配置帧」）：
	 *   B0 ROI 起始    默认 0x11
	 *   B1 ROI 结束    默认 0x22
	 *   B2 预留        0x00
	 *   B3 光强阈值    默认 0xFF
	 *   B4 旋转角      0x00 = 不旋转
	 *   B5 模式        4x4 + 持续测量 = 0x01 | 0x04 = 0x05
	 *   B6 预留        0x00
	 *   B7 设备号
	 */
	buf[0] = TOF8_CFG_ROI_START;
	buf[1] = TOF8_CFG_ROI_END;
	buf[2] = 0x00U;
	buf[3] = TOF8_CFG_LIGHT_THRESHOLD;
	buf[4] = TOF8_CFG_ROTATE;
	buf[5] = TOF8_MODE_DEFAULT;
	buf[6] = 0x00U;
	buf[7] = s_dev_list[slot];

	/* Can_SendStd(): 0=成功，1=失败 */
	return (Can_SendStd(TOF8_ID_CFG, buf, 8U) == 0U) ? 1U : 0U;
}

uint8_t Tof8_ContinuousSet(uint8_t slot, uint8_t on)
{
	uint8_t buf[8];

	if (slot >= TOF8_DEV_NUM)
	{
		return 0U;
	}

	/* 通道 10：连续测距开关
	 *   B0 设备号  B1=10  B2=1 开 / 0 关  B3 ROI 起 0x11  B4 ROI 止 0x22  B5~B7 预留 */
	buf[0] = s_dev_list[slot];
	buf[1] = TOF8_CH_CONT_MEASURE;
	buf[2] = (on != 0U) ? 1U : 0U;
	buf[3] = TOF8_CFG_ROI_START;
	buf[4] = TOF8_CFG_ROI_END;
	buf[5] = 0U;
	buf[6] = 0U;
	buf[7] = 0U;

	/* Can_SendStd(): 0=成功，1=失败 */
	if (Can_SendStd(TOF8_ID_REQ, buf, 8U) != 0U)
	{
		return 0U;
	}

	s_slot[slot].req_count++;
	return 1U;
}

uint8_t Tof8_MeasureOnce(uint8_t slot)
{
	/* 通道 11：单次测距，发一帧就启动一次 */
	return Tof8_SendReq(slot, TOF8_CH_MEASURE_ONCE, 0U);
}

uint8_t Tof8_RequestDistance(uint8_t slot)
{
	/* 通道 12：取距离读数 */
	return Tof8_SendReq(slot, TOF8_CH_DISTANCE, 0U);
}

uint8_t Tof8_RequestResolution(uint8_t slot)
{
	return Tof8_SendReq(slot, TOF8_CH_RESOLUTION, 0U);
}

uint8_t Tof8_RequestFrequency(uint8_t slot)
{
	return Tof8_SendReq(slot, TOF8_CH_FREQUENCY, 0U);
}

uint8_t Tof8_Reset(uint8_t slot, uint8_t target)
{
	/* 通道 8 = 复位 VL53Lx，通道 9 = 复位 MCU */
	return Tof8_SendReq(slot,
	                    (target != 0U) ? TOF8_CH_RESET_MCU : TOF8_CH_RESET_VL53,
	                    0U);
}

/* =========================================================================
 *                       ★ 设备号扫描（不依赖 s_dev_list）
 * ========================================================================= */

/** @brief 按「任意设备号」发一帧请求（扫描专用，不走 s_dev_list） */
static uint8_t Tof8_SendReqRaw(uint8_t dev, uint8_t ch, uint8_t b2)
{
	uint8_t buf[8];

	buf[0] = (uint8_t)(dev & 0x0FU);
	buf[1] = ch;
	buf[2] = b2;
	buf[3] = 0U;
	buf[4] = 0U;
	buf[5] = 0U;
	buf[6] = 0U;
	buf[7] = 0U;

	return (Can_SendStd(TOF8_ID_REQ, buf, 8U) == 0U) ? 1U : 0U;
}

uint8_t Tof8_ScanDevices(uint32_t gap_ms)
{
	uint8_t dev;

	if (gap_ms == 0U) { gap_ms = 20U; }

	g_tof8_scan_cnt = 0U;
	for (dev = 0U; dev < 16U; dev++)
	{
		s_scan_hit[dev]  = 0U;
		s_scan_dist[dev] = TOF8_DIST_MM_INVALID;
	}

	s_scanning = 1U;

	/* ---- 每个设备号发两次（防丢帧），中间留间隔 ---- */
	for (dev = 0U; dev < 16U; dev++)
	{
		(void)Tof8_SendReqRaw(dev, TOF8_CH_DISTANCE, 0U);
		osDelay(gap_ms);
		(void)Tof8_SendReqRaw(dev, TOF8_CH_DISTANCE, 0U);
		osDelay(gap_ms);
	}

	s_scanning = 0U;

	/* ---- 汇总：谁应答了就记谁 ---- */
	for (dev = 0U; dev < 16U; dev++)
	{
		if (s_scan_hit[dev] != 0U)
		{
			if (g_tof8_scan_cnt < 16U)
			{
				g_tof8_scan_list[g_tof8_scan_cnt] = dev;
				g_tof8_scan_cnt++;
			}
		}
	}

	return g_tof8_scan_cnt;
}

uint16_t Tof8_ReadDistanceByDev(uint8_t dev, uint32_t timeout_ms)
{
	uint32_t t0;

	dev = (uint8_t)(dev & 0x0FU);

	if (timeout_ms == 0U) { timeout_ms = 150U; }

	/* ★ 临时开一次扫描捕获模式：复用同一条「记录任意设备号应答」的路径，
	 *   这样不需要动 s_dev_list 也能读到数据。 */
	s_scan_hit[dev]  = 0U;
	s_scan_dist[dev] = TOF8_DIST_MM_INVALID;

	if (s_scanning == 0U) { s_scanning = 1U; }

	t0 = HAL_GetTick();
	for (;;)
	{
		(void)Tof8_SendReqRaw(dev, TOF8_CH_DISTANCE, 0U);
		osDelay(TOF8_REQ_GAP_MS);

		if ((s_scan_hit[dev] != 0U) && (s_scan_dist[dev] != TOF8_DIST_MM_INVALID))
		{
			uint16_t d = s_scan_dist[dev];
			s_scanning = 0U;
			return d;
		}

		if ((uint32_t)(HAL_GetTick() - t0) >= timeout_ms)
		{
			break;
		}
	}

	s_scanning = 0U;
	return TOF8_DIST_MM_INVALID;
}

/* =========================================================================
 *                              接收解析
 * ========================================================================= */

uint8_t Tof8_OnFrame(const CanFrame_t *frame)
{
	uint8_t  slot;
	uint8_t  ch;
	uint16_t raw;

	if (frame == NULL)
	{
		return 0U;
	}

	/* 先把明显不是 TOF8 的 ID 全部放行，避免影响电机模块的判断速度 */
	if ((frame->id != TOF8_ID_RESP) &&
	    (frame->id != TOF8_ID_REPORT_DIST) &&
	    (frame->id != TOF8_ID_REPORT_DIST_TEMP))
	{
		return 0U;
	}

	if (frame->dlc < 1U)
	{
		return 0U;
	}

	/* ★★★ 扫描捕获：必须在「按设备号过滤」之前 ★★★
	 *
	 *   扫描的目的就是找出「设备号到底是多少」，所以这时 s_dev_list
	 *   里配的值很可能是错的。如果先过滤，正确设备号的应答反而会被
	 *   放行给电机模块，永远扫不到。
	 *
	 *   只认「0x0409 且通道 12（取距离）」的帧：
	 *   电机也发 0x0409，但通道号是查询类型（0x25 等），不会撞。 */
	if ((s_scanning != 0U) &&
	    (frame->id == TOF8_ID_RESP) &&
	    (frame->dlc >= 5U) &&
	    (frame->data[1] == TOF8_CH_DISTANCE))
	{
		uint8_t dev = (uint8_t)(frame->data[0] & 0x0FU);
		uint16_t raw = (uint16_t)frame->data[2] | ((uint16_t)frame->data[3] << 8);

		s_scan_hit[dev]  = 1U;
		s_scan_dist[dev] = Tof8_DecodeDist(raw);
	}

	/* 设备号不是本驱动管的 -> 放行给电机模块 */
	slot = Tof8_FindSlot(frame->data[0]);
	if (slot == 0xFFU)
	{
		return 0U;
	}

	/* ---- 0x0793 / 0x0794：自动上报 [设备号, 预留, 距离L, 距离H, (温度)] ---- */
	if ((frame->id == TOF8_ID_REPORT_DIST) ||
	    (frame->id == TOF8_ID_REPORT_DIST_TEMP))
	{
		if (frame->dlc < 4U)
		{
			return 1U;              /* 是我的设备，但帧太短，吞掉不报错 */
		}
		raw = (uint16_t)frame->data[2] | ((uint16_t)frame->data[3] << 8);
		s_slot[slot].dist_raw = raw;
		s_slot[slot].dist_mm  = Tof8_DecodeDist(raw);
		s_slot[slot].fresh    = 1U; /* 主动上报 = 新数据 */
		if (s_slot[slot].dist_mm != TOF8_DIST_MM_INVALID)
		{
			s_slot[slot].has_data = 1U;
		}
		return 1U;
	}

	/* ---- 0x0409：请求应答 [设备号, 通道号, 数据...] ---- */
	if (frame->dlc < 2U)
	{
		return 1U;                  /* 是我的设备，但帧太短 */
	}

	ch = frame->data[1];
	s_slot[slot].resp_count++;

	switch (ch)
	{
	case TOF8_CH_DISTANCE:
		/* 应答 8 字节：[B0 设备号, B1=12, B2-B3 距离 u16LE, B4 新数据标志, B5-B7 预留] */
		if (frame->dlc < 5U)
		{
			break;
		}
		raw = (uint16_t)frame->data[2] | ((uint16_t)frame->data[3] << 8);
		s_slot[slot].dist_raw = raw;
		s_slot[slot].dist_mm  = Tof8_DecodeDist(raw);
		s_slot[slot].fresh    = (uint8_t)(frame->data[4] & 0x01U);
		if (s_slot[slot].dist_mm != TOF8_DIST_MM_INVALID)
		{
			s_slot[slot].has_data = 1U;
		}
		break;

	case TOF8_CH_RESOLUTION:
		/* 应答：[B0 设备号, B1=15, B2=16(4x4) / 64(8x8)] */
		if (frame->dlc >= 3U)
		{
			s_slot[slot].resolution = frame->data[2];
		}
		break;

	case TOF8_CH_FREQUENCY:
		/* 应答：[B0 设备号, B1=16, B2=频率 Hz] */
		if (frame->dlc >= 3U)
		{
			s_slot[slot].frequency = frame->data[2];
		}
		break;

	case TOF8_CH_CONT_MEASURE:
	case TOF8_CH_MEASURE_ONCE:
	case TOF8_CH_RESET_VL53:
	case TOF8_CH_RESET_MCU:
	default:
		/* 开关/复位/单次启动的应答只是一次 ACK，具体结果看 resp_count 和后续数据帧 */
		break;
	}

	return 1U;
}

/* =========================================================================
 *                              数据读取
 * ========================================================================= */

uint16_t Tof8_GetDistanceMm(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_slot[slot].dist_mm : TOF8_DIST_MM_INVALID;
}

uint16_t Tof8_GetDistanceRaw(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_slot[slot].dist_raw : 0U;
}

uint8_t Tof8_HasData(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_slot[slot].has_data : 0U;
}

uint8_t Tof8_IsFresh(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_slot[slot].fresh : 0U;
}

uint32_t Tof8_GetRespCount(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_slot[slot].resp_count : 0U;
}

uint8_t Tof8_GetResolution(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_slot[slot].resolution : 0U;
}

uint8_t Tof8_GetFrequency(uint8_t slot)
{
	return (slot < TOF8_DEV_NUM) ? s_slot[slot].frequency : 0U;
}

uint16_t Tof8_ReadDistanceMm(uint8_t slot, uint32_t timeout_ms)
{
	uint32_t t0;
	uint16_t d;

	if (slot >= TOF8_DEV_NUM)
	{
		return TOF8_DIST_MM_INVALID;
	}

	t0 = HAL_GetTick();

	for (;;)
	{
		/* 每轮先清 fresh，避免把上一轮的「新数据」误当成这一轮的 */
		s_slot[slot].fresh = 0U;

		/* ★ 发送失败（CAN 邮箱满）不算致命，总线太挤时下一轮再试即可，
		 *   真正判成败的是下面有没有拿到「新数据」 */
		(void)Tof8_RequestDistance(slot);
		osDelay(TOF8_REQ_GAP_MS);

		d = s_slot[slot].dist_mm;
		if ((d != TOF8_DIST_MM_INVALID) && (s_slot[slot].fresh != 0U))
		{
			return d;
		}

		if ((uint32_t)(HAL_GetTick() - t0) >= timeout_ms)
		{
			break;
		}
	}

	/* 超时：把手上最后一帧原样返回（可能是无效值），由调用方判断 */
	return s_slot[slot].dist_mm;
}
