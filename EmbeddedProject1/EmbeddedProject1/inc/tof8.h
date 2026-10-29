/**
  ******************************************************************************
  * @file    tof8.h
  * @brief   课题三·TOF8 激光测距模块 CAN 驱动（协议见《tof8通讯协议.docx》）
  *
  *  ★ 一句话理解这套协议：
  *     TOF8 和电机共用同一套 CAN ID（0x0408 请求 / 0x0409 应答 / 0x0316 配置），
  *     靠 **Data[0]=设备号 + Data[1]=通道号** 区分是「谁、干什么」。
  *     所以本驱动必须只认自己的设备号，其它帧一律放行走 Motor_OnFrame()，
  *     否则会把电机的查询应答抢走（或反过来被电机模块误解析）。
  *
  *  协议摘要（8 字节帧）：
  *  ┌──────────────────────────────────────────────────────────────────────┐
  *  │ 配置帧   0x0316  [ROI起 ROI止 预留 光强 旋转 模式 预留 设备号]        │
  *  │ 请求帧   0x0408  [设备号 通道号 参数...]                             │
  *  │ 应答帧   0x0409  [设备号 通道号 数据...]                             │
  *  │ 上报帧   0x0793  [设备号 预留 距离L 距离H ...]  （距离自动上报）      │
  *  │ 上报帧   0x0794  [设备号 预留 距离L 距离H 温度 ...] （距离+温度）     │
  *  └──────────────────────────────────────────────────────────────────────┘
  *
  *  通道号（Data[1]）：
  *      8  = 复位 VL53Lx        9  = 复位 MCU
  *      10 = 连续测距开关        11 = 单次测距(启动)
  *      12 = 距离读数            15 = 分辨率(4x4/8x8)
  *      16 = 测量频率(Hz)
  *
  *  距离单位：u16 小端，0.1 mm（除以 10 得 mm）
  *  无效值：原始值 <= 10，或 == 8888 / 9999 —— 统一对外返回 9999
  *
  *  标准使用流程（文档「二、流程」）：
  *      1) 开机发一次配置帧     Tof8_Config()
  *      2) 开连续测量           Tof8_ContinuousSet(1)
  *      3) 循环取数             Tof8_RequestDistance() + Tof8_GetDistanceMm()
  *      4) 用完关连续测量       Tof8_ContinuousSet(0)
  ******************************************************************************
  */

#ifndef __TOF8_H__
#define __TOF8_H__

#include <stdint.h>
#include "can.h"

/* =========================================================================
 *                       1. 协议常量（照抄文档，不要改）
 * ========================================================================= */

/* CAN ID */
#define TOF8_ID_CFG                 0x0316U  /* 配置帧（与电机参数配置同 ID）*/
#define TOF8_ID_REQ                 0x0408U  /* 请求帧（与电机配轮径/查询同 ID）*/
#define TOF8_ID_RESP                0x0409U  /* 应答帧（与电机查询应答同 ID）*/
#define TOF8_ID_REPORT_DIST         0x0793U  /* 距离自动上报 */
#define TOF8_ID_REPORT_DIST_TEMP    0x0794U  /* 距离+温度自动上报 */

/* 通道号（应答帧 Data[1]）*/
#define TOF8_CH_RESET_VL53          8U       /* 复位 VL53Lx 传感器 */
#define TOF8_CH_RESET_MCU           9U       /* 复位 MCU */
#define TOF8_CH_CONT_MEASURE        10U      /* 连续测距开关：Data[2]=1 开 / 0 关 */
#define TOF8_CH_MEASURE_ONCE        11U      /* 单次测距：发一帧即启动一次 */
#define TOF8_CH_DISTANCE            12U      /* 距离读数 */
#define TOF8_CH_RESOLUTION          15U      /* 分辨率：Data[2]=16(4x4) / 64(8x8) */
#define TOF8_CH_FREQUENCY           16U      /* 测量频率：Data[2]=Hz */

/* 配置帧默认值 */
#define TOF8_CFG_ROI_START          0x11U    /* ROI 起始 */
#define TOF8_CFG_ROI_END            0x22U    /* ROI 结束 */
#define TOF8_CFG_LIGHT_THRESHOLD    0xFFU    /* 光强阈值 */
#define TOF8_CFG_ROTATE             0x00U    /* 旋转角：0 = 不旋转 */

#define TOF8_MODE_4X4               0x01U    /* 4x4 分区 */
#define TOF8_MODE_8X8               0x02U    /* 8x8 分区（未在文档配置示例中使用）*/
#define TOF8_MODE_ALWAYS_MEASURE    0x04U    /* 持续测量 */
#define TOF8_MODE_DEFAULT           (TOF8_MODE_4X4 | TOF8_MODE_ALWAYS_MEASURE)  /* = 0x05 */

/* 距离无效判定 */
#define TOF8_DIST_RAW_MIN_VALID     10U      /* 原始值 <= 10 视为无效（贴死/无回波）*/
#define TOF8_DIST_RAW_INVALID_A     8888U    /* 无效标志值 A */
#define TOF8_DIST_RAW_INVALID_B     9999U    /* 无效标志值 B */
#define TOF8_DIST_MM_INVALID        9999U    /* 对外的无效值（与文档一致）*/

/* 连续模式首次出数可能需要 > 100 ms，这里给足 500 ms */
#define TOF8_FIRST_DATA_TIMEOUT_MS  500U

/* 槽位数上限（设备号映射表的长度上限）*/
#define TOF8_MAX_SLOTS              4U

/* =========================================================================
 *                       2. 设备槽位（数量由 car_cfg.h 的 TOF8_DEV_NUM 决定）
 * ========================================================================= */

/**
  * @brief  初始化驱动内部状态（上电调用一次，不发送任何报文）
  */
void Tof8_Init(void);

/**
  * @brief  查某个槽位对应的物理设备号
  * @param  slot  槽位下标 0..TOF8_DEV_NUM-1
  * @retval 设备号；槽位非法返回 0xFF
  */
uint8_t Tof8_GetDev(uint8_t slot);

/**
  * @brief  按设备号反查槽位下标
  * @param  dev   物理设备号
  * @retval 槽位下标；不是本驱动管理的设备返回 0xFF
  */
uint8_t Tof8_FindSlot(uint8_t dev);

/* =========================================================================
 *                       3. 发送指令
 * ========================================================================= */

/**
  * @brief  发配置帧 0x0316（开机发一次即可）
  *         [ROI起 ROI止 预留 光强 旋转 模式 预留 设备号]
  */
uint8_t Tof8_Config(uint8_t slot);

/**
  * @brief  连续测距开关（通道 10）
  * @param  on  1 = 开，0 = 关
  */
uint8_t Tof8_ContinuousSet(uint8_t slot, uint8_t on);

/**
  * @brief  单次测距启动（通道 11），启动后需再用 Tof8_RequestDistance() 取结果
  */
uint8_t Tof8_MeasureOnce(uint8_t slot);

/**
  * @brief  请求一次距离读数（通道 12），结果在后续 0x0409 应答里
  */
uint8_t Tof8_RequestDistance(uint8_t slot);

/**
  * @brief  查询分辨率（通道 15）
  */
uint8_t Tof8_RequestResolution(uint8_t slot);

/**
  * @brief  查询测量频率（通道 16）
  */
uint8_t Tof8_RequestFrequency(uint8_t slot);

/**
  * @brief  复位
  * @param  target  0 = 复位 VL53Lx 传感器（通道 8），1 = 复位 MCU（通道 9）
  */
uint8_t Tof8_Reset(uint8_t slot, uint8_t target);

/* =========================================================================
 *                       4. 接收与数据读取
 * ========================================================================= */

/**
  * @brief  解析一帧 CAN 报文
  *  ★ 调用顺序极其重要：必须在 Motor_OnFrame() 之前调用！
  *     本函数只「认领」属于 TOF8 设备号的 0x0409/0x0793/0x0794，
  *     认领了返回 1，调用方就不要再把这一帧交给 Motor_OnFrame()；
  *     返回 0 说明不是 TOF8 的帧，正常交给电机模块处理。
  * @retval 1 = 本帧已由 TOF8 消费；0 = 与 TOF8 无关
  */
uint8_t Tof8_OnFrame(const CanFrame_t *frame);

/**
  * @brief  取最近一次有效距离（mm）
  * @retval 距离 mm；无数据或无效返回 TOF8_DIST_MM_INVALID(9999)
  */
uint16_t Tof8_GetDistanceMm(uint8_t slot);

/**
  * @brief  取最近一次距离原始值（0.1 mm，未做无效替换）
  */
uint16_t Tof8_GetDistanceRaw(uint8_t slot);

/**
  * @brief  是否收到过有效数据
  */
uint8_t Tof8_HasData(uint8_t slot);

/**
  * @brief  最近一次应答的「新数据标志」（Data[4]）
  *         1 = 模块端有新数据；0 = 还是旧数据（说明请求发太快或模块没测完）
  */
uint8_t Tof8_IsFresh(uint8_t slot);

/**
  * @brief  该槽位收到 0x0409 应答的累计帧数（判断通信是否通）
  */
uint32_t Tof8_GetRespCount(uint8_t slot);

/**
  * @brief  连续请求距离，直到拿到一次「新数据」或超时
  *         ★ 这是最常用的取数接口，内部自己处理重试节奏
  * @param  slot       槽位
  * @param  timeout_ms 超时
  * @retval 距离 mm；超时或无效返回 TOF8_DIST_MM_INVALID
  */
uint16_t Tof8_ReadDistanceMm(uint8_t slot, uint32_t timeout_ms);

/**
  * @brief  最近一次查询到的分辨率（16=4x4 / 64=8x8），0=未查到
  */
uint8_t Tof8_GetResolution(uint8_t slot);

/**
  * @brief  最近一次查询到的测量频率（Hz），0=未查到
  */
uint8_t Tof8_GetFrequency(uint8_t slot);

#endif /* __TOF8_H__ */
