/**
  ******************************************************************************
  * @file    motor.h
  * @brief   电机 CAN 协议 V1.1 协议层（STM32 侧）
  *
  *  报文格式与上位机 GyroHostCS/Motor.cs（已实机验证）逐字节一致，
  *  区别只是把 Thread.Sleep 阻塞等待换成由 Motor_Tick() 推进的非阻塞状态机。
  *
  *  用法：
  *      main() 里：Can_Init(); Motor_Init(); Can_SetRxCallback(OnCanRx);
  *      接收任务里：Motor_OnFrame(&frame);          // 每收到一帧都喂进去，非电机帧自动忽略
  *      周期任务里：Motor_StartInit();               // 只调一次
  *                  while(1) { Motor_Tick(); osDelayUntil(...); }   // 每 10ms 一次
  *
  *  单位：位移 0.1mm，速度 0.1mm/s，加速度 0.1mm/s^2（本文件所有接口都用这套单位）
  ******************************************************************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 左右电机下标（与 car_cfg.h 的配置顺序一致） */
#define MOTOR_LEFT      0U
#define MOTOR_RIGHT     1U
#define MOTOR_COUNT     2U

#define MOTOR_TICK_PERIOD_MS    10U     /* Motor_Tick() 的调用周期 */
#define MOTOR_INIT_GAP_MS       300U    /* 初始化各步之间的间隔（与上位机实测值一致） */

/* 初始化状态 */
typedef enum
{
	MOTOR_INIT_IDLE = 0,        /* 还没开始 */
	MOTOR_INIT_RUNNING,         /* 正在按步骤下发 */
	MOTOR_INIT_READY,           /* 两台电机初始化完成 */
	MOTOR_INIT_FAILED           /* 本地发送失败（总线/邮箱异常） */
} MotorInitState_t;

/* 初始化步骤编号（调试观察用，g_motor_step） */
enum
{
	MOTOR_STEP_IDLE = 0,
	MOTOR_STEP_BIND_SN,
	MOTOR_STEP_CONFIG_PARAM,
	MOTOR_STEP_CONFIG_WHEEL,
	MOTOR_STEP_SET_ZERO,
	MOTOR_STEP_ENABLE,
	MOTOR_STEP_SET_ACC,
	MOTOR_STEP_QUERY_POS,
	MOTOR_STEP_DONE
};

/* ------------------------------------------------------------------ 初始化 */

/* 上电初始化（调度器启动前调用）：填好配置、清空状态，不发任何报文 */
void Motor_Init(void);

/* 启动初始化序列（可重复调用，会从头再来一遍） */
void Motor_StartInit(void);

/* 推进初始化状态机；每 MOTOR_TICK_PERIOD_MS 调用一次。返回 1 表示本次推进了一步 */
uint8_t Motor_Tick(void);

MotorInitState_t Motor_GetInitState(void);
uint8_t Motor_GetStep(void);                 /* 当前步骤（MOTOR_STEP_*） */
uint8_t Motor_GetInitingIndex(void);         /* 当前正在初始化第几台（0=左 1=右） */
uint8_t Motor_IsReady(void);                 /* 1 = 两台都初始化完成 */

/* ------------------------------------------------------------------ 收帧 */

/* 解析一帧电机反馈/应答（在 CAN 接收任务里调用；非电机帧自动忽略） */
void Motor_OnFrame(const CanFrame_t *frame);

/* ------------------------------------------------------------------ 底层指令 */
/* 全部返回 0 = 已交给 CAN 外设，1 = 失败（参数非法 / 发送失败） */

uint8_t Motor_SendPosRel(uint8_t idx, int32_t dist_0p1mm);      /* 相对位移 */
uint8_t Motor_SendPosAbs(uint8_t idx, int32_t pos_0p1mm);       /* 绝对位置 */
uint8_t Motor_SendSetZero(uint8_t idx);                         /* 设当前位置=0（仅主帧组支持） */
uint8_t Motor_SendSpeed(uint8_t idx, uint32_t speed_0p1mmps);
uint8_t Motor_SendAcc(uint8_t idx, uint32_t acc_0p1mmps2);
uint8_t Motor_SendLock(uint8_t idx, uint8_t lock_code);         /* 0x5A 使能 / 0x4A 释放 */
uint8_t Motor_SendQuery(uint8_t idx, uint8_t query_type);       /* 0x06/0x08/0x25/0xFF */
uint8_t Motor_SupportsSetZero(uint8_t idx);

/* 查询快捷方式（等价于 Motor_SendQuery） */
uint8_t Motor_QueryTemp(uint8_t idx);
uint8_t Motor_QueryPos(uint8_t idx);
uint8_t Motor_QueryStatus(uint8_t idx);
uint8_t Motor_QueryVersion(uint8_t idx);

/* BB 声提示：0x0408，DLC=6，[设备号, 0x07, 音量, 频率低, 频率中, 频率高]
 * ★ 文档只写了"频率信息等"，没有给编码方式；这里按 24 位小端 u32 低 3 字节发送，
 *   是推断值，第一次实机使用时请确认蜂鸣器是否按预期叫。 */
uint8_t Motor_SendBeep(uint8_t idx, uint8_t volume, uint32_t freq);

/* 主动索取 SN（0x060D）；更可靠的做法是给电机断电再上电，被动收 0x0312 广播 */
uint8_t Motor_RequestSn(uint8_t idx);

/* ------------------------------------------------------------------ 状态读取 */

uint8_t Motor_GetDev(uint8_t idx);
uint8_t Motor_GetUseMainFrame(uint8_t idx);

uint8_t Motor_HasRemain(uint8_t idx);        /* 1 = 收到过该电机的剩余位移反馈 */
int32_t Motor_GetRemain(uint8_t idx);        /* 剩余位移 0.1mm；0 = 已到位停稳 */
int16_t Motor_GetTorque(uint8_t idx);

uint8_t Motor_HasQueryPos(uint8_t idx);      /* 1 = 收到过 0x0409 位置查询应答 */
int32_t Motor_GetQueryPos(uint8_t idx);      /* 电机自报绝对位置 0.1mm */

/* ------------------------------------------------------------------
 *  ★ 课题三·飞箱：按「物理设备号」直接访问（不经过左/右轮索引）
 *
 *  为什么必须单独一组：
 *    小车那两台是固件「主动写」设备号 0/1，所以可以用 MOTOR_LEFT / MOTOR_RIGHT
 *    这种索引。飞箱的 5 台电机设备号出厂固定且不是 0/1，固件只能「读」不能改，
 *    所以一切访问必须直接用设备号。
 *
 *  ★ 设备号有效范围 0..15（内部缓存数组长度 MOTOR_DEV_SLOTS = 16）。
 *    飞箱的设备号必须在这个范围里，否则下面这些接口一律返回 0/无数据。
 * ------------------------------------------------------------------ */

/** @brief 按设备号发查询（0x0408 的 B1 = query_type），不需要先登记配置表 */
uint8_t  Motor_SendQueryDev(uint8_t dev, uint8_t query_type);

/**
  * @brief  ★ 按 SN 绑定设备号：0x0313，数据 = [SN0..SN6, 设备号]，DLC=8
  *
  *         ★★★ 为什么这条是飞箱的核心 ★★★
  *           协议里【没有】「查询设备号」的命令（能查的只有温度/位置/状态/版本），
  *           所以 5 台电机如果出厂都是 0 号，靠听反馈帧永远分不清谁是谁
  *           （CAN 位仲裁会把「同 ID 同数据」的帧合并成一帧）。
  *
  *           但 0x0313 是按 SN 寻址的：只有 SN 完全匹配的那一台会接受。
  *           所以只要知道每台的 SN（电机标签上印着），就能把 5 台一次性
  *           分别绑到 0/1/2/3/4，全程不需要逐台上电、不会误伤别的电机。
  *
  *         ★ 与 0x0316 的关系：
  *           0x0313 负责「能被哪台收到」，0x0316 负责「Data[4] 用哪个号寻址」。
  *           飞箱里两者填同一个值（见 flybox.h 的 FLY_MOTOR_NO_EQ_DEV）。
  *
  * @param  sn7  7 字节 SN（最后一位通常是 0x00）
  * @param  dev  目标设备号 0..15
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t  Motor_SendBindSn(const uint8_t *sn7, uint8_t dev);

/**
  * @brief  按设备号写电机参数：0x0316，数据 = [B4 BF CF AA 00 5F, Byte6, 设备号]
  *         Byte6 低 4 位 = motor_no（★ 必须填这台电机【当前自己的】号，保持原值）
  *         Byte6 bit5/bit4 = 0 -> 自动回传剩余位移 / 力矩
  * @retval 0 成功，1 参数非法或发送失败
  */
uint8_t  Motor_SendParamDev(uint8_t dev, uint8_t motor_no);

/** @brief 该设备号是否收到过剩余位移反馈（0x0420）*/
uint8_t  Motor_HasRemainDev(uint8_t dev);

/** @brief 该设备号的剩余位移 0.1mm（0 = 已到位）*/
int32_t  Motor_GetRemainDev(uint8_t dev);

/** @brief 该设备号是否收到过位置查询应答（0x0409 B1=0x08）*/
uint8_t  Motor_HasQueryPosDev(uint8_t dev);

/** @brief 该设备号自报的绝对位置 0.1mm */
int32_t  Motor_GetQueryPosDev(uint8_t dev);

/**
  * @brief  该设备号累计收到多少帧（0x0420 / 0x0440 / 0x0380 / 0x0409）
  *         ★ 设备号扫描的核心依据：先清零（Motor_ClearFbCount），
  *           再逐个设备号发查询，帧数从 0 变成非 0 的，就是真实存在的设备。
  *         比「LED/蜂鸣器反推」安全得多，因为它只是一次读操作。
  */
uint32_t Motor_GetFbCountDev(uint8_t dev);

/** @brief 把所有设备号的接收帧计数清零（扫描前调用）*/
void     Motor_ClearFbCount(void);

/* 反馈帧序号：每收到一帧该电机的 0x0420 / 0x0409 就 +1。
 * 用它可以判断"是否来了一帧新的反馈"，避免把旧值当成新值重复使用。 */
uint32_t Motor_GetFeedbackSeq(uint8_t idx);

uint8_t Motor_HasQueryTemp(uint8_t idx);     /* 1 = 收到过温度应答 */
int16_t Motor_GetQueryTemp(uint8_t idx);     /* 温度，0.1℃（250 = 25.0℃）*/

uint8_t Motor_HasQueryStatus(uint8_t idx);   /* 1 = 收到过状态应答 */
uint8_t Motor_GetQueryStatus(uint8_t idx);   /* 状态位，见 MOTOR_STATUS_* */

uint8_t Motor_HasQueryVersion(uint8_t idx);  /* 1 = 收到过版本应答 */
uint32_t Motor_GetQueryVersion(uint8_t idx); /* 固件版本号 */

uint32_t Motor_GetAckCount(void);            /* 累计 ACK 帧数 */
uint16_t Motor_GetLastAckId(void);           /* 最近一次 ACK 对应的命令帧 ID */
uint16_t Motor_GetErrorCode(void);           /* 最近一次错误上报的错误码（0 = 没有） */
uint16_t Motor_GetErrorFrameId(void);        /* 错误帧的 ID（0x0100~0x017F） */
uint32_t Motor_GetErrorCount(void);
uint32_t Motor_GetMsSinceLastFrame(void);    /* 距上一次收到电机帧的时间（ms）；0xFFFFFFFF = 从没收到过 */

/* 运行中收到的 SN 广播（0x0312），以及收到次数 */
const uint8_t *Motor_GetRxSn(void);
uint32_t Motor_GetRxSnCount(void);

/**
  * @brief  ★ 取上电广播里第 idx 台【不同】电机的 SN（自动识别左右轮用）
  *
  *         上电时两台电机各广播一条 0x0312，这里按"首次出现的不同 SN"
  *         分别存进两个槽位，所以两条都能拿到 —— 不再需要 car_cfg.h
  *         里写死 SN。
  * @param  idx  0 或 1
  * @retval SN 指针（7 字节）；该槽位还没收到过则返回 0
  */
const uint8_t *Motor_GetBootSn(uint8_t idx);

/** @brief 上电广播里已收集到几台不同 SN 的电机（0 / 1 / 2）*/
uint8_t Motor_GetBootSnUsed(void);

/** @brief 累计收到多少条 0x0312 广播（含重复）*/
uint32_t Motor_GetBootSnTotal(void);

/** @brief 清空上电广播的 SN 槽位（重新识别前调用）*/
void Motor_ClearBootSn(void);

/* ------------------------------------------------------------------ Watch 变量 */
/* 调试器里直接观察这些全局变量即可（定义在 motor.c） */
extern volatile uint8_t  g_motor_state;         /* MotorInitState_t */
extern volatile uint8_t  g_motor_step;          /* MOTOR_STEP_* */
extern volatile uint8_t  g_motor_index;         /* 正在初始化第几台（0=左 1=右）*/
extern volatile uint8_t  g_motor_ready;         /* 1 = 两台都初始化完成 */
extern volatile uint8_t  g_motor_sn_missing;    /* 位图：哪台电机没 SN、跳过了绑定 */
extern volatile uint32_t g_motor_ack_count;     /* 累计 ACK 帧数 */
extern volatile uint32_t g_motor_send_fail;     /* 本地发送失败次数 */
extern volatile int32_t  g_motor_remain_left;   /* 左电机剩余位移 0.1mm */
extern volatile int32_t  g_motor_remain_right;  /* 右电机剩余位移 0.1mm */
extern volatile int32_t  g_motor_pos_left;      /* 左电机自报绝对位置 0.1mm */
extern volatile int32_t  g_motor_pos_right;
extern volatile int16_t  g_motor_temp_left;     /* 左电机温度 0.1℃（250 = 25.0℃）*/
extern volatile int16_t  g_motor_temp_right;
extern volatile uint8_t  g_motor_status_left;   /* 左电机状态位（MOTOR_STATUS_*）*/
extern volatile uint8_t  g_motor_status_right;
extern volatile uint32_t g_motor_ver_left;      /* 左电机固件版本号 */
extern volatile uint32_t g_motor_ver_right;
extern volatile uint16_t g_motor_error_code;    /* 最近一次错误码 */
extern volatile uint32_t g_motor_rx_sn_count;   /* 收到 0x0312 SN 的次数 */
extern volatile uint8_t  g_motor_rx_sn[7];      /* 最近一次收到的 SN */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
