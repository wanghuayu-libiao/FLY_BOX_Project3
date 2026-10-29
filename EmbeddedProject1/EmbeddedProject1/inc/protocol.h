/**
  ******************************************************************************
  * @file    protocol.h
  * @brief   CAN 报文定义（标准帧，1Mbps），分两个互相独立的区域：
  *          【区域 A】陀螺仪板 <-> 上位机（数据均为小端），上位机（GyroHost/Protocol.cs）必须与本文件保持一致。
  *          【区域 B】电机 CAN 协议 V1.1（文件末尾），与陀螺仪协议互不干涉。
  *
  *  ---- 区域 A：陀螺仪协议 ----
  *  0x101 角度帧  板->上位机  每 10ms，DLC=8
  *        [0..3] int32  角度，单位 0.001°，范围 -180000 ~ +179999
  *        [4..5] int16  角速度（已扣零偏），单位 0.01°/s
  *        [6..7] uint16 序号，每个 10ms 周期 +1（用于判断丢帧/还原时间轴）
  *  0x102 信息帧  板->上位机  每 100ms，DLC=8
  *        [0..1] int16  温度，单位 0.01℃
  *        [2]    uint8  状态寄存器 0x04 的读回值
  *        [3]    uint8  标志位（PROTO_FLAG_*）
  *        [4..5] int16  当前温度下的零偏，单位 0.001°/s
  *        [6..7] int16  在线学到的零偏温度系数，单位 0.001°/s/℃（旧固件恒为 0）
  *  0x110 命令帧  上位机->板  DLC>=1
  *        [0]    uint8  命令码（PROTO_CMD_*）
  ******************************************************************************
  */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdint.h>

/* =========================================================================
 *                    区域 A：陀螺仪 CAN 协议（陀螺仪板 <-> 上位机）
 * ========================================================================= */

/* 报文 ID */
#define PROTO_ID_ANGLE          0x101U
#define PROTO_ID_INFO           0x102U
#define PROTO_ID_CMD            0x110U
#define PROTO_ID_TOTAL          0x103U   /* ★ 标度因子/时间基自检（每 100ms 一帧）*/

/* 角度帧周期；信息帧每 PROTO_INFO_DIV 个角度帧发送一次 */
#define PROTO_ANGLE_PERIOD_MS   10U
#define PROTO_INFO_DIV          10U

/* 信息帧标志位 */
#define PROTO_FLAG_CALIBRATED   0x01U   /* 零偏标定完成，角度有效 */
#define PROTO_FLAG_CALIBRATING  0x02U   /* 正在标定零偏（需保持静止） */
#define PROTO_FLAG_COMM_FAULT   0x04U   /* 状态寄存器读回 0x00/0xFF，SPI 通信异常 */
#define PROTO_FLAG_SATURATED    0x08U   /* 角速度曾达到量程上限，角度已不可信，清零后复位 */

/* 命令码 */
#define PROTO_CMD_ZERO_ANGLE    0x01U   /* 把当前角度置 0 */
#define PROTO_CMD_RECALIBRATE   0x02U   /* 重新标定零偏（标定期间需保持静止，完成后角度置 0） */

static inline int32_t Proto_RoundF(float x)
{
	return (int32_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}

static inline void Proto_PackAngle(uint8_t *b, float angle_deg, float rate_dps, uint16_t seq)
{
	uint32_t a = (uint32_t)Proto_RoundF(angle_deg * 1000.0f);
	uint16_t r = (uint16_t)(int16_t)Proto_RoundF(rate_dps * 100.0f);

	b[0] = (uint8_t)(a);
	b[1] = (uint8_t)(a >> 8);
	b[2] = (uint8_t)(a >> 16);
	b[3] = (uint8_t)(a >> 24);
	b[4] = (uint8_t)(r);
	b[5] = (uint8_t)(r >> 8);
	b[6] = (uint8_t)(seq);
	b[7] = (uint8_t)(seq >> 8);
}

static inline void Proto_PackInfo(uint8_t *b, float temp_c, uint8_t status, uint8_t flags, float bias_dps, float temp_coef)
{
	uint16_t t  = (uint16_t)(int16_t)Proto_RoundF(temp_c * 100.0f);
	uint16_t bi = (uint16_t)(int16_t)Proto_RoundF(bias_dps * 1000.0f);
	uint16_t kc = (uint16_t)(int16_t)Proto_RoundF(temp_coef * 1000.0f);

	b[0] = (uint8_t)(t);
	b[1] = (uint8_t)(t >> 8);
	b[2] = status;
	b[3] = flags;
	b[4] = (uint8_t)(bi);
	b[5] = (uint8_t)(bi >> 8);
	b[6] = (uint8_t)(kc);
	b[7] = (uint8_t)(kc >> 8);
}

/* =========================================================================
 *                        区域 B：电机 CAN 协议 V1.1
 * ========================================================================= */

/* --- 电机设备号 --- */
#define MOTOR_DEV_MAIN              0x00U   /* 主电机 */
#define MOTOR_DEV_SLAVE             0x01U   /* 从电机 */

/* --- 1. 控制指令 ID（主控发 → 电机）--- */
#define CAN_ID_MOTOR_SET_POS_MAIN   0x0416U   /* 设置主电机目标位置（6字节）*/
#define CAN_ID_MOTOR_SET_POS_SLAVE  0x0419U   /* 设置从电机目标位置（5字节）*/
#define CAN_ID_MOTOR_SET_SPD_MAIN   0x0415U   /* 设置主电机速度（5字节）*/
#define CAN_ID_MOTOR_SET_SPD_SLAVE  0x0418U   /* 设置从电机速度（5字节）*/
#define CAN_ID_MOTOR_SET_ACC_MAIN   0x0414U   /* 设置主电机加速度（5字节）*/
#define CAN_ID_MOTOR_SET_ACC_SLAVE  0x0417U   /* 设置从电机加速度（5字节）*/
#define CAN_ID_MOTOR_LOCK           0x0413U   /* 控制电机锁定/释放（5字节）*/

/* ★ 0x0408 是「多用途」ID：配置轮径、查询（温度/位置/状态/版本）、BB 声提示都用同一个 ID，
 *   靠 Data[1] 区分具体用途（查询类型见下面的 MOTOR_QUERY_*）。
 *   CAN_ID_MOTOR_CFG_WHEEL 与 CAN_ID_MOTOR_QUERY 数值相同，只是按用途起的两个名字。 */
#define CAN_ID_MOTOR_CFG_WHEEL      0x0408U   /* 配置轮径/查询/BB声（多用途，靠 Data[1] 区分）*/
#define CAN_ID_MOTOR_CFG_PARAM      0x0316U   /* 配置电机参数（初始化，8字节）*/

/* --- 0x0408 的子功能码（Data[1]）--- */
#define MOTOR_CFG_SUB_WHEEL         0x24U     /* 配置轮径：Data[2..3] 轮径 u16，0.01mm，需连发 3 次 */
#define MOTOR_CFG_SUB_BEEP          0x07U     /* BB 声提示：Data[2] 音量，Data[3..5] 频率 */

/* --- SN 相关（课题二 第二步"电机绑定"必须用）--- */
/* 实测：电机上电后不绑定 SN，只会回 ACK、不转也不报位移 */
#define CAN_ID_MOTOR_GET_SN         0x060DU   /* 索取 SN：Data[0] 设备类型，Data[1]=0 */
#define CAN_ID_MOTOR_SN             0x0312U   /* SN 应答/上电广播：Data[0..6] = 7 字节 SN */
#define CAN_ID_MOTOR_BIND_SN        0x0313U   /* 绑定 SN：Data[0..6]=SN，Data[7]=设备号（8 字节）*/

/* --- 2. 查询指令 ID（主控发 → 电机）--- */
#define CAN_ID_MOTOR_QUERY          0x0408U   /* 统一查询入口（与 CAN_ID_MOTOR_CFG_WHEEL 同一个 ID，多用途），Data[1]区分查询类型 */
#define CAN_ID_MOTOR_QUERY_RESP     0x0409U   /* 电机查询应答 */

/* --- 3. 反馈帧 ID（电机发 → 主控）--- */
#define CAN_ID_MOTOR_ACK            0x041AU   /* 参数设置 ACK（2字节）*/
#define CAN_ID_MOTOR_FB_POS_BASE    0x0420U   /* 剩余位移反馈，实际ID = 0x0420 + 设备号（基址 + 设备号）*/
#define CAN_ID_MOTOR_FB_TORQUE_BASE 0x0440U   /* 力矩反馈，实际ID = 0x0440 + 设备号（基址 + 设备号）*/
#define CAN_ID_MOTOR_FB_HEART_BASE  0x0380U   /* 心跳反馈，实际ID = 0x0380 + 设备号（基址 + 设备号）*/
#define CAN_ID_MOTOR_ERR_BASE       0x0100U   /* 错误上报起始 ID，实际范围 0x0100~0x017F */
#define CAN_ID_MOTOR_ERR_END        0x017FU

/* 注意：0x0419（从电机目标位置）的 DLC=5，Data[4] 就是设备号，**没有目标类型字节**；
 *       目标类型 0x0A/0x0B 只存在于 0x0416（主电机，DLC=6，Data[5]=目标类型）。
 *       因此"设当前位置为某值"只能对使用主帧组的电机下发。 */

/* --- 控制指令中的参数 --- */
#define MOTOR_TARGET_RELATIVE       0x00U     /* 相对位置 */
#define MOTOR_TARGET_ABSOLUTE       0x05U     /* 绝对位置 */
#define MOTOR_TARGET_HOMING         0x0AU     /* 归零 */
#define MOTOR_TARGET_SET_CURRENT    0x0BU     /* 设当前位置为某值 */

#define MOTOR_LOCK_ENABLE           0x5AU     /* 上电使能 */
#define MOTOR_LOCK_RELEASE          0x4AU     /* 下电释放 */

/* --- 查询类型（对应 CAN_ID_MOTOR_QUERY 的 Data[1]）--- */
#define MOTOR_QUERY_TEMP            0x06U     /* 查询温度: 应答 Data[2..3] s16, 0.1℃ */
#define MOTOR_QUERY_POS             0x08U     /* 查询当前位置: 应答 Data[2..5] s32, 0.1mm */
#define MOTOR_QUERY_STATUS          0x25U     /* 查询状态: 应答 Data[2] 状态位（MOTOR_STATUS_*）*/
#define MOTOR_QUERY_VERSION         0xFFU     /* 查询固件版本号: 应答 Data[2..5] u32 */

/* --- 状态位（0x0409 应答 Data[2] 的解析）--- */
#define MOTOR_STATUS_LOCKED         0x01U     /* Bit0：已锁定（上电使能）*/
#define MOTOR_STATUS_ZEROED         0x02U     /* Bit1：在零位 */

/* --- 常见错误码 --- */
#define MOTOR_ERR_NOT_REACHED       0x0102U   /* 任务未完成，未达到目标点 */
#define MOTOR_ERR_STALL             0x0104U   /* 堵转 */
#define MOTOR_ERR_OVER_TEMP         0x0105U   /* 超温 */
#define MOTOR_ERR_UNDER_VOLTAGE     0x0107U   /* 欠压 */
#define MOTOR_ERR_DISCONNECT        0x0108U   /* 断线 */

/* --- 位移数据单位说明 --- */
/* 注意：所有位移单位均为 0.1mm，速度单位均为 0.1mm/s */
/*       R 为轮径（直径），数值单位 0.01mm（例：直径 100mm 时 R = 10000）。
 *       弧长 = π × 直径 × 角位移 / 360，换成 0.1mm 单位后：
 *       线位移(0.1mm) = 角位移(°) * π * R * 0.01 / 360 / 0.1 = 角位移(°) * π * R / 360 * 0.1
 *       核对：R = 10000，转 360° -> 3141.6（0.1mm）= 314.16mm = π × 100mm
 *       该换算公式后续在联动控制中使用 */

/* --- 反馈数据结构 --- */
typedef struct {
	int32_t main_pos;       /* 主通道位移，单位 0.1mm */
	int32_t slave_pos;      /* 从通道位移，单位 0.1mm */
} CanFrame_MotorPos_t;

typedef struct {
	int16_t torque;         /* 当前扭矩值，单位由电机决定 */
	uint8_t reserved[2];
} CanFrame_MotorTorque_t;

#endif /* __PROTOCOL_H */
