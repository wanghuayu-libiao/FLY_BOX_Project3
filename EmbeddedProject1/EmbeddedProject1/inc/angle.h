/**
  ******************************************************************************
  * @file    angle.h
  * @brief   角度计算：读 XV7001BB 角速度 -> 扣零偏 -> 积分成角度（-180° ~ +180°）
  *          用法：XV7001bb_Init() 之后调用 Angle_Init()；
  *                然后每 ANGLE_SAMPLE_PERIOD_MS（1ms）严格周期调用一次 Angle_Step()，
  *                它内部做 ANGLE_DECIM 点平均，每凑满一个 10ms 块返回 1；
  *                其它任务通过 Angle_GetData() 取一致的快照。
  ******************************************************************************
  */

#ifndef __ANGLE_H
#define __ANGLE_H

#include <stdint.h>

#define ANGLE_SAMPLE_PERIOD_MS  1U      /* 陀螺采样周期：1kHz 过采样 */
#define ANGLE_OUTPUT_PERIOD_MS  10U     /* 输出（积分/上报）周期：10ms（与上位机 100Hz 对应） */

typedef struct
{
	float    angle_deg;     /* 角度，-180 ~ +180 ° */
	float    rate_dps;      /* 角速度（已扣零偏），°/s */
	float    temp_c;        /* 芯片温度，℃（约每 100ms 更新） */
	float    bias_dps;      /* 当前温度下使用的零偏，°/s */
	float    temp_coef;     /* 在线学到的零偏温度系数，°/s/℃ */
	uint16_t seq;           /* 采样序号，每次 Angle_Step() +1 */
	uint8_t  status;        /* 状态寄存器 0x04 读回值（约每 100ms 更新） */
	uint8_t  flags;         /* PROTO_FLAG_* 组合 */
} AngleData_t;

void Angle_Init(void);
uint8_t Angle_Step(void);               /* 每 1ms 调用；返回 1 表示完成一个 10ms 输出块（新数据已发布） */
void Angle_GetData(AngleData_t *out);

/* ---- ★ 标度因子 / 时间基 自检（陀螺仪到底准不准）----
 *
 *  angle_total_deg : 【不回绕】的累计角度。转 10 圈就是 3600° 左右，
 *                    不像 angle_deg 那样 -180~+180 循环，可以直接读。
 *  ppm             : 时间基误差 = (名义时间 - 实际时间) / 实际时间 × 1e6
 *                    名义时间 = 完成的 10ms 块数 × 10ms
 *                    实际时间 = HAL_GetTick() 差值
 *                    ±4300 ppm 就正好等于「每 360° 偏 1.55°」。
 *                    若 ppm 很小而增益仍偏，则误差一定来自 ANGLE_SCALE_LSB_DPS。
 *  secs            : 上电后经过的秒数
 *  blocks          : 完成的 10ms 块数
 */
void Angle_GetTotal(float *angle_total_deg, int16_t *ppm, uint16_t *secs, uint32_t *blocks);

/* 以下两个可在其它任务/中断里调用，实际动作在下一次 Angle_Step() 里执行 */
void Angle_RequestZero(void);           /* 当前角度置 0，并清除"饱和"标志 */
void Angle_RequestRecalibrate(void);    /* 重新标定零偏（期间需保持静止），完成后角度置 0 */

/* 零偏在线学习开关（默认 1 = 开，行为与以前完全一致）。
 * 车在走的时候要关掉：车真实的慢速跑偏（<0.15°/s）会被当成零偏学进去，
 * 陀螺仪就看不到跑偏，闭环也就不会去纠。
 * 只要某个 1s 窗口里出现过"关"，整个窗口都不参与学习（不会被半个窗口污染）。 */
void Angle_SetBiasLearning(uint8_t enable);

/* 角速度标度 ppm 修正：积分前把角速度乘以 (1 + ppm×1e-6)，默认 0（不修正）。
 * 陀螺仪读数偏大（转多少都比真实多）填负值，偏小填正值。
 * 只有显式调用才生效，上电默认 0，所以不调用的测试（如方形）行为不变。 */
void Angle_SetScalePpm(int32_t ppm);

#endif /* __ANGLE_H */
