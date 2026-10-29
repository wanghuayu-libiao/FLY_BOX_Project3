/**
  ******************************************************************************
  * @file    car.h
  * @brief   小车运动层（基础版）：航向展开 + 里程计 + 运动状态机
  *
  *  本层只做"骨架"：单位换算、姿态跟踪、里程累计、直线/转向的执行与到位判定。
  *  不含 PID 调参，也不含航向闭环纠偏（那属于后续 S4）。
  *
  *  ★ 与课题一的区别：陀螺仪测的是车体航向 θ，电机反馈测的是轮子里程 s，
  *    两者不再同轴可比，必须经运动学耦合：
  *        车体前进距离 = (左轮位移 + 右轮位移) / 2
  *
  *  用法：
  *      Car_Init();
  *      每 CAR_UPDATE_PERIOD_MS（20ms）：
  *          Angle_Step() 由 Task_Angle 负责；
  *          Car_Update();                       // 更新航向/里程、推进状态机
  *      发命令：
  *          Car_StartStraight(50000, 2000, 2000);   // 5m，200mm/s，200mm/s^2
  *          Car_StartTurn(180.0f, 1500, 2000);      // 原地转 180°
  *
  *  单位：位移 0.1mm，速度 0.1mm/s，加速度 0.1mm/s^2，角度 °
  ******************************************************************************
  */

#ifndef __CAR_H
#define __CAR_H

#include <stdint.h>
#include "car_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 运动状态 */
typedef enum
{
	CAR_IDLE = 0,       /* 空闲 */
	CAR_MOVING,         /* 直线段执行中 */
	CAR_TURNING,        /* 原地转向执行中 */
	CAR_DONE,           /* 上一段已完成（等调用方取结果） */
	CAR_ABORTED         /* 超时 / 发送失败 / 无反馈 */
} CarState_t;

/* ------------------------------------------------------------------ 初始化/周期 */

void Car_Init(void);

/* 周期调用（CAR_UPDATE_PERIOD_MS）：更新航向、里程，推进状态机 */
void Car_Update(void);

uint8_t Car_IsBusy(void);                   /* 1 = 正在执行某段运动 */

/* ------------------------------------------------------------------ 姿态 */

/* 连续航向（°），已展开：可超过 ±180，用于多圈/长距离任务的误差判断。
 * 这是 Angle 的 ±180° 输出累加 unwrap 后的结果。 */
float Car_GetYaw(void);

/* 把当前航向清零（之后 Car_GetYaw() 从 0 开始）*/
void Car_ResetYaw(void);

/* 角度归一化到 (-180, +180]，本层内部/外部都用它 */
float Car_Wrap180(float deg);

/* ------------------------------------------------------------------ 里程计 */

/* 左右轮累计位移，0.1mm（正 = 该轮正方向滚过的距离）*/
int32_t Car_GetOdoLeft(void);
int32_t Car_GetOdoRight(void);

/* 车体前进距离，0.1mm = (左 + 右) / 2 */
int32_t Car_GetOdoForward(void);

/* 里程计归零（同时把电机侧的基准对齐，不会产生跳变）*/
void Car_ResetOdometer(void);

/* 是否有可用的剩余位移反馈（没有就只能靠超时判停）*/
uint8_t Car_HasWheelFeedback(void);

/* 陀螺仪是否在持续更新（靠 Angle 的 seq 是否变化判断）*/
uint8_t Car_GyroAlive(void);

/* ------------------------------------------------------------------ 单位换算 */

/* 轮子转角(°) -> 线位移(0.1mm)：d = deg × π × R / 360 × 0.1，R 单位 0.01mm */
int32_t Car_DegToLinear0p1mm(float deg, uint16_t wheel_0p01mm);

/* 线位移(0.1mm) -> 轮子转角(°)：上式的逆运算 */
float Car_LinearToDeg(int32_t linear_0p1mm, uint16_t wheel_0p01mm);

/* 原地转向时每个轮子要走的弧长(0.1mm)：arc = B × deg × π / 360 × ... 
 * 推导：每轮位移 = (轮距/2) × θ_rad，θ_rad = deg × π / 180
 *       -> mm = B/2 × deg × π / 180 = B × deg × π / 360
 *       -> 0.1mm = B × deg × π / 36
 * B 单位 mm。返回值恒为正，方向由调用方按左右分配。 */
int32_t Car_TurnArc0p1mm(float deg, float wheel_base_mm);

/* ------------------------------------------------------------------ 运动指令 */
/* 返回 0 = 已接受，1 = 拒绝（参数非法 / 电机未就绪 / 正在忙）*/

/* 直线：两轮同向走 dist_0p1mm（负值 = 后退）*/
uint8_t Car_StartStraight(int32_t dist_0p1mm, uint32_t speed_0p1mmps, uint32_t acc_0p1mmps2);

/* 原地转向：deg 为车体转角（正 = 逆时针，负 = 顺时针）。
 * 两轮反向等量转动；到位判据用陀螺仪航向（轮子只做前馈）。 */
uint8_t Car_StartTurn(float deg, uint32_t speed_0p1mmps, uint32_t acc_0p1mmps2);

/* 停车：给两轮发"相对位移 0"，让电机原地保持（不释放，避免溜车）*/
void Car_Stop(void);

CarState_t Car_GetState(void);
const char *Car_GetStateName(void);

/* 上一段运动的执行结果（供流程层判断）：
 *   期望值 vs 实际值，单位 0.1mm / ° */
int32_t Car_GetLastTarget(void);
int32_t Car_GetLastActual(void);

/* ------------------------------------------------------------------ Watch 变量 */
extern volatile uint8_t  g_car_state;         /* CarState_t */
extern volatile float    g_car_yaw;           /* 连续航向 ° */
extern volatile float    g_car_yaw_raw;       /* 陀螺仪原始 ±180° 输出 */
extern volatile int32_t  g_car_odo_left;      /* 左轮里程 0.1mm */
extern volatile int32_t  g_car_odo_right;     /* 右轮里程 0.1mm */
extern volatile int32_t  g_car_odo_forward;   /* 车体前进距离 0.1mm */
extern volatile int32_t  g_car_remain_left;   /* 左轮剩余位移 0.1mm */
extern volatile int32_t  g_car_remain_right;  /* 右轮剩余位移 0.1mm */
extern volatile int32_t  g_car_seg_target;    /* 本段目标值（位移 0.1mm 或角度×100）*/
extern volatile int32_t  g_car_seg_actual;    /* 本段实际值 */
extern volatile uint32_t g_car_elapsed_ms;    /* 本段已耗时 */
extern volatile uint8_t  g_car_abort_reason;  /* 0=正常 1=超时 2=发送失败 3=无反馈 */

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H */
