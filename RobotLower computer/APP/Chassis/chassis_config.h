#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include "bsp_config.h"

/*
 * 底盘标定/物理参数集中入口,风格对齐 APP/Roboticarm/RoboticArm.h 中的
 * ROBOTICARM_* 宏:只保留待标定的机械量和运动规划参数,不含 RTOS、
 * 超时、限速等安全状态机相关配置。标定后请在此处直接改数值。
 */

/* 物理车轮到 C620 电调 ID 的固定映射,数组顺序始终是 FL、FR、RL、RR。 */
#define CHASSIS_M3508_ID_FL (1U)
#define CHASSIS_M3508_ID_FR (2U)
#define CHASSIS_M3508_ID_RL (3U)
#define CHASSIS_M3508_ID_RR (4U)

/* ChassisUpdate 的固定控制周期,单位 ms。 */
#define CHASSIS_CONTROL_PERIOD_MS (3U)

/*
 * 麦轮运动学机械参数,待实测标定。
 * CHASSIS_WHEEL_RADIUS_M:带载后车轮有效半径,单位 m。
 * CHASSIS_HALF_WHEELBASE_M/HALF_TRACK_M:车体中心到前后/左右轮的距离,单位 m。
 * CHASSIS_GEAR_RATIO:电机轴转数/车轮转数,直驱填 1.0f。
 */
#define CHASSIS_WHEEL_RADIUS_M (0.079f)
#define CHASSIS_HALF_WHEELBASE_M (0.246f)
#define CHASSIS_HALF_TRACK_M (0.3144f)
#define CHASSIS_GEAR_RATIO (3591.0f / 187.0f)

/*
 * 车轮角速度 rad/s 与电机转速 rpm 之间的单位换算系数,配合 CHASSIS_GEAR_RATIO
 * 用于 chassis_mecanum 的逆解/正解,数学关系固定,无需标定。圆周率统一取自
 * bsp_config.h 的 BSP_PI。
 */
#define CHASSIS_MECANUM_PI BSP_PI
#define CHASSIS_MECANUM_RADPS_TO_RPM (60.0f / (2.0f * CHASSIS_MECANUM_PI))
#define CHASSIS_MECANUM_RPM_TO_RADPS ((2.0f * CHASSIS_MECANUM_PI) / 60.0f)

/* 电机安装方向修正,顺序固定为 FL、FR、RL、RR,只能填 +1 或 -1,待架空验证。 */
#define CHASSIS_MOTOR_DIRECTION_FL (1)
#define CHASSIS_MOTOR_DIRECTION_FR (1)
#define CHASSIS_MOTOR_DIRECTION_RL (1)
#define CHASSIS_MOTOR_DIRECTION_RR (1)

/*
 * 位移接口使用的七段 S 曲线参数。平移拆分为 x、y 两条完全独立的规划器,
 * 参数(下同一组宏)保持一致;偏航单独用一条规划器。
 * 平移单位为 m/s^2、m/s、m/s^3,偏航单位为 rad/s^2、rad/s、rad/s^3。
 */
#define CHASSIS_PLAN_TRANSLATION_MAX_ACCEL_MPS2 (3.8f)
#define CHASSIS_PLAN_TRANSLATION_MAX_SPEED_MPS (1.6f)
#define CHASSIS_PLAN_TRANSLATION_MAX_JERK_MPS3 (7.2f)

/* y轴速度规划备用参数组:test.c状态机在TEST_POSITION_STATE_START_MOVE_6起~
   WAIT_MOVE_8阶段将y轴S曲线规划器重新初始化为这一组上限,x轴不受影响,
   待整定。 */
#define CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_ACCEL_MPS2 (4.4f)
#define CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_SPEED_MPS (1.2f)
#define CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_JERK_MPS3 (10.2f)

#define CHASSIS_PLAN_YAW_MAX_ACCEL_RADPS2 (0.0f)
#define CHASSIS_PLAN_YAW_MAX_SPEED_RADPS (0.0f)
#define CHASSIS_PLAN_YAW_MAX_JERK_RADPS3 (0.0f)

/*
 * 位移接口跟踪器(PositionTrack)使用的位置跟踪PID参数。平移 x、y 两轴各自
 * 独立整定,偏航单独用一套,目标为规划位置、输出叠加到规划速度前馈上,
 * 待整定。
 */
#define CHASSIS_TRACK_TRANSLATION_X_KP (0.25f)
#define CHASSIS_TRACK_TRANSLATION_X_KI (0.00015f)
#define CHASSIS_TRACK_TRANSLATION_X_KD (0.1f)
#define CHASSIS_TRACK_TRANSLATION_X_MAX_OUT (0.04f)
#define CHASSIS_TRACK_TRANSLATION_X_MAX_IOUT (0.003f)

/* x跟踪备用参数组:test.c状态机在TEST_POSITION_STATE_START_MOVE_4/WAIT_MOVE_4
   阶段(大幅度x回退运动)临时切换到这一组,离开该阶段后切到下面第三组,
   待整定。 */
#define CHASSIS_TRACK_TRANSLATION_X_ALT_KP (0.4f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT_KI (0.0001f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT_KD (0.1f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT_MAX_OUT (0.11f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT_MAX_IOUT (0.003f)

/* x跟踪第三套参数组:test.c状态机在TEST_POSITION_STATE_START_MOVE_5起~
   WAIT_MOVE_8阶段切换到这一组,待整定。 */
#define CHASSIS_TRACK_TRANSLATION_X_ALT2_KP (0.75f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT2_KI (0.001f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT2_KD (0.15f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT2_MAX_OUT (0.3f)
#define CHASSIS_TRACK_TRANSLATION_X_ALT2_MAX_IOUT (0.015f)

#define CHASSIS_TRACK_TRANSLATION_KP (1.1f)
#define CHASSIS_TRACK_TRANSLATION_KI (0.00025f)
#define CHASSIS_TRACK_TRANSLATION_KD (0.1f)
#define CHASSIS_TRACK_TRANSLATION_MAX_OUT (0.4f)
#define CHASSIS_TRACK_TRANSLATION_MAX_IOUT (0.02f)

/* y跟踪备用参数组:test.c状态机在TEST_POSITION_STATE_START_MOVE_6起~
   WAIT_MOVE_8阶段切换到这一组,离开该区间(若有)可切回上面的默认组,
   待整定。 */
#define CHASSIS_TRACK_TRANSLATION_Y_ALT_KP (0.9f)
#define CHASSIS_TRACK_TRANSLATION_Y_ALT_KI (0.002f)
#define CHASSIS_TRACK_TRANSLATION_Y_ALT_KD (0.1f)
#define CHASSIS_TRACK_TRANSLATION_Y_ALT_MAX_OUT (0.4f)
#define CHASSIS_TRACK_TRANSLATION_Y_ALT_MAX_IOUT (0.04f)
/* PositionTrack死区,|实际位置-规划目标位置|小于该值(单位m)时不做PID补偿,
   只保留速度前馈,避免死区内里程计噪声/量化误差引起的抖动。 */
#define CHASSIS_TRACK_TRANSLATION_DEADBAND_M (0.01f)
#define CHASSIS_SPEEDPLAN_CONTROL_THRESHOLD (0.02f)

#define CHASSIS_TRACK_YAW_KP (11.5f)
#define CHASSIS_TRACK_YAW_KI (0.013f)
#define CHASSIS_TRACK_YAW_KD (4.0f)
#define CHASSIS_TRACK_YAW_MAX_OUT (1.55f)
#define CHASSIS_TRACK_YAW_MAX_IOUT (0.13f)
/* 同上,单位rad */
#define CHASSIS_TRACK_YAW_DEADBAND_RAD (0.0025f)

#endif
