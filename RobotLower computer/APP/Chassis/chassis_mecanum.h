#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 四轮 X 形麦克纳姆轮底盘运动学模块，不直接控制电机、不发 CAN 报文。
 * 车体坐标系：+X 前，+Y 左，+Wz 从车顶看逆时针。
 * 轮序固定为 FL、FR、RL、RR，与电机驱动层保持一致。
 *
 * ChassisMecanum_Inverse(): 车体速度 Vx/Vy/Wz -> 四个电机目标 rpm。
 * ChassisMecanum_Forward(): 四个电机反馈 rpm -> 估算车体速度。
 *
 * 只处理麦轮尺寸相同、45 度滚子、矩形四轮布局、X 形安装的理想模型，
 * 不补偿打滑等误差；电机 ID、FDCAN 等硬件参数由 chassis.c 负责。
 */
#define CHASSIS_MECANUM_WHEEL_COUNT  (4U)

/* 车轮角速度 rad/s 与电机转速 rpm 之间的单位换算系数,定义见 chassis_config.h。 */

/* 数组下标固定为 FL、FR、RL、RR。 */
typedef enum
{
    CHASSIS_MECANUM_WHEEL_FRONT_LEFT = 0,
    CHASSIS_MECANUM_WHEEL_FRONT_RIGHT,
    CHASSIS_MECANUM_WHEEL_REAR_LEFT,
    CHASSIS_MECANUM_WHEEL_REAR_RIGHT
} ChassisMecanum_Wheel_t;

/* 底盘机械参数，长度单位米。 */
typedef struct
{
    float wheel_radius_m;       /* 麦轮带载有效半径。 */
    float half_wheelbase_m;     /* 轴距的一半。 */
    float half_track_m;         /* 轮距的一半。 */
    float gear_ratio;           /* 电机转数/车轮转数，直驱填 1.0f。 */
    int8_t motor_direction[CHASSIS_MECANUM_WHEEL_COUNT]; /* 每项只能为 +1 或 -1。 */
} ChassisMecanum_Config_t;

/*
 * motor_direction[] 的确定方法：先都填 +1，架空底盘发送很小的纯前进
 * 命令，方向不对的轮子改为 -1。不要通过交换数组位置来修正方向。
 */

/* 车体目标速度。 */
typedef struct
{
    float vx_mps;   /* 前后速度，m/s，正值前进。 */
    float vy_mps;   /* 横移速度，m/s，正值左移。 */
    float wz_radps; /* 自转角速度，rad/s，正值逆时针。 */
} ChassisMecanum_BodyVelocity_t;

/* 逆解结果，可直接作为电机速度闭环目标。 */
typedef struct
{
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT]; /* 顺序固定为 FL、FR、RL、RR。 */
} ChassisMecanum_MotorCommand_t;

/* 解算器实例。 */
typedef struct
{
    ChassisMecanum_Config_t config;
} ChassisMecanum_t;

/* 保存机械参数。 */
void ChassisMecanum_Init(
    ChassisMecanum_t *mecanum,
    const ChassisMecanum_Config_t *config);

/* 逆运动学：车体速度 -> 四个电机目标转速。 */
void ChassisMecanum_Inverse(
    const ChassisMecanum_t *mecanum,
    const ChassisMecanum_BodyVelocity_t *body_velocity,
    ChassisMecanum_MotorCommand_t *motor_command);

/* 正运动学：四个电机反馈转速 -> 估算车体速度。 */
void ChassisMecanum_Forward(
    const ChassisMecanum_t *mecanum,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    ChassisMecanum_BodyVelocity_t *body_velocity);

#ifdef __cplusplus
}
#endif

#endif
