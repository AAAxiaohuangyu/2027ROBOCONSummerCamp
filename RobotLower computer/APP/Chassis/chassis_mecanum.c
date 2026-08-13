#include "chassis_mecanum.h"

#include "chassis_config.h"

void ChassisMecanum_Init(
    ChassisMecanum_t *mecanum,
    const ChassisMecanum_Config_t *config)
{
    mecanum->config = *config;
}

void ChassisMecanum_Inverse(
    const ChassisMecanum_t *mecanum,
    const ChassisMecanum_BodyVelocity_t *body_velocity,
    ChassisMecanum_MotorCommand_t *motor_command)
{
    float wheel_radps[CHASSIS_MECANUM_WHEEL_COUNT];
    float rotation_arm_m;
    uint32_t index;

    /* 自转项等效力臂 L = 半轴距 + 半轮距。 */
    rotation_arm_m = mecanum->config.half_wheelbase_m +
                     mecanum->config.half_track_m;

    /*
     * X 形麦轮逆运动学，得到每个车轮角速度 rad/s：
     *   FL = (Vx - Vy - L*Wz) / r      FR = (Vx + Vy + L*Wz) / r
     *   RL = (Vx + Vy - L*Wz) / r      RR = (Vx - Vy + L*Wz) / r
     */
    wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] =
        (body_velocity->vx_mps - body_velocity->vy_mps -
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] =
        (body_velocity->vx_mps + body_velocity->vy_mps +
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] =
        (body_velocity->vx_mps + body_velocity->vy_mps -
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT] =
        (body_velocity->vx_mps - body_velocity->vy_mps +
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;

    /* 车轮 rad/s -> 电机轴 rpm：乘减速比、单位换算、安装方向修正。 */
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        motor_command->motor_rpm[index] =
            wheel_radps[index] * mecanum->config.gear_ratio *
            CHASSIS_MECANUM_RADPS_TO_RPM *
            (float)mecanum->config.motor_direction[index];
    }
}

void ChassisMecanum_Forward(
    const ChassisMecanum_t *mecanum,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    ChassisMecanum_BodyVelocity_t *body_velocity)
{
    /* 输入须为电机轴 rpm，而非车轮 rpm。 */
    float wheel_radps[CHASSIS_MECANUM_WHEEL_COUNT];
    float rotation_arm_m;
    uint32_t index;

    /* 撤销安装方向和减速比，还原为车轮 rad/s。 */
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        wheel_radps[index] = motor_rpm[index] *
                             (float)mecanum->config.motor_direction[index] *
                             CHASSIS_MECANUM_RPM_TO_RADPS /
                             mecanum->config.gear_ratio;
    }

    rotation_arm_m = mecanum->config.half_wheelbase_m +
                     mecanum->config.half_track_m;

    /* 逆解矩阵的反变换，估算车体速度（理想无滑动模型）。 */
    body_velocity->vx_mps = mecanum->config.wheel_radius_m * 0.25f *
        (wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]);
    body_velocity->vy_mps = mecanum->config.wheel_radius_m * 0.25f *
        (-wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] -
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]);
    body_velocity->wz_radps = mecanum->config.wheel_radius_m *
        (-wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] -
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]) /
        (4.0f * rotation_arm_m);
}
