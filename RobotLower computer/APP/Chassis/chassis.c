#include "chassis.h"

#include <math.h>

#include "chassis_config.h"
#include "cmsis_os2.h"

void ChassisInit(Chassis_TypeDef *chassis, FDCAN_HandleTypeDef *can_handle, uint16_t ctrl_id)
{
    ChassisMecanum_Config_t mecanum_config;

    M3508GroupInit(&chassis->drive.motor_group, can_handle, ctrl_id);

    chassis->drive.motor_id[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] = CHASSIS_M3508_ID_FL;
    chassis->drive.motor_id[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] = CHASSIS_M3508_ID_FR;
    chassis->drive.motor_id[CHASSIS_MECANUM_WHEEL_REAR_LEFT] = CHASSIS_M3508_ID_RL;
    chassis->drive.motor_id[CHASSIS_MECANUM_WHEEL_REAR_RIGHT] = CHASSIS_M3508_ID_RR;

    mecanum_config.wheel_radius_m = CHASSIS_WHEEL_RADIUS_M;
    mecanum_config.half_wheelbase_m = CHASSIS_HALF_WHEELBASE_M;
    mecanum_config.half_track_m = CHASSIS_HALF_TRACK_M;
    mecanum_config.gear_ratio = CHASSIS_GEAR_RATIO;
    mecanum_config.motor_direction[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] = CHASSIS_MOTOR_DIRECTION_FL;
    mecanum_config.motor_direction[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] = CHASSIS_MOTOR_DIRECTION_FR;
    mecanum_config.motor_direction[CHASSIS_MECANUM_WHEEL_REAR_LEFT] = CHASSIS_MOTOR_DIRECTION_RL;
    mecanum_config.motor_direction[CHASSIS_MECANUM_WHEEL_REAR_RIGHT] = CHASSIS_MOTOR_DIRECTION_RR;
    ChassisMecanum_Init(&chassis->drive.mecanum, &mecanum_config);

    SpeedPlanInit(&chassis->displacement_plan.translation_x,
                  CHASSIS_PLAN_TRANSLATION_MAX_ACCEL_MPS2,
                  CHASSIS_PLAN_TRANSLATION_MAX_SPEED_MPS,
                  CHASSIS_PLAN_TRANSLATION_MAX_JERK_MPS3,
                  CHASSIS_TRACK_TRANSLATION_DEADBAND_M);
    SpeedPlanInit(&chassis->displacement_plan.translation_y,
                  CHASSIS_PLAN_TRANSLATION_MAX_ACCEL_MPS2,
                  CHASSIS_PLAN_TRANSLATION_MAX_SPEED_MPS,
                  CHASSIS_PLAN_TRANSLATION_MAX_JERK_MPS3,
                  CHASSIS_TRACK_TRANSLATION_DEADBAND_M);
    SpeedPlanInit(&chassis->displacement_plan.yaw,
                  CHASSIS_PLAN_YAW_MAX_ACCEL_RADPS2,
                  CHASSIS_PLAN_YAW_MAX_SPEED_RADPS,
                  CHASSIS_PLAN_YAW_MAX_JERK_RADPS3,
                  CHASSIS_TRACK_YAW_DEADBAND_RAD);

    PIDInit(&chassis->displacement_plan.translation_x.track_pid,
            CHASSIS_TRACK_TRANSLATION_X_KP,
            CHASSIS_TRACK_TRANSLATION_X_KI,
            CHASSIS_TRACK_TRANSLATION_X_KD,
            CHASSIS_TRACK_TRANSLATION_X_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_X_MAX_IOUT);
    PIDInit(&chassis->displacement_plan.translation_y.track_pid,
            CHASSIS_TRACK_TRANSLATION_KP,
            CHASSIS_TRACK_TRANSLATION_KI,
            CHASSIS_TRACK_TRANSLATION_KD,
            CHASSIS_TRACK_TRANSLATION_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_MAX_IOUT);
    PIDInit(&chassis->displacement_plan.yaw.track_pid,
            CHASSIS_TRACK_YAW_KP,
            CHASSIS_TRACK_YAW_KI,
            CHASSIS_TRACK_YAW_KD,
            CHASSIS_TRACK_YAW_MAX_OUT,
            CHASSIS_TRACK_YAW_MAX_IOUT);

    chassis->displacement_plan.translation_target_x_m = 0.0f;
    chassis->displacement_plan.translation_target_y_m = 0.0f;
    chassis->displacement_plan.yaw_target_rad = 0.0f;
    chassis->displacement_plan.yaw_start_rad = 0.0f;

    chassis->velocity.vx_mps = 0.0f;
    chassis->velocity.vy_mps = 0.0f;
    chassis->velocity.wz_radps = 0.0f;
    chassis->velocity_mode = 1U;

    chassis->actual_velocity.vx_mps = 0.0f;
    chassis->actual_velocity.vy_mps = 0.0f;
    chassis->actual_velocity.wz_radps = 0.0f;

    chassis->pose.x_m = 0.0f;
    chassis->pose.y_m = 0.0f;
    chassis->pose.yaw_rad = 0.0f;
}

void ChassisSetVelocity(Chassis_TypeDef *chassis, float vx_mps, float vy_mps, float wz_radps)
{
    chassis->velocity_mode = 1U;
    chassis->velocity.vx_mps = vx_mps;
    chassis->velocity.vy_mps = vy_mps;
    chassis->velocity.wz_radps = wz_radps;
}

void ChassisStop(Chassis_TypeDef *chassis)
{
    chassis->velocity_mode = 1U;
    chassis->velocity.vx_mps = 0.0f;
    chassis->velocity.vy_mps = 0.0f;
    chassis->velocity.wz_radps = 0.0f;
}

void ChassisSetTranslation(Chassis_TypeDef *chassis, float x_target_m, float y_target_m)
{
    chassis->velocity_mode = 0U;

    if (fabsf(x_target_m - chassis->displacement_plan.translation_target_x_m) > CHASSIS_SPEEDPLAN_CONTROL_THRESHOLD)
        chassis->displacement_plan.translation_x.state = init;
    if (fabsf(y_target_m - chassis->displacement_plan.translation_target_y_m) > CHASSIS_SPEEDPLAN_CONTROL_THRESHOLD)
        chassis->displacement_plan.translation_y.state = init;

    chassis->displacement_plan.translation_target_x_m = x_target_m;
    chassis->displacement_plan.translation_target_y_m = y_target_m;
}

void ChassisSetYaw(Chassis_TypeDef *chassis, float dyaw_rad)
{
    chassis->velocity_mode = 0U;
    chassis->displacement_plan.yaw_target_rad = dyaw_rad;
    chassis->displacement_plan.yaw_start_rad = chassis->pose.yaw_rad;

    chassis->displacement_plan.yaw.state = init;
}

void ChassisSetPosition(Chassis_TypeDef *chassis, float x_m, float y_m)
{
    chassis->pose.x_m = x_m;
    chassis->pose.y_m = y_m;
}

void ChassisUpdate(Chassis_TypeDef *chassis)
{
    while (1)
    {
        ChassisMecanum_BodyVelocity_t body_velocity;
        ChassisMecanum_MotorCommand_t motor_command;
        float motor_rpm_actual[CHASSIS_MECANUM_WHEEL_COUNT];
        uint32_t wheel;

        for (wheel = 0U; wheel < CHASSIS_MECANUM_WHEEL_COUNT; ++wheel)
        {
            motor_rpm_actual[wheel] = M3508GroupGetSpeed(&chassis->drive.motor_group, chassis->drive.motor_id[wheel]);
        }
        ChassisMecanum_Forward(&chassis->drive.mecanum, motor_rpm_actual, &chassis->actual_velocity);

        if (chassis->velocity_mode == 0U)
        {
            float yaw_actual;

            yaw_actual = chassis->pose.yaw_rad - chassis->displacement_plan.yaw_start_rad;

            SpeedPlanUpdate(&chassis->displacement_plan.translation_x, chassis->pose.x_m,
                                 chassis->displacement_plan.translation_target_x_m);
            SpeedPlanUpdate(&chassis->displacement_plan.translation_y, chassis->pose.y_m,
                                 chassis->displacement_plan.translation_target_y_m);
            SpeedPlanUpdate(&chassis->displacement_plan.yaw, yaw_actual, chassis->displacement_plan.yaw_target_rad);

            /* 全局跟踪 */
            chassis->velocity.vx_mps = PositionTrack(&chassis->displacement_plan.translation_x, chassis->pose.x_m);
            chassis->velocity.vy_mps = PositionTrack(&chassis->displacement_plan.translation_y, chassis->pose.y_m);
            chassis->velocity.wz_radps = PositionTrack(&chassis->displacement_plan.yaw, yaw_actual);
        }

        body_velocity.vx_mps = chassis->velocity.vx_mps;
        body_velocity.vy_mps = chassis->velocity.vy_mps;
        body_velocity.wz_radps = chassis->velocity.wz_radps;

        ChassisMecanum_Inverse(&chassis->drive.mecanum, &body_velocity, &motor_command);
        for (wheel = 0U; wheel < CHASSIS_MECANUM_WHEEL_COUNT; ++wheel)
        {
            M3508GroupSetTarget(&chassis->drive.motor_group, chassis->drive.motor_id[wheel], motor_command.motor_rpm[wheel]);
        }

        M3508GroupUpdate(&chassis->drive.motor_group);
        osDelay(CHASSIS_CONTROL_PERIOD_MS);
    }
}

uint8_t ChassisTranslationReached(Chassis_TypeDef *chassis, float tolerance_m)
{
    float error_x = chassis->displacement_plan.translation_target_x_m - chassis->pose.x_m;
    float error_y = chassis->displacement_plan.translation_target_y_m - chassis->pose.y_m;

    return (sqrtf(error_x * error_x + error_y * error_y) <= tolerance_m);
}

uint8_t ChassisYawReached(Chassis_TypeDef *chassis, float tolerance_rad)
{
    float error = fabsf(chassis->displacement_plan.yaw_target_rad) - chassis->displacement_plan.yaw.s;

    if (error < 0.0f)
    {
        error = -error;
    }
    return (error <= tolerance_rad);
}
