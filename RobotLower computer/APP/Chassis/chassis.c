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

    SpeedPlanInit(&chassis->displacement_plan.translation,
                  CHASSIS_PLAN_TRANSLATION_MAX_ACCEL_MPS2,
                  CHASSIS_PLAN_TRANSLATION_MAX_SPEED_MPS,
                  CHASSIS_PLAN_TRANSLATION_MAX_JERK_MPS3);
    SpeedPlanInit(&chassis->displacement_plan.yaw,
                  CHASSIS_PLAN_YAW_MAX_ACCEL_RADPS2,
                  CHASSIS_PLAN_YAW_MAX_SPEED_RADPS,
                  CHASSIS_PLAN_YAW_MAX_JERK_RADPS3);

    PIDInit(&chassis->displacement_plan.translation.track_pid,
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

    chassis->displacement_plan.translation_target_m = 0.0f;
    chassis->displacement_plan.translation_direction_x = 0.0f;
    chassis->displacement_plan.translation_direction_y = 0.0f;
    chassis->displacement_plan.translation_start_x = 0.0f;
    chassis->displacement_plan.translation_start_y = 0.0f;
    chassis->displacement_plan.yaw_target_rad = 0.0f;
    chassis->displacement_plan.yaw_start_rad = 0.0f;

    chassis->velocity.vx_mps = 0.0f;
    chassis->velocity.vy_mps = 0.0f;
    chassis->velocity.wz_radps = 0.0f;
    chassis->velocity_mode = 1U;

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

void ChassisSetTranslation(Chassis_TypeDef *chassis, float dx_m, float dy_m)
{
    float distance_m = sqrtf(dx_m * dx_m + dy_m * dy_m);

    chassis->velocity_mode = 0U;
    chassis->displacement_plan.translation_target_m = distance_m;
    chassis->displacement_plan.translation_start_x = chassis->pose.x_m;
    chassis->displacement_plan.translation_start_y = chassis->pose.y_m;

    if (distance_m > 0.0f)
    {
        chassis->displacement_plan.translation_direction_x = dx_m / distance_m;
        chassis->displacement_plan.translation_direction_y = dy_m / distance_m;
    }
    else
    {
        /* dx_m、dy_m同时为0:目标位移为0,不存在方向,避免0/0产生NaN */
        chassis->displacement_plan.translation_direction_x = 0.0f;
        chassis->displacement_plan.translation_direction_y = 0.0f;
    }

    chassis->displacement_plan.translation.state = init;
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
        uint32_t wheel;

        if (chassis->velocity_mode == 0U)
        {
            float translation_actual;
            float translation_speed;
            float yaw_actual;

            translation_actual = (chassis->pose.x_m - chassis->displacement_plan.translation_start_x) * chassis->displacement_plan.translation_direction_x +
                                  (chassis->pose.y_m - chassis->displacement_plan.translation_start_y) * chassis->displacement_plan.translation_direction_y;
            yaw_actual = chassis->pose.yaw_rad - chassis->displacement_plan.yaw_start_rad;

            SpeedPlanUpdate(&chassis->displacement_plan.translation, translation_actual, chassis->displacement_plan.translation_target_m);
            SpeedPlanUpdate(&chassis->displacement_plan.yaw, yaw_actual, chassis->displacement_plan.yaw_target_rad);

            translation_speed = SpeedPlanTrack(&chassis->displacement_plan.translation, translation_actual);
            chassis->velocity.vx_mps = chassis->displacement_plan.translation_direction_x * translation_speed;
            chassis->velocity.vy_mps = chassis->displacement_plan.translation_direction_y * translation_speed;
            chassis->velocity.wz_radps = SpeedPlanTrack(&chassis->displacement_plan.yaw, yaw_actual);
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
    float error = chassis->displacement_plan.translation_target_m - chassis->displacement_plan.translation.s;

    if (error < 0.0f)
    {
        error = -error;
    }
    return (error <= tolerance_m);
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
