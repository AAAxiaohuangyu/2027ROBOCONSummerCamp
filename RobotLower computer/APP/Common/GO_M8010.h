#ifndef __GO_M8010_H__
#define __GO_M8010_H__

#include "main.h"
#include "usart.h"
#include "StrategyAlogrithm.h"
#include "bsp_config.h"
#include <stdint.h>

#define PI2 (2.0f * BSP_PI)

#define GO_M8010_REDUCTION_RATIO 6.33f

#define GO_M8010_POS_CTRL_A_MAX 15.0f
#define GO_M8010_POS_CTRL_V_MAX 4.0f
#define GO_M8010_POS_CTRL_J     25.0f
#define GO_M8010_POS_CTRL_KP    0.55f
#define GO_M8010_POS_CTRL_KD    0.2f

#define GO_M8010_VEL_CTRL_KP 0.0f
#define GO_M8010_VEL_CTRL_KD 0.3f

#define GO_M8010_CONTROL_FRAME_SIZE  17U
#define GO_M8010_FEEDBACK_FRAME_SIZE 16U
#define GO_M8010_RX_DMA_BUFFER_SIZE  32U

typedef struct
{
    uint8_t bytes[GO_M8010_CONTROL_FRAME_SIZE];
} GOM8010ControlPacket_TypeDef;

typedef struct
{
    uint8_t bytes[GO_M8010_RX_DMA_BUFFER_SIZE];
} GOM8010FeedbackPacket_TypeDef;

typedef enum
{
    GOM8010_CTRL_MODE_POSITION = 0,
    GOM8010_CTRL_MODE_VELOCITY = 1,
} GOM8010CtrlMode_TypeDef;

typedef struct
{
    float a_max;
    float v_max;
    float j;
    float kp;
    float kd;
} GOM8010PositionCtrlParam_TypeDef;

typedef struct
{
    float kp;
    float kd;
} GOM8010VelocityCtrlParam_TypeDef;

typedef struct
{
    uint8_t mode;
    uint8_t timeout;
    float torque;
    float speed;
    float position;
    float kp;
    float kd;
    SpeedPlan_TypeDef plan;
    float position_target;
    float velocity_target;
    float torque_feedforward;
    GOM8010CtrlMode_TypeDef ctrl_mode;
    GOM8010PositionCtrlParam_TypeDef position_param;
    GOM8010VelocityCtrlParam_TypeDef velocity_param;
    GOM8010ControlPacket_TypeDef packet;
} GOM8010Control_TypeDef;

typedef struct
{
    GOM8010FeedbackPacket_TypeDef packet;
    float position;
    float position_offset;
    uint8_t position_initialized;
} GOM8010Feedback_TypeDef;

typedef struct
{
    uint8_t id;
    UART_HandleTypeDef *huart;
    GOM8010Control_TypeDef control;
    GOM8010Feedback_TypeDef feedback;
} GOM8010Motor_TypeDef;

void GOM8010MotorInit(GOM8010Motor_TypeDef *motor, uint8_t id, UART_HandleTypeDef *huart);
void GOM8010MotorSetTarget(GOM8010Motor_TypeDef *motor, float position_target);
void GOM8010MotorSetVelocityTarget(GOM8010Motor_TypeDef *motor, float velocity_target);
void GOM8010MotorSetTorqueFeedforward(GOM8010Motor_TypeDef *motor, float torque_feedforward);
void GOM8010MotorUpdate(GOM8010Motor_TypeDef *motor);
void GOM8010MotorRxEvent(GOM8010Motor_TypeDef *motor, UART_HandleTypeDef *huart, uint16_t size);
void GOM8010MotorRxErrorEvent(GOM8010Motor_TypeDef *motor, UART_HandleTypeDef *huart);

#endif /* __GO_M8010_H__ */
