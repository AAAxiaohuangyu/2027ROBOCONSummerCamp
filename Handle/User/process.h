#ifndef __PROCESS_H
#define __PROCESS_H

#include <string.h>
#include <stdint.h>
#include "usart.h"
#include "adc.h"

#define HANDLE_USART            (&huart1)

#define KEY_SCAN_PERIOD_MS      (0.005f)  //按键扫描周期，s
#define KEY_PRESS_THRESHOLD     (0.005f)  //确认按键按下时间阈值,s
#define KEY_MODE_PORT           GPIOA
#define KEY_MODE_PIN            GPIO_PIN_1
#define KEY_STOP_PORT           GPIOA
#define KEY_STOP_PIN            GPIO_PIN_1
#define KEY_GRIP_PORT           GPIOA
#define KEY_GRIP_PIN            GPIO_PIN_1
#define KEY_RELEASE_PORT        GPIOA
#define KEY_RELEASE_PIN         GPIO_PIN_1
#define KEY_FORWARD_PORT        GPIOA
#define KEY_FORWARD_PIN         GPIO_PIN_1
#define KEY_BACKWARD_PORT       GPIOA
#define KEY_BACKWARD_PIN        GPIO_PIN_1
#define KEY_LIFT_PORT           GPIOA
#define KEY_LIFT_PIN            GPIO_PIN_1
#define KEY_DOWN_PORT           GPIOA
#define KEY_DOWN_PIN            GPIO_PIN_1
#define KEY_POS_FLIP_PORT       GPIOA
#define KEY_POS_FLIP_PIN        GPIO_PIN_1
#define KEY_NEG_FLIP_PORT       GPIOA
#define KEY_NEG_FLIP_PIN        GPIO_PIN_1

#define ADC_NUMBER              ADC1
#define ADC_ADDRESS             &hadc1
#define ADC_RESOLUTION          (4096.0f)
#define ADC_CENTER_VALUE        (2048.0f)
#define ADC_VREF                (3.3f)
#define ADC_CHANNELS            3U
#define ADC_SPEED_VX_INDEX      0U
#define ADC_SPEED_VY_INDEX      1U
#define ADC_OMEGA_INDEX         2U
#define ADC_MAX_SPEED_VX        (10.0f) //ADC映射的最大X轴速度
#define ADC_MAX_SPEED_VY        (10.0f) //ADC映射的最大Y轴速度
#define ADC_OMEGA               (5.0f) //底盘定速旋转
#define ADC_DEAD_ZONE           80U

typedef enum{
    KEY_IDLE = 0,
    KEY_TEMP_PRESSED,
    KEY_PRESSED
}KeyStatus;

// ADC读取的原始数据
typedef struct{
    uint32_t chassis_raw_vx;
    uint32_t chassis_raw_vy;
    uint32_t chassis_raw_omega;
}ADCRawData_t;

//经过映射处理的ADC数据
typedef struct{
    int32_t chassis_vx;
    int32_t chassis_vy;
    int32_t chassis_omega;
}ADCData_t;

//由按键确定的控制状态
typedef struct{
    uint8_t mode_switch;
    uint8_t emergency_stop;
    uint8_t arm_grip;
    uint8_t arm_release;
    uint8_t joint_forward;
    uint8_t joint_backward;
    uint8_t joint_lift;
    uint8_t joint_down;
    uint8_t joint_positive_flip;
    uint8_t joint_negative_flip;
}KeyData_t;

//按键对应编号
typedef enum{
    KEY_MODE = 0,
    KEY_STOP,
    KEY_GRIP,
    KEY_RELEASE,
    KEY_FORWARD,
    KEY_BACKWARD,
    KEY_LIFT,
    KEY_DOWN,
    KEY_POSITIVE_FLIP,
    KEY_NEGATIVE_FLIP
} HandleKey_t;

typedef struct{
    ADCData_t ChassisData;
    KeyData_t ArmData;
}SendData_t;

typedef struct{
    ADCData_t ChassisData;
    KeyData_t ArmData;
}ReceiveData_t;

void Handle_Init(void);
void HandleOrderProcess(SendData_t *sdata);

void HandleSendData(SendData_t *data);
void HandleReceiveData(SendData_t *data);

#endif
