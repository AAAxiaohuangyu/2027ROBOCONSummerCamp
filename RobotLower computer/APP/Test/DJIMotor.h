#ifndef __DJI_MOTOR_H_
#define __DJI_MOTOR_H_

#include "fdcan_common.h"

/*
DJI C620电调CAN通信协议(适用于C610/C620 + M2006/M3508系列电机)

控制帧(主控->电调,标准帧,DLC 8):
    0x200: 控制1~4号电调电流,DATA[2*(id-1)]为高8位,DATA[2*(id-1)+1]为低8位
    0x1FF: 控制5~8号电调电流,字段排布同上
    电流给定值范围 -16384~16384,对应电调输出转矩电流 -20~20A

反馈帧(电调->主控,标准帧,DLC 8,ID = 0x200 + 电调ID,默认1kHz发送):
    DATA[0-1]: 转子机械角度,高8位在前,范围0~8191对应0~360°
    DATA[2-3]: 转速,单位rpm
    DATA[4-5]: 实际转矩电流
    DATA[6]  : 电机温度,单位摄氏度
    DATA[7]  : 保留
*/

#define DJI_MOTOR_CTRL_ID_1TO4 0x200u     /* 1~4号电调电流控制帧ID */
#define DJI_MOTOR_CTRL_ID_5TO8 0x1FFu     /* 5~8号电调电流控制帧ID */
#define DJI_MOTOR_FEEDBACK_ID_BASE 0x200u /* 反馈帧ID = 该宏 + 电调ID */

#define DJI_MOTOR_ID_MIN 1u
#define DJI_MOTOR_ID_MAX 8u

#define DJI_MOTOR_CURRENT_RAW_MAX 16384 /* 电流给定值幅值上限,对应20A */

typedef struct
{
    uint16_t angle;       // 转子机械角度,0~8191对应0~360°
    int16_t speed_rpm;    // 转速,单位rpm
    int16_t current;      // 实际转矩电流(原始值)
    uint8_t temperature;  // 电机温度,单位摄氏度

    uint32_t update_cnt; // 累计收到反馈帧的次数,可用于判断电调是否离线
} DJIMotorFeedback_TypeDef;

typedef struct
{
    uint8_t id; // 电调ID,1~8
    DJIMotorFeedback_TypeDef feedback;
} DJIMotor_TypeDef;

/* 将motor登记为id号电调的反馈接收对象,登记后收到对应反馈帧会自动更新motor->feedback */
void DJIMotorRegister(DJIMotor_TypeDef *motor, uint8_t id);

/* 按协议将4个电调的电流给定值打包进8字节的CAN数据 */
void DJIMotorCurrentPack(uint8_t *tx_data, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

/* 打包并通过FDCAN_Handle发送电流控制帧,ctrl_id取DJI_MOTOR_CTRL_ID_1TO4或DJI_MOTOR_CTRL_ID_5TO8 */
void DJIMotorSendCurrent(FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t ctrl_id, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);

#endif
