#ifndef __CORE_H_
#define __CORE_H_

#include "RoboticArm.h"
#include "zigbee.h"
#include "flip.h"

/* 机器人整体状态机状态,占位,具体状态由后续任务流程补充 */
typedef enum
{
    ROBOT_STATE_IDLE = 0,
    
} RobotState_TypeDef;

/* 机器人整体状态,占位,具体字段(各子系统状态、标志位等)由后续补充 */
typedef struct
{
    RobotState_TypeDef state;
    RoboticArm_TypeDef roboticarm;
    ZigbeeHandle_TypeDef zigbee;
    FlipState_TypeDef flip_state;
} Robot_TypeDef;

extern Robot_TypeDef Robot;

/* 初始化机器人整体状态机,占位,具体内容由后续补充 */
void RobotInit(void);

/* 周期调用,推进机器人整体状态机,占位,具体内容由后续补充 */
void RobotStateUpdate(Robot_TypeDef *Robot);

#endif
