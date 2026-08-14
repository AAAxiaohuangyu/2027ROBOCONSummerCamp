#ifndef __CORE_H_
#define __CORE_H_

#include "RoboticArm.h"
#include "chassis.h"
#include "zigbee.h"
#include "flip.h"
#include "Pickup.h"

/* 机器人整体状态机状态,占位,具体状态由后续任务流程补充 */
typedef enum
{
    ROBOT_STATE_IDLE = 0, // 空闲状态
    ROBOT_STATE_MOVE1,    // 从启动区运动到翻转区
    ROBOT_STATE_FLIP,     // 翻转KFS
    ROBOT_STATE_MOVE2,    // 从翻转区返回启动区
    ROBOT_STATE_VISION_WAIT, // 等待视觉判断下一个KFS是否需要抓取(判断完成前保持当前位置)
    ROBOT_STATE_MOVE3,    // 从启动区运动到第一个正确KFS位置(默认运动到KFS1,由视觉判断是否需要跳过默认位置)
    ROBOT_STATE_MOVE4,    // 运动到第二个正确的FKS位置(默认运动到ROBOT_STATE_MOVE3的后一个位置,由视觉判断是否需要跳过默认位置)
    ROBOT_STATE_MOVE5,    // 运动到第三个正确的FKS位置(默认运动到ROBOT_STATE_MOVE4的后一个位置,由视觉判断是否需要跳过默认位置)
    ROBOT_STATE_PICKUP,   // 拾取KFS
    ROBOT_STATE_MOVE6,    // 上斜坡
    ROBOT_STATE_MANUAL,   // 手动操作模式

} RobotState_TypeDef;

/* 场地KFS工位数量,固定为4个,从1开始编号 */
#define ROBOT_KFS_COUNT (4U)

/* 各段底盘相对位移(ChassisSetTranslation的dx/dy)与到位判定容差,占位,
   按实际场地尺寸标定后直接改这里的数值 */
#define ROBOT_MOVE_START_TO_FLIP_X (0.0f) /* 启动区->翻转区 */
#define ROBOT_MOVE_START_TO_FLIP_Y (0.0f)
#define ROBOT_MOVE_FLIP_TO_START_X (0.0f) /* 翻转区->启动区 */
#define ROBOT_MOVE_FLIP_TO_START_Y (0.0f)
#define ROBOT_MOVE_START_TO_KFS1_X (0.0f) /* 启动区->KFS1(相对启动区) */
#define ROBOT_MOVE_START_TO_KFS1_Y (0.0f)
#define ROBOT_MOVE_KFS_STEP_X (0.0f) /* 相邻KFS工位间距 */
#define ROBOT_MOVE_KFS_STEP_Y (0.0f)
#define ROBOT_MOVE_UP_SLOPE_X (0.0f) /* 最后一个KFS工位->斜坡上方 */
#define ROBOT_MOVE_UP_SLOPE_Y (0.0f)
#define ROBOT_CHASSIS_POSITION_TOLERANCE_M (0.01f)

/* 顶层状态机周期,单位ms */
#define ROBOT_STATE_UPDATE_PERIOD_MS (5U)

/* 机器人整体状态,占位,具体字段(各子系统状态、标志位等)由后续补充 */
typedef struct
{
    RobotState_TypeDef state;
    RoboticArm_TypeDef roboticarm;
    Chassis_TypeDef chassis;
    ZigbeeHandle_TypeDef zigbee;
    FlipState_TypeDef flip_state;
    PickupState_TypeDef pick_state;

    uint8_t kfs_last_index;         /* 上一次到位的KFS序号,1~ROBOT_KFS_COUNT,0表示仍在启动区 */
    uint8_t kfs_target_index;       /* 当前目标KFS序号,由视觉判断结果给出 */
    RobotState_TypeDef pickup_return_state; /* PICKUP完成后应返回的状态(MOVE4/MOVE5/MOVE6) */

    RobotState_TypeDef vision_next_state; /* VISION_WAIT判断完成后应跳转的状态(MOVE3/MOVE4/MOVE5) */
    uint8_t vision_request_sent;    /* 本轮视觉判断请求是否已发出,避免重复请求 */
} Robot_TypeDef;

extern Robot_TypeDef Robot;

/* 初始化机器人整体状态机,占位,具体内容由后续补充 */
void RobotInit(void);

/* 周期调用,推进机器人整体状态机,占位,具体内容由后续补充 */
void RobotStateUpdate(Robot_TypeDef *Robot);

/* 视觉判断下一个应抓取的KFS序号(1~ROBOT_KFS_COUNT)是否需要:第一个KFS在
   MOVE2回到启动区后即可判断,之后每个KFS必须等上一个KFS拾取完成后才能判
   断下一个是否需要抓取。视觉判断需要时间,不能阻塞在原地等待固定时长,因此
   拆成"发起请求/轮询是否完成"两个接口,由ROBOT_STATE_VISION_WAIT状态周期
   调用:
   - RobotVisionRequestKfsIndex:进入等待状态时调用一次,发起本次判断请求,
     default_index为按顺序推荐的默认值。
   - RobotVisionKfsIndexReady:此后每个周期调用,判断尚未完成时返回0(不改
     动*result);判断完成时返回非0,并通过*result给出最终序号——需要则为
     default_index,不需要则为跳过后的序号。
   本文件只提供请求后立即完成、直接采用default_index的默认实现,具体判断
   逻辑由视觉模块提供强定义覆盖 */
__weak void RobotVisionRequestKfsIndex(uint8_t default_index);
__weak uint8_t RobotVisionKfsIndexReady(uint8_t *result);

#endif
