#ifndef __CORE_H_
#define __CORE_H_

#include "RoboticArm.h"
#include "chassis.h"
#include "encoder.h"
#include "zigbee.h"
#include "vision.h"
#include "yis512.h"
#include "Servo.h"
#include "flip.h"
#include "Pickup.h"

/* 转弯提前系数(无量纲):RobotStateUpdateTask处于WAIT_MOVE_X状态时,判断是否可以
   切换到START_MOVE_Y所用的容差为该系数乘以x轴当前速度对应的S曲线减速距离
   (SpeedPlanDecelDistance),而非固定的米数,因此能随速度规划参数自动适配;
   k=1表示x刚进入减速阶段就切换,转弯越圆滑需要越大的k,但也会让实际路径更早
   偏离中间点 */
#define ROBOT_STATE_CORNER_BLEND_K (1.7f)

/* MOVE_9(上斜坡)：码盘 y 轴世界系累计位置(Robot.encoder.y_m,与 chassis->pose.y_m
   同一坐标系)到达该值附近即认为冲坡到位,暂定,需实测标定后手动修改 */
#define ROBOT_STATE_MOVE_9_ENCODER_Y_TARGET_M (-10.38f)

/* RobotStateUpdateTask状态机各状态:START_*只在进入时下发一次ChassisSetTranslation
   (该函数每次调用都会令S曲线重新规划,跑向同一目标期间不能重复调用;x/y现为世界系绝对
   目标,START_MOVE_Y必须原样带上START_MOVE_X已下发的x目标,否则x会被打断拉回0);
   WAIT_MOVE_Y用固定容差轮询ChassisTranslationReached等待终点到位,WAIT_MOVE_X则用
   ROBOT_STATE_CORNER_BLEND_K放大的动态容差提前判定"到位",在x轴尚未停稳前就切到
   START_MOVE_Y以实现转弯圆滑过渡,而非在拐角处完全停顿 */
typedef enum
{
   ROBOT_STATE_START_MOVE_1,
   ROBOT_STATE_WAIT_MOVE_1,
   ROBOT_STATE_START_MOVE_2,
   ROBOT_STATE_WAIT_MOVE_2,
   ROBOT_STATE_START_MOVE_3,
   ROBOT_STATE_WAIT_MOVE_3,
   ROBOT_STATE_FLIIP,
   ROBOT_STATE_START_MOVE_4,
   ROBOT_STATE_WAIT_MOVE_4,
   ROBOT_STATE_START_MOVE_5,
   ROBOT_STATE_WAIT_MOVE_5,
   ROBOT_STATE_START_MOVE_6,
   ROBOT_STATE_WAIT_MOVE_6,
   ROBOT_STATE_START_MOVE_7,
   ROBOT_STATE_WAIT_MOVE_7,
   ROBOT_STATE_START_MOVE_8,
   ROBOT_STATE_WAIT_MOVE_8,
   ROBOT_STATE_START_MOVE_9,
   ROBOT_STATE_WAIT_MOVE_9,
   ROBOT_STATE_PICKUP,
   ROBOT_STATE_MANUAL,
   ROBOT_STATE_DONE,
} RobotState_TypeDef;

/* 顶层状态机周期,单位ms */
#define ROBOT_STATE_UPDATE_PERIOD_MS (5U)

/* ChassisTranslationReached的到位判定容差,单位m */
#define ROBOT_CHASSIS_POSITION_TOLERANCE_M (0.01f)

/* 机器人整体状态,汇总各子系统的运行时状态,供RobotInit()统一初始化、各RTOS任务
   共享访问 */
typedef struct
{
    RoboticArm_TypeDef roboticarm;
    Chassis_TypeDef chassis;
    Encoder_TypeDef encoder;
    ZigbeeHandle_TypeDef zigbee;
    VisionHandle_TypeDef vision;
    Yis512_TypeDef yis512;
    Flip_TypeDef flip;
    Pickup_TypeDef pickup;
    RobotState_TypeDef state;
    uint32_t time_stamp;
} Robot_TypeDef;

extern Robot_TypeDef Robot;

/* 初始化机械臂、底盘、编码器、YIS512、ZigBee与视觉模块,配置好FDCAN过滤器/FIFO与
   UART空闲线DMA接收,供RTOS启动前调用一次;ZigBee/视觉之后的持续接收完全由
   HAL_UARTEx_RxEventCallback(bsp_callback.c)中断驱动,不需要额外的周期任务 */
void RobotInit(void);

/* RTOS任务入口:持续下发底盘控制帧,触发电调反馈 */
void RobotChassisUpdateTask(void *argument);

/* RTOS任务入口:周期发送编码器位置请求,触发编码器应答 */
void RobotEncoderUpdateTask(void *argument);

/* RTOS任务入口:周期下发J60(升降)/GO(前后平移)电机控制帧,触发各自反馈 */
void RobotRoboticArmUpdateTask(void *argument);

/* RTOS任务入口:推进机器人整体位移状态机,按序下发各段ChassisSetTranslation目标,
   全部到位后转入手柄(ZigBee)遥控的手动模式 */
void RobotStateUpdateTask(void *argument);

void RobotServoUpdateTask(void *argument);

/* RTOS任务入口:独立推进flip状态机,仅在Robot.flip.active(由RobotStateUpdateTask的
   ROBOT_STATE_FLIIP阶段置位)为真时才调用RoboticArmFlipMotion */
void RobotFlipUpdateTask(void *argument);

/* RTOS任务入口:仅在Robot.pickup.active为真时，按action推进抓取状态机；
   到达VOID终态后置Robot.pickup.complete。 */
void RobotPickupUpdateTask(void *argument);

#endif
