#ifndef __TEST_H_
#define __TEST_H_

/*
 * 底盘(M3508)+ Encoder CAN接收测试:只拉起需要测的子系统,不进RobotInit/整机状态机。
 * 验收不看本模块的代码,直接在调试器Watch窗口看:
 *   Robot.chassis.drive.motor_group.motor[0..3].feedback.update_cnt 持续增长
 *     -> 过滤器0/RXFIFO0 收到了3508反馈帧;
 *   Robot.encoder.x_axis/y_axis.feedback_ready 变1且raw_position_count随实际转动变化
 *     -> 过滤器1/RXFIFO1 收到了encoder应答帧。
 */

/* 初始化底盘与编码器,配置好FDCAN过滤器/FIFO,供RTOS启动前调用一次 */
void TestChassisEncoderRxInit(void);

/* RTOS任务入口:持续下发底盘控制帧(速度维持ChassisInit给的默认0),触发电调反馈 */
void TestChassisUpdateTask(void *argument);

/* RTOS任务入口:周期发送编码器位置请求,触发编码器应答 */
void TestEncoderUpdateTask(void *argument);

/* RTOS任务入口:下发一次固定测试速度(ChassisSetVelocity),之后空转;
   由TestChassisUpdateTask持续下发对应控制帧,用于验证电机能否按目标转动 */
void TestChassisSetVelocityTask(void *argument);

#endif
