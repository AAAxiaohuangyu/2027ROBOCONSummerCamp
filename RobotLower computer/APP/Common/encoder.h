#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "bsp_config.h"
#include "fdcan.h"

/*
 * BRT38M-COM4096D32-RT1 双万向轮编码器模块。
 *
 * 本模块只完成三件事：
 * 1. 接收上层 FDCAN 转交的编码器位置帧，在接收回调中直接把相邻两帧的计数差换算
 *    为本帧滚动距离并累加到对应轴的 distance_m；
 * 2. 在 EncoderUpdate() 这一自循环 RTOS 任务中周期发送位置查询请求帧；
 * 3. 取两轴滚动距离相对上一次积分的增量，按各自安装偏移扣除底盘旋转在该
 *    安装点引入的牵连线速度分量，再按安装角度投影为底盘车体系增量位移，
 *    最后按当前 yaw 把该增量旋转到世界系后累加到 x_m、y_m——即编码器安装点
 *    不在底盘几何中心、且底盘存在转动时的里程计融合，见 EncoderUpdate()。
 *
 * 坐标约定：ID1 编码器为 x_axis，ID2 编码器为 y_axis，理想安装下二者滚动方向
 * 分别与底盘 x 轴重合、垂直；实际安装角度由 ENCODER_X_INSTALL_ANGLE_DEG /
 * ENCODER_Y_INSTALL_ANGLE_DEG 描述，安装点相对底盘几何中心的偏移由
 * ENCODER_X/Y_OFFSET_X/Y_M 描述，见 EncoderUpdate()。本模块不写入编码器硬件
 * 零点，是否使用本模块由调用者是否调用 EncoderInit() 决定，不依赖额外的使能宏。
 */

/* ------------------------- 可调配置：集中在此处 ------------------------- */

/* EncoderUpdate() 任务内部循环周期，单位 ms。当前编码器查询使用 10 ms。 */
#define ENCODER_UPDATE_PERIOD_MS          (10U)

/*
 * 两只编码器挂载的 FDCAN 与节点 ID 集中在 Core/bsp_config.h 中定义（ENCODER_X_FDCAN_HANDLE
 * / ENCODER_X_NODE_ID / ENCODER_Y_FDCAN_HANDLE / ENCODER_Y_NODE_ID），调用者据此传给
 * EncoderInit()。更换 CAN 接口或修改编码器 ID 时改 bsp_config.h 中的宏值，不用修改本模块
 * 的协议解析代码。
 */

/* 两只万向轮当前均按直径 5 cm 计算；距离单位统一为 m。 */
#define ENCODER_WHEEL_DIAMETER_M          (0.05f)
#define ENCODER_WHEEL_RADIUS_M            (ENCODER_WHEEL_DIAMETER_M * 0.5f)

/*
 * 两只编码器滚动方向与底盘坐标系 x 轴的夹角，角度制，逆时针为正，取值范围不限于
 * 0~90 度。若编码器计数增加方向与实际滚动正方向相反，直接把该轴角度加/减 180 度
 * 即可翻转，无需额外的方向符号参数（cos(θ+180°)=-cosθ，sin(θ+180°)=-sinθ，与单独
 * 乘 -1 等价）。若实际安装角度存在偏差，只需修改这里的角度值，EncoderUpdate() 会
 * 按新角度重新把两轴滚动距离分解、叠加到底盘 x、y，无需改动换算代码。
 */
#define ENCODER_X_INSTALL_ANGLE_DEG       (45.0f)
#define ENCODER_Y_INSTALL_ANGLE_DEG       (-45.0f)

/*
 * 两只编码器滚动接触点相对底盘几何中心的安装偏移，单位 m，底盘坐标系下的
 * x/y 分量（与 ENCODER_X/Y_INSTALL_ANGLE_DEG 同一坐标系）。若码盘未装在
 * 几何中心，底盘转动会在该安装点叠加 ω×r 的牵连线速度，编码器会把这部分
 * 也当成滚动计入，需据此在 EncoderUpdate() 中先扣除、再换算为几何中心的
 * 位移。恰好装在几何中心时四个偏移量保持为 0 即可。实测标定。
 */
#define ENCODER_X_OFFSET_X_M              (0.162f)
#define ENCODER_X_OFFSET_Y_M              (0.185f)
#define ENCODER_Y_OFFSET_X_M              (0.162f)
#define ENCODER_Y_OFFSET_Y_M              (0.1115f)

/* BRT38M 固定协议与型号参数，不应作为现场调参项。 */
#define ENCODER_COUNTS_PER_REVOLUTION     (4096U)
#define ENCODER_CMD_READ_POSITION          (0x01U)
#define ENCODER_POSITION_REQUEST_LENGTH   (4U)
#define ENCODER_POSITION_REPLY_LENGTH     (7U)

/* 编码器硬件自带的多圈计数范围有限，原始计数返回值为 0~X（X=单圈分辨率*圈数-1），
   到达上限后会环绕回0（反向转动则从0环绕回X）。实际使用中累计转动圈数可能超出
   这个硬件范围，故在EncoderAxisParseFeedback()计算相邻两帧计数差时按该范围做
   环绕修正，等效于用软件把有限的硬件多圈范围扩展为不受圈数上限约束的多圈计数；
   该修正假设两次查询之间（ENCODER_UPDATE_PERIOD_MS）实际转动的圈数远小于
   ENCODER_HARDWARE_TURN_COUNT 的一半，否则无法区分“正转环绕”与“反转环绕”。 */
#define ENCODER_HARDWARE_TURN_COUNT       (32U)
#define ENCODER_RAW_COUNT_RANGE           (ENCODER_COUNTS_PER_REVOLUTION * ENCODER_HARDWARE_TURN_COUNT)

/*
 * 单只编码器的运行状态。FDCAN_Handle/node_id 由 EncoderInit() 赋值；
 * raw_position_count、origin_position_count、distance_m 均由 EncoderParseFeedback()
 * 在 FDCAN 接收回调中更新——浮点距离换算与累加就在回调中一次性完成，
 * EncoderUpdate() 任务循环不再重复计算，只读取 distance_m 做坐标分解。
 */
typedef struct
{
    FDCAN_HandleTypeDef *FDCAN_Handle; /* 该编码器挂载的 FDCAN 总线实例 */
    uint8_t node_id;                   /* 编码器 CAN 节点 id */

    volatile uint32_t raw_position_count;
    uint32_t origin_position_count; /* 上一帧位置反馈的计数值，作为下一帧计算增量
                                        位移的基准，每帧位置反馈到达后都会刷新为
                                        该帧的计数值，见EncoderAxisParseFeedback() */
    volatile float distance_m; /* 相对软件原点的累计滚动距离，单位 m；在FDCAN接收
                                   回调中按相邻两帧计数差增量累加，EncoderUpdate()
                                   任务与回调分属不同上下文，故需volatile */
    volatile uint8_t position_initialized; /* 0表示尚未收到过位置反馈：编码器上电时的
                                                原始计数是任意值，并非0，若沿用
                                                origin_position_count的初值0计算首帧
                                                增量会把该任意值误计入distance_m；置1
                                                后表示已用首帧计数建立基准，之后每帧
                                                才按正常增量累加，见
                                                EncoderAxisParseFeedback() */
} EncoderAxis_TypeDef;

/* 模块整体状态：两只编码器，及扣除安装偏心、按 yaw 转换到世界系后累加得到
   的底盘世界系位置，与 chassis->pose.x_m/y_m 同一坐标系，可直接互相赋值。 */
typedef struct
{
    EncoderAxis_TypeDef x_axis; /* ID1 编码器 */
    EncoderAxis_TypeDef y_axis; /* ID2 编码器 */

    float x_m; /* 底盘世界系累计位置 x，见 EncoderUpdate() 中的积分过程 */
    float y_m; /* 底盘世界系累计位置 y，见 EncoderUpdate() 中的积分过程 */

    /* 上一次积分时的两轴累计距离快照，用于取本周期增量，见
       EncoderIntegrateOdometry()。 */
    float prev_x_axis_distance_m;
    float prev_y_axis_distance_m;
    float prev_yaw_rad; /* 上一次积分时的底盘 yaw（来自 chassis->pose.yaw_rad），
                            用于取 yaw 增量、把车体系位移增量旋转到世界系 */
    uint8_t pose_initialized; /* 0 表示尚未建立积分基准：上电首次调用只记录
                                  上述三个快照、不累加位移，避免把上电瞬间的
                                  任意状态误计入位移，见 EncoderIntegrateOdometry() */
} Encoder_TypeDef;

/*
 * 初始化双编码器模块：x_axis/y_axis 各自的 FDCAN 句柄、节点 id 均由调用者传入，
 * 在此一并赋值到结构体，同时为各自挂载的总线配置对应节点 id 的接收过滤器并启动 FDCAN
 * （两轴若共用同一条总线，会对该总线重复调用一次，不影响正确性）。
 * 本函数不发送位置查询，首次有效反馈到来前两轴 distance_m 均为 0。
 */
void EncoderInit(Encoder_TypeDef *encoder,
                  FDCAN_HandleTypeDef *x_fdcan_handle, uint8_t x_node_id,
                  FDCAN_HandleTypeDef *y_fdcan_handle, uint8_t y_node_id);

/*
 * 解析一帧 FDCAN 反馈数据：先由 fdcan_handle+std_id 判断该帧属于 x 轴还是 y 轴。
 * 命中位置反馈则用本帧计数与上一帧计数（origin_position_count）的差值换算为
 * 本帧滚动距离并累加进 distance_m，浮点换算与累加均在本函数中完成，不再交给
 * EncoderUpdate() 任务处理。不命中则不作任何处理。命中返回 1，不命中返回 0；
 * 调用者在 HAL_FDCAN_RxFifo1Callback 中取得 std_id 和数据后调用。
 */
uint8_t EncoderParseFeedback(Encoder_TypeDef *encoder, FDCAN_HandleTypeDef *fdcan_handle,
                              uint32_t std_id, const uint8_t data[8]);

/*
 * RTOS 任务入口：内部为 while(1) 循环，每轮向 x/y 两只编码器各发送一次位置
 * 读取请求（浮点距离换算与累加已在 EncoderParseFeedback() 回调中完成），再取
 * 两轴累计滚动距离相对上一次积分的增量，按 ENCODER_X/Y_OFFSET_X/Y_M 扣除
 * 底盘转动在安装点引入的牵连线速度分量，按 ENCODER_X/Y_INSTALL_ANGLE_DEG
 * 投影为底盘车体系位移增量，再用底盘 yaw（读自 Robot.chassis.pose.yaw_rad）
 * 旋转到世界系后累加，写入 encoder->x_m / y_m，与 chassis->pose.x_m/y_m
 * 同一坐标系。每轮结束调用 osDelay(ENCODER_UPDATE_PERIOD_MS) 让出 CPU，
 * 函数本身不会返回，需通过 osThreadNew 启动、且不能在 FDCAN 中断回调中调用。
 */
void EncoderUpdate(Encoder_TypeDef *encoder);

#endif /* ENCODER_H */
