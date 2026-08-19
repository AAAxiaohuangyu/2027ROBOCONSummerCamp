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
 * 3. 按各自的安装角度把两轴累计滚动距离分解到底盘坐标系 x、y 方向后叠加。
 *
 * 坐标约定：ID1 编码器为 x_axis，ID2 编码器为 y_axis，理想安装下二者滚动方向
 * 分别与底盘 x 轴重合、垂直；实际安装角度由 ENCODER_X_INSTALL_ANGLE_DEG /
 * ENCODER_Y_INSTALL_ANGLE_DEG 描述，见 EncoderUpdate()。EncoderInit() 上电时会向
 * 每只编码器发送一次“设置多圈中点”请求（FUNC=0x0C，把编码器内部多圈计数器重置到
 * 量程中点、留出上下余量，不改变零点定义），并阻塞等待应答；收到应答后触发的
 * 下一帧位置反馈自动成为本次上电后的软件原点，因此两轴滚动距离均从 0 m 开始。
 * 该流程假设“设置中点”请求一定会在 EncoderInit() 的等待超时前得到应答——若某轴
 * 超时未收到应答，该轴将永远不会建立软件原点。本模块不写入编码器硬件零点，是否
 * 使用本模块由调用者是否调用 EncoderInit() 决定，不依赖额外的使能宏。
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
#define ENCODER_X_INSTALL_ANGLE_DEG       (225.0f)
#define ENCODER_Y_INSTALL_ANGLE_DEG       (-225.0f)

/* BRT38M 固定协议与型号参数，不应作为现场调参项。 */
#define ENCODER_COUNTS_PER_REVOLUTION     (4096U)
#define ENCODER_CMD_READ_POSITION          (0x01U)
#define ENCODER_POSITION_REQUEST_LENGTH   (4U)
#define ENCODER_POSITION_REPLY_LENGTH     (7U)

/* BRT FUNC=0x0C：将多圈计数器设置为中点位置，使计数器上下都留有余量，避免
   编码器在零位附近来回滚动时发生环绕。除 EncoderInit() 上电时发送一次外，底盘
   运行中因编码器硬件圈数限制也可能再次调用，此时 origin_position_count 会在
   收到"设置中点"应答之后的下一帧位置反馈到达时直接置为该帧的计数值（而不是
   请求发出后的下一帧——应答到达前的位置反馈仍可能是重置前的陈旧计数），使这
   一帧不计入位移增量，避免中点重置造成的计数跳变被累加进 distance_m，见
   EncoderAxisRequestSetMidpoint()/EncoderAxisParseFeedback()。 */
#define ENCODER_CMD_SET_MIDPOINT          (0x0CU)
#define ENCODER_MIDPOINT_REQUEST_LENGTH   (4U)
#define ENCODER_MIDPOINT_REQUEST_PARAM    (0x01U)
#define ENCODER_MIDPOINT_REPLY_LENGTH     (4U)
#define ENCODER_MIDPOINT_REPLY_STATUS_OK  (0x00U)

/* BRT FUNC=0x04：设置模式，PARAM=0x00。EncoderInit() 上电时向每只编码器发送一次并
   阻塞等待应答；成功应答为请求帧的原样回传：04 ID 04 00，见
   EncoderAxisRequestSetMode()/EncoderAxisParseFeedback()。 */
#define ENCODER_CMD_SET_MODE              (0x04U)
#define ENCODER_MODE_REQUEST_LENGTH       (4U)
#define ENCODER_MODE_REQUEST_PARAM        (0x00U)
#define ENCODER_MODE_REPLY_LENGTH         (4U)
#define ENCODER_MODE_REPLY_STATUS_OK      (0x00U)
/* EncoderInit() 等待两轴"设置中点"应答的上限，超时未收到也不阻塞后续流程。 */
#define ENCODER_MIDPOINT_ACK_TIMEOUT_MS   (500U)

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

    volatile uint8_t mode_ack; /* 收到"设置模式"成功应答后置1，供EncoderInit()阻塞等待 */
    volatile uint8_t midpoint_ack; /* 收到"设置中点"成功应答后置1，供EncoderInit()阻塞等待 */
    volatile uint8_t midpoint_pending; /* 收到"设置中点"成功应答后置1(不是请求发出时)，
                                           确保只有编码器已确认完成内部重置才让下一帧
                                           位置反馈重新建立增量基准，避免把应答到达前
                                           仍可能陈旧的位置反馈误当作新基准、把重置造成
                                           的计数跳变累加进distance_m；该帧消费后清0，
                                           见EncoderAxisParseFeedback()。若"设置中点"
                                           应答一直不到达，该轴将永远不会清除这个标志 */
    volatile uint8_t request_blocked; /* 请求帧发送标志位：0表示允许发送位置查询请求帧；
                                          EncoderAxisRequestSetMidpoint()发出"设置中点"请求后
                                          立即置1，禁止再发送位置查询请求帧，直到收到编码器
                                          返回的设置成功应答（见EncoderAxisParseFeedback()）
                                          才清0，避免中点重置期间的位置查询与重置应答互相干扰 */
} EncoderAxis_TypeDef;

/* 模块整体状态：两只编码器及按安装角度分解、叠加后的底盘坐标系位置。 */
typedef struct
{
    EncoderAxis_TypeDef x_axis; /* ID1 编码器 */
    EncoderAxis_TypeDef y_axis; /* ID2 编码器 */

    float x_m; /* 底盘坐标系下的 x，由两轴滚动距离按安装角度分解、叠加得到 */
    float y_m; /* 底盘坐标系下的 y，由两轴滚动距离按安装角度分解、叠加得到 */
} Encoder_TypeDef;

/*
 * 初始化双编码器模块：x_axis/y_axis 各自的 FDCAN 句柄、节点 id 均由调用者传入，
 * 在此一并赋值到结构体，同时为各自挂载的总线配置对应节点 id 的接收过滤器并启动 FDCAN
 * （两轴若共用同一条总线，会对该总线重复调用一次，不影响正确性）。
 * 过滤器配置完成后，会各向 x_axis/y_axis 先发送一次 FUNC=0x04“设置模式”请求并阻塞
 * 等待原样回传的成功应答（置位 mode_ack），再发送一次 FUNC=0x0C“设置多圈中点”请求，
 * 并在 ENCODER_MIDPOINT_ACK_TIMEOUT_MS 内忙等对应应答（应答经 FDCAN 接收中断异步
 * 到达，由 EncoderParseFeedback 解析并置位 midpoint_ack/midpoint_pending）。增量位移的
 * 计算基准由该应答触发的下一帧位置反馈建立，因此本函数假设应答一定会在超时前到达——
 * 若某轴超时仍未收到应答，该轴的首帧位置反馈会把重置前后的计数跳变误计入
 * distance_m。本函数不发送位置查询，首次有效反馈到来前两轴 distance_m 均为 0。
 */
void EncoderInit(Encoder_TypeDef *encoder,
                  FDCAN_HandleTypeDef *x_fdcan_handle, uint8_t x_node_id,
                  FDCAN_HandleTypeDef *y_fdcan_handle, uint8_t y_node_id);

/*
 * 解析一帧 FDCAN 反馈数据：先由 fdcan_handle+std_id 判断该帧属于 x 轴还是 y 轴。
 * 命中"设置中点"应答则置位 midpoint_ack/midpoint_pending；命中位置反馈则用本帧
 * 计数与上一帧计数（origin_position_count）的差值换算为本帧滚动距离并累加进
 * distance_m，浮点换算与累加均在本函数中完成，不再交给 EncoderUpdate() 任务处理。
 * 不命中以上两种回复则不作任何处理。命中返回 1，不命中返回 0；调用者在
 * HAL_FDCAN_RxFifo1Callback 中取得 std_id 和数据后调用。
 */
uint8_t EncoderParseFeedback(Encoder_TypeDef *encoder, FDCAN_HandleTypeDef *fdcan_handle,
                              uint32_t std_id, const uint8_t data[8]);

/*
 * RTOS 任务入口：内部为 while(1) 循环，每轮向 x/y 两只编码器各发送一次位置
 * 读取请求（浮点距离换算与累加已在 EncoderParseFeedback() 回调中完成），再按
 * ENCODER_X_INSTALL_ANGLE_DEG / ENCODER_Y_INSTALL_ANGLE_DEG 把两轴累计滚动距离
 * 分解到底盘 x、y 方向并叠加，写入 encoder->x_m / y_m。每轮结束调用
 * osDelay(ENCODER_UPDATE_PERIOD_MS) 让出 CPU，函数本身不会返回，需通过
 * osThreadNew 启动、且不能在 FDCAN 中断回调中调用。
 */
void EncoderUpdate(Encoder_TypeDef *encoder);

/*
 * 对x_axis/y_axis各发送一次FUNC=0x0C"设置多圈中点"请求,不阻塞等待应答——是否已
 * 完成重置以EncoderParseFeedback()异步置位的midpoint_ack/midpoint_pending为准。
 * 供EncoderInit()之外、底盘运行中需要重新居中的场景调用。
 */
void EncoderRequestSetMidpoint(Encoder_TypeDef *encoder);

#endif /* ENCODER_H */
