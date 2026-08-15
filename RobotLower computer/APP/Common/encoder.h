#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "bsp_config.h"
#include "fdcan.h"

/*
 * BRT38M-COM4096D32-RT1 双万向轮编码器模块。
 *
 * 本模块只完成三件事：
 * 1. 接收上层 FDCAN 转交的编码器位置帧，更新对应轴的原始计数；
 * 2. 在 EncoderUpdate() 这一自循环 RTOS 任务中将两个相对计数换算为各自轮子的滚动距离；
 * 3. 按各自的安装角度把两轴滚动距离分解到底盘坐标系 x、y 方向后叠加。
 *
 * 坐标约定：ID1 编码器为 x_axis，ID2 编码器为 y_axis，理想安装下二者滚动方向
 * 分别与底盘 x 轴重合、垂直；实际安装角度由 ENCODER_X_INSTALL_ANGLE_DEG /
 * ENCODER_Y_INSTALL_ANGLE_DEG 描述，见 EncoderUpdate()。首次收到每只编码器的
 * 合法位置反馈时，该位置自动成为本次上电后的软件原点，因此两轴滚动距离均从
 * 0 m 开始。本模块不写入编码器硬件零点，是否使用本模块由调用者是否调用
 * EncoderInit() 决定，不依赖额外的使能宏。
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

/* 两只万向轮当前均按直径 10 cm 计算；距离单位统一为 m。 */
#define ENCODER_WHEEL_DIAMETER_M          (0.10f)
#define ENCODER_WHEEL_RADIUS_M            (ENCODER_WHEEL_DIAMETER_M * 0.5f)

/*
 * 当实际滚动的正方向与编码器位置计数增加方向一致时设为 +1；相反时设为 -1。
 * 这只影响最终 x/y 的正负号，不修改编码器本身的方向配置。
 */
#define ENCODER_X_DIRECTION_SIGN          (1)
#define ENCODER_Y_DIRECTION_SIGN          (1)

/*
 * 两只编码器滚动方向与底盘坐标系 x 轴的夹角，角度制，逆时针为正。
 * 理想安装下 x_axis（ID1）滚动方向与底盘 x 轴重合，取 0 度；y_axis（ID2）
 * 滚动方向与底盘 x 轴垂直，取 90 度。若实际安装存在偏差，只需修改这里的
 * 角度值，EncoderUpdate() 会按新角度重新把两轴滚动距离分解、叠加到底盘
 * x、y，无需改动换算代码。
 */
#define ENCODER_X_INSTALL_ANGLE_DEG       (0.0f)
#define ENCODER_Y_INSTALL_ANGLE_DEG       (90.0f)

/* BRT38M 固定协议与型号参数，不应作为现场调参项。 */
#define ENCODER_COUNTS_PER_REVOLUTION     (4096U)
#define ENCODER_CMD_READ_POSITION          (0x01U)
#define ENCODER_POSITION_REQUEST_LENGTH   (4U)
#define ENCODER_POSITION_REPLY_LENGTH     (7U)

/*
 * 单只编码器的运行状态。FDCAN_Handle/node_id/direction_sign 由 EncoderInit() 赋值；
 * raw_position_count、feedback_ready 由 EncoderParseFeedback() 在 FDCAN 接收回调中更新；
 * origin_position_count、distance_m 由 EncoderUpdate() 任务循环更新。
 */
typedef struct
{
    FDCAN_HandleTypeDef *FDCAN_Handle; /* 该编码器挂载的 FDCAN 总线实例 */
    uint8_t node_id;                   /* 编码器 CAN 节点 id */
    int8_t direction_sign;             /* 计数方向与实际滚动正方向的符号关系，+1/-1 */

    volatile uint32_t raw_position_count;
    volatile uint8_t feedback_ready;
    uint32_t origin_position_count;
    float distance_m; /* 相对软件原点的累计滚动距离，单位 m */
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
 * EncoderUpdate 作为 RTOS 任务入口运行时的入参：把该任务生命周期内固定不变的
 * encoder 实例指针、以及底盘当前位置的写入地址打包成一个结构体，配合
 * osThreadNew 只接受单个 void* argument 的限制使用（用法与 ChassisUpdate 接受
 * 单个 Chassis_TypeDef* 一致，调用处对 EncoderUpdate 做函数指针类型转换后传入
 * &task 即可）。各字段由 EncoderInit() 一并赋值，调用者需保证 task 及其指向的
 * encoder、chassis_x_m/chassis_y_m 生命周期覆盖整个任务运行期间（通常为静态或
 * 全局变量）。
 */
typedef struct
{
    Encoder_TypeDef *encoder; /* 待更新的编码器实例 */
    float *chassis_x_m;       /* 底盘当前位置 x 的写入地址，可为 NULL 表示不写 */
    float *chassis_y_m;       /* 底盘当前位置 y 的写入地址，可为 NULL 表示不写 */
} EncoderTask_TypeDef;

/*
 * 初始化双编码器模块：x_axis/y_axis 各自的 FDCAN 句柄、节点 id、方向符号均由调用者传入，
 * 在此一并赋值到结构体，同时为各自挂载的总线配置对应节点 id 的接收过滤器并启动 FDCAN
 * （两轴若共用同一条总线，会对该总线重复调用一次，不影响正确性）。
 * 同时初始化 EncoderUpdate 任务参数 task：把 encoder 本身与底盘当前位置的写入地址
 * chassis_x_m/chassis_y_m 一并写入 task，供 osThreadNew 启动 EncoderUpdate 时把
 * &task 作为单个 argument 传入；chassis_x_m/chassis_y_m 允许为 NULL，表示该任务
 * 不需要同步对应方向的底盘位置。
 * 本函数不发送位置查询，首次有效反馈到来前两轴 distance_m 均为 0。
 */
void EncoderInit(Encoder_TypeDef *encoder, EncoderTask_TypeDef *task,
                  FDCAN_HandleTypeDef *x_fdcan_handle, uint8_t x_node_id, int8_t x_direction_sign,
                  FDCAN_HandleTypeDef *y_fdcan_handle, uint8_t y_node_id, int8_t y_direction_sign,
                  float *chassis_x_m, float *chassis_y_m);

/*
 * 解析一帧 FDCAN 反馈数据：先由 fdcan_handle+std_id 判断该帧属于 x 轴还是 y 轴，命中则
 * 更新对应轴的原始计数并返回 1，不命中则不作任何处理并返回 0；调用者在
 * HAL_FDCAN_RxFifo0Callback 中取得 std_id 和数据后调用。函数内部只保存整数计数，不做
 * 浮点换算、串口输出或阻塞操作。
 */
uint8_t EncoderParseFeedback(Encoder_TypeDef *encoder, FDCAN_HandleTypeDef *fdcan_handle,
                              uint32_t std_id, const uint8_t data[8]);

/*
 * RTOS 任务入口：内部为 while(1) 循环，每轮向 x/y 两只编码器各发送一次位置
 * 读取请求，把最近一次接收的原始计数换算为两轴的滚动距离，再按
 * ENCODER_X_INSTALL_ANGLE_DEG / ENCODER_Y_INSTALL_ANGLE_DEG 把两轴滚动距离
 * 分解到底盘 x、y 方向并叠加，写入 task->encoder->x_m / y_m；task->chassis_x_m/
 * chassis_y_m 非 NULL 时会把同一结果通过指针同步写入调用者指定的地址（例如
 * &chassis->pose.x_m / &chassis->pose.y_m），用于刷新底盘当前位置。每轮结束
 * 调用 osDelay(ENCODER_UPDATE_PERIOD_MS) 让出 CPU，函数本身不会返回，需通过
 * osThreadNew 启动、且不能在 FDCAN 中断回调中调用。
 */
void EncoderUpdate(EncoderTask_TypeDef *task);

#endif /* ENCODER_H */
