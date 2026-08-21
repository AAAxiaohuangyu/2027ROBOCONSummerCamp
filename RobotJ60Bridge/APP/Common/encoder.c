#include "encoder.h"
#include <math.h>
#include "fdcan_common.h"
#include "cmsis_os2.h"
#include "Core.h"

/* 初始化单只编码器：只赋值外设句柄/节点id，清空原始计数与软件原点；
   不在这里配置FDCAN接收过滤器——两轴可能共用同一条总线，过滤器范围需由
   EncoderInit统一计算后一次性下发，否则后配置的一路会覆盖先配置的一路。 */
static void EncoderAxisInit(EncoderAxis_TypeDef *axis, FDCAN_HandleTypeDef *fdcan_handle,
                            uint8_t node_id)
{
    axis->FDCAN_Handle = fdcan_handle;
    axis->node_id = node_id;
    axis->raw_position_count = 0U;
    axis->origin_position_count = 0U;
    axis->distance_m = 0.0f;
    axis->position_initialized = 0U;
}

/* 向一只编码器发送 BRT 协议 FUNC=0x01 的“读取多圈位置”请求。 */
static void EncoderAxisRequestPosition(const EncoderAxis_TypeDef *axis)
{
    uint8_t data[ENCODER_POSITION_REQUEST_LENGTH];

    data[0] = ENCODER_POSITION_REQUEST_LENGTH;
    data[1] = axis->node_id;
    data[2] = ENCODER_CMD_READ_POSITION;
    data[3] = 0U;

    /* 发送失败仅表示本帧未进入FIFO；下一周期会再次请求，不在这里做额外状态机。 */
    FDCANSendStandard(axis->FDCAN_Handle, axis->node_id, data, ENCODER_POSITION_REQUEST_LENGTH);
}

/* 解析一帧反馈数据到指定轴：先判断该帧是否来自该轴挂载的总线与节点id，
   再判断是否为BRT位置读取回复；命中则更新原始计数并返回1，否则返回0。 */
static uint8_t EncoderAxisParseFeedback(EncoderAxis_TypeDef *axis, FDCAN_HandleTypeDef *fdcan_handle,
                                        uint32_t std_id, const uint8_t data[8])
{
    uint32_t position_count;

    if ((fdcan_handle != axis->FDCAN_Handle) || (std_id != axis->node_id))
    {
        return 0U;
    }

    /* BRT位置回复固定为：07 ID 01 POS0 POS1 POS2 POS3，位置计数为小端序。 */
    if ((data[0] != ENCODER_POSITION_REPLY_LENGTH) ||
        (data[1] != axis->node_id) ||
        (data[2] != ENCODER_CMD_READ_POSITION))
    {
        return 0U;
    }

    position_count = ((uint32_t)data[3]) |
                     ((uint32_t)data[4] << 8) |
                     ((uint32_t)data[5] << 16) |
                     ((uint32_t)data[6] << 24);

    if (axis->position_initialized == 0U)
    {
        /* 首帧位置反馈：编码器上电时的原始计数是任意值，只把它记为基准，
           不计入distance_m，确保位移从0开始，见encoder.h中
           position_initialized的注释。 */
        axis->origin_position_count = position_count;
        axis->raw_position_count = position_count;
        axis->position_initialized = 1U;
        return 1U;
    }

    axis->origin_position_count = axis->raw_position_count;
    axis->raw_position_count = position_count;

    /* 用相邻两帧的计数差换算为本帧滚动的角度、再按轮子周长换算为本帧滚动
       距离，直接累加进distance_m——浮点换算与累加都在本回调（FDCAN接收
       中断上下文）中一次性完成，EncoderUpdate()任务不再重复计算。 */
    {
        int32_t delta_count;
        float angle_deg;
        float wheel_circumference_m;

        delta_count = (int32_t)axis->raw_position_count - (int32_t)axis->origin_position_count;

        /* 计数差超过硬件多圈范围一半，说明本帧相对上一帧发生了环绕（见
           encoder.h中ENCODER_RAW_COUNT_RANGE的注释），据此修正为环绕前后的
           真实增量，使distance_m的累加不受硬件多圈范围上限约束。 */
        if (delta_count > (int32_t)(ENCODER_RAW_COUNT_RANGE / 2U))
        {
            delta_count -= (int32_t)ENCODER_RAW_COUNT_RANGE;
        }
        else if (delta_count < -(int32_t)(ENCODER_RAW_COUNT_RANGE / 2U))
        {
            delta_count += (int32_t)ENCODER_RAW_COUNT_RANGE;
        }

        angle_deg = ((float)delta_count * 360.0f) / (float)ENCODER_COUNTS_PER_REVOLUTION;
        wheel_circumference_m = 2.0f * BSP_PI * ENCODER_WHEEL_RADIUS_M;

        axis->distance_m += (angle_deg / 360.0f) * wheel_circumference_m;
    }

    return 1U;
}

/* 把两轴各自的滚动距离，按各自安装角度分解到底盘 x、y 方向后叠加。 */
static void EncoderDecomposeAxes(Encoder_TypeDef *encoder)
{
    float x_angle_rad = ENCODER_X_INSTALL_ANGLE_DEG * BSP_PI / 180.0f;
    float y_angle_rad = ENCODER_Y_INSTALL_ANGLE_DEG * BSP_PI / 180.0f;

    encoder->x_m = (encoder->x_axis.distance_m * cosf(x_angle_rad)) +
                   (encoder->y_axis.distance_m * cosf(y_angle_rad));
    encoder->y_m = (encoder->x_axis.distance_m * sinf(x_angle_rad)) +
                   (encoder->y_axis.distance_m * sinf(y_angle_rad));
}

void EncoderInit(Encoder_TypeDef *encoder,
                 FDCAN_HandleTypeDef *x_fdcan_handle, uint8_t x_node_id,
                 FDCAN_HandleTypeDef *y_fdcan_handle, uint8_t y_node_id)
{
    EncoderAxisInit(&encoder->x_axis, x_fdcan_handle, x_node_id);
    EncoderAxisInit(&encoder->y_axis, y_fdcan_handle, y_node_id);
    encoder->x_m = 0.0f;
    encoder->y_m = 0.0f;

    if (x_fdcan_handle == y_fdcan_handle)
    {
        /* FilterIndex固定为0,目标RXFIFO1;同一条总线上两只编码器共用这一个过滤器，
           必须合并成一次调用覆盖两只编码器的节点id，否则后一次调用会覆盖前一次的过滤器。 */
        uint8_t range_min = (x_node_id < y_node_id) ? x_node_id : y_node_id;
        uint8_t range_max = (x_node_id > y_node_id) ? x_node_id : y_node_id;

        FDCANFilterInit(x_fdcan_handle, 0, range_min, range_max, FDCAN_FILTER_TO_RXFIFO1);
    }
    else
    {
        FDCANFilterInit(x_fdcan_handle, 0, x_node_id, x_node_id, FDCAN_FILTER_TO_RXFIFO1);
        FDCANFilterInit(y_fdcan_handle, 0, y_node_id, y_node_id, FDCAN_FILTER_TO_RXFIFO1);
    }

    HAL_Delay(500);
}

uint8_t EncoderParseFeedback(Encoder_TypeDef *encoder, FDCAN_HandleTypeDef *fdcan_handle,
                             uint32_t std_id, const uint8_t data[8])
{
    if (EncoderAxisParseFeedback(&encoder->x_axis, fdcan_handle, std_id, data))
    {
        return 1U;
    }

    return EncoderAxisParseFeedback(&encoder->y_axis, fdcan_handle, std_id, data);
}

void EncoderUpdate(Encoder_TypeDef *encoder)
{
    while (1)
    {
        /* 浮点距离换算与累加已在EncoderAxisParseFeedback()回调（FDCAN接收中断上下文）
           中完成；本任务只负责周期发送位置请求帧、以及把两轴累计距离按安装角度
           分解叠加为底盘坐标。 */
        EncoderAxisRequestPosition(&encoder->x_axis);
        EncoderAxisRequestPosition(&encoder->y_axis);

        osDelay(ENCODER_UPDATE_PERIOD_MS);

        EncoderDecomposeAxes(encoder);

        /*暂时将码盘数据直接赋值给底盘坐标*/
        ChassisSetPosition((Chassis_TypeDef *)&Robot.chassis, Robot.encoder.x_m, Robot.encoder.y_m);
    }
}
