#include "encoder.h"
#include <math.h>
#include "fdcan_common.h"
#include "cmsis_os2.h"

/* 初始化单只编码器：只赋值外设句柄/节点id/方向符号，清空原始计数与软件原点；
   不在这里配置FDCAN接收过滤器——两轴可能共用同一条总线，过滤器范围需由
   EncoderInit统一计算后一次性下发，否则后配置的一路会覆盖先配置的一路。 */
static void EncoderAxisInit(EncoderAxis_TypeDef *axis, FDCAN_HandleTypeDef *fdcan_handle,
                             uint8_t node_id, int8_t direction_sign)
{
    axis->FDCAN_Handle = fdcan_handle;
    axis->node_id = node_id;
    axis->direction_sign = direction_sign;
    axis->raw_position_count = 0U;
    axis->feedback_ready = 0U;
    axis->origin_position_count = 0U;
    axis->distance_m = 0.0f;
}

/* 将一条轴的最新位置计数换算为相对软件原点的行进距离。 */
static float EncoderAxisConvertDistance(const EncoderAxis_TypeDef *axis)
{
    int32_t relative_count;
    float wheel_circumference_m;

    if (axis->feedback_ready == 0U)
    {
        return 0.0f;
    }

    relative_count = (int32_t)axis->raw_position_count -
                      (int32_t)axis->origin_position_count;
    relative_count *= (int32_t)axis->direction_sign;

    wheel_circumference_m = 2.0f * BSP_PI * ENCODER_WHEEL_RADIUS_M;
    return ((float)relative_count * wheel_circumference_m) /
           (float)ENCODER_COUNTS_PER_REVOLUTION;
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

    axis->raw_position_count = position_count;
    if (axis->feedback_ready == 0U)
    {
        /* 第一帧只建立软件原点，不把编码器上电前的绝对位置算入x/y。 */
        axis->origin_position_count = position_count;
        axis->feedback_ready = 1U;
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

void EncoderInit(Encoder_TypeDef *encoder, EncoderTask_TypeDef *task,
                  FDCAN_HandleTypeDef *x_fdcan_handle, uint8_t x_node_id, int8_t x_direction_sign,
                  FDCAN_HandleTypeDef *y_fdcan_handle, uint8_t y_node_id, int8_t y_direction_sign,
                  float *chassis_x_m, float *chassis_y_m)
{
    EncoderAxisInit(&encoder->x_axis, x_fdcan_handle, x_node_id, x_direction_sign);
    EncoderAxisInit(&encoder->y_axis, y_fdcan_handle, y_node_id, y_direction_sign);
    encoder->x_m = 0.0f;
    encoder->y_m = 0.0f;

    task->encoder = encoder;
    task->chassis_x_m = chassis_x_m;
    task->chassis_y_m = chassis_y_m;

    if (x_fdcan_handle == y_fdcan_handle)
    {
        /* 同一条总线只有一个可用的列表过滤器(FilterIndex固定为0),必须合并成一次调用
           覆盖两只编码器的节点id，否则后一次FDCANStandardInit会覆盖前一次的过滤器。 */
        uint8_t range_min = (x_node_id < y_node_id) ? x_node_id : y_node_id;
        uint8_t range_max = (x_node_id > y_node_id) ? x_node_id : y_node_id;

        FDCANStandardInit(x_fdcan_handle, range_min, range_max);
    }
    else
    {
        FDCANStandardInit(x_fdcan_handle, x_node_id, x_node_id);
        FDCANStandardInit(y_fdcan_handle, y_node_id, y_node_id);
    }
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

void EncoderUpdate(EncoderTask_TypeDef *task)
{
    while (1)
    {
        /* FDCAN请求和浮点距离换算都在本任务中完成。 */
        EncoderAxisRequestPosition(&task->encoder->x_axis);
        EncoderAxisRequestPosition(&task->encoder->y_axis);

        task->encoder->x_axis.distance_m = EncoderAxisConvertDistance(&task->encoder->x_axis);
        task->encoder->y_axis.distance_m = EncoderAxisConvertDistance(&task->encoder->y_axis);

        EncoderDecomposeAxes(task->encoder);

        /* 通过指针把最新底盘位置同步写入调用者指定的地址（如 chassis->pose.x_m/y_m）。 */
        if (task->chassis_x_m != NULL)
        {
            *task->chassis_x_m = task->encoder->x_m;
        }
        if (task->chassis_y_m != NULL)
        {
            *task->chassis_y_m = task->encoder->y_m;
        }

        osDelay(ENCODER_UPDATE_PERIOD_MS);
    }
}
