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
    axis->mode_ack = 0U;
    axis->midpoint_ack = 0U;
    axis->midpoint_pending = 0U;
    axis->request_blocked = 0U;
}

/* 向一只编码器发送 BRT 协议 FUNC=0x01 的“读取多圈位置”请求。若该轴的“设置中点”
   请求正在等待应答（request_blocked为1），本次请求帧不发送，避免与中点重置期间
   的应答互相干扰；见EncoderAxisRequestSetMidpoint()/EncoderAxisParseFeedback()。 */
static void EncoderAxisRequestPosition(const EncoderAxis_TypeDef *axis)
{
    uint8_t data[ENCODER_POSITION_REQUEST_LENGTH];

    if (axis->request_blocked != 0U)
    {
        return;
    }

    data[0] = ENCODER_POSITION_REQUEST_LENGTH;
    data[1] = axis->node_id;
    data[2] = ENCODER_CMD_READ_POSITION;
    data[3] = 0U;

    /* 发送失败仅表示本帧未进入FIFO；下一周期会再次请求，不在这里做额外状态机。 */
    FDCANSendStandard(axis->FDCAN_Handle, axis->node_id, data, ENCODER_POSITION_REQUEST_LENGTH);
}

/* 向一只编码器发送 BRT 协议 FUNC=0x04 的“设置模式”请求，PARAM固定为0x00。
   成功应答为请求帧的原样回传，故这里先清空上一次的应答标记，避免把上一次
   遗留的旧应答误当作这次请求的确认（见EncoderAxisParseFeedback()）。 */
static void EncoderAxisRequestSetMode(EncoderAxis_TypeDef *axis)
{
    uint8_t data[ENCODER_MODE_REQUEST_LENGTH];

    data[0] = ENCODER_MODE_REQUEST_LENGTH;
    data[1] = axis->node_id;
    data[2] = ENCODER_CMD_SET_MODE;
    data[3] = ENCODER_MODE_REQUEST_PARAM;

    axis->mode_ack = 0U;

    FDCANSendStandard(axis->FDCAN_Handle, axis->node_id, data, ENCODER_MODE_REQUEST_LENGTH);
}

/* 向一只编码器发送 BRT 协议 FUNC=0x0C 的“设置多圈中点”请求：EncoderInit() 上电时
   发送一次，底盘运行中因编码器硬件圈数限制需要重新居中时也可再次调用。是否已
   完成重置以应答为准（见EncoderAxisParseFeedback()的应答分支），发送请求本身
   不代表编码器已完成内部重置，故这里只清空上一次的应答标记，避免把上一次遗留
   的旧应答误当作这次请求的确认。 */
static void EncoderAxisRequestSetMidpoint(EncoderAxis_TypeDef *axis)
{
    uint8_t data[ENCODER_MIDPOINT_REQUEST_LENGTH];

    data[0] = ENCODER_MIDPOINT_REQUEST_LENGTH;
    data[1] = axis->node_id;
    data[2] = ENCODER_CMD_SET_MIDPOINT;
    data[3] = ENCODER_MIDPOINT_REQUEST_PARAM;

    axis->midpoint_ack = 0U;

    FDCANSendStandard(axis->FDCAN_Handle, axis->node_id, data, ENCODER_MIDPOINT_REQUEST_LENGTH);

    /* 发送后立即置1，禁止位置查询请求帧发送，直到收到设置成功应答才在
       EncoderAxisParseFeedback()中清0。 */
    axis->request_blocked = 1U;
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

    /* BRT“设置模式”回复为请求帧的原样回传：04 ID 04 00，命中则说明模式设置成功，
       供EncoderInit()阻塞等待。 */
    if ((data[0] == ENCODER_MODE_REPLY_LENGTH) &&
        (data[1] == axis->node_id) &&
        (data[2] == ENCODER_CMD_SET_MODE) &&
        (data[3] == ENCODER_MODE_REPLY_STATUS_OK))
    {
        axis->mode_ack = 1U;
        return 1U;
    }

    /* BRT“设置多圈中点”回复固定为：04 ID 0C 00，00表示设置成功。只有收到这个应答
       才说明编码器已经完成内部重置，此时才置位midpoint_pending，让下一帧位置反馈
       重新建立增量计算基准；请求刚发出、应答尚未到达期间收到的位置反馈仍可能是
       重置前的陈旧计数，不能提前当作新基准。 */
    if ((data[0] == ENCODER_MIDPOINT_REPLY_LENGTH) &&
        (data[1] == axis->node_id) &&
        (data[2] == ENCODER_CMD_SET_MIDPOINT) &&
        (data[3] == ENCODER_MIDPOINT_REPLY_STATUS_OK))
    {
        axis->midpoint_ack = 1U;
        axis->midpoint_pending = 1U;
        axis->request_blocked = 0U;
        return 1U;
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

    axis->origin_position_count = axis->raw_position_count;
    axis->raw_position_count = position_count;
    if (axis->midpoint_pending != 0U)
    {
        /* 中点重置应答之后的首帧反馈：编码器内部计数发生了跳变，这一帧不能
           计入位移增量，只把它作为下一帧起计算增量的新基准，避免把中点重置
           造成的计数跳变累加进distance_m。 */
        axis->midpoint_pending = 0U;
    }
    else
    {
        /* 用相邻两帧的计数差换算为本帧滚动的角度、再按轮子周长换算为本帧滚动
           距离，直接累加进distance_m——浮点换算与累加都在本回调（FDCAN接收
           中断上下文）中一次性完成，EncoderUpdate()任务不再重复计算。 */
        int32_t delta_count;
        float angle_deg;
        float wheel_circumference_m;

        delta_count = (int32_t)axis->raw_position_count - (int32_t)axis->origin_position_count;
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

    /* 过滤器就绪后，各向两轴先发送一次"设置模式"请求并忙等其原样回传的成功应答，
       再发送一次"设置多圈中点"请求；中点请求的应答经FDCAN接收中断异步到达
       (HAL_FDCAN_RxFifo1Callback -> EncoderParseFeedback -> EncoderAxisParseFeedback)，
       此处仅在有限时间内忙等。增量计算基准完全依赖这次应答触发（见EncoderAxisParseFeedback()
       的midpoint_pending分支）：若超时仍未收到应答，对应轴的首帧位置反馈会把中点重置
       造成的计数跳变误计入distance_m。当前假设该应答一定会在超时前到达。 */

    EncoderAxisRequestSetMode(&encoder->x_axis);
    while (encoder->x_axis.mode_ack != 1)
    {
    }

    EncoderAxisRequestSetMode(&encoder->y_axis);
    while (encoder->y_axis.mode_ack != 1)
    {
    }

    EncoderAxisRequestSetMidpoint(&encoder->x_axis);
    while (encoder->x_axis.midpoint_pending != 1)
    {
    }

    EncoderAxisRequestSetMidpoint(&encoder->y_axis);
    while (encoder->y_axis.midpoint_pending != 1)
    {
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
        ChassisSetPosition(&Robot.chassis, Robot.encoder.x_m, Robot.encoder.y_m);
    }
}
