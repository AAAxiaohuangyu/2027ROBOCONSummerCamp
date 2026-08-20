#include "test_zigbee.h"

#include "usart.h"
#include <string.h>

static ZigbeeHandle_TypeDef test_zigbee;

/* 保持为全局符号，便于在 Debug Watch 中直接观察 */
ZigbeeData_TypeDef test_tx_data;
ZigbeeData_TypeDef test_rx_data;

static TestZigbeeStatus_TypeDef test_status;


/*
 * 生成一帧测试数据。
 *
 * 每次调用后，速度和关节数据会发生变化，
 * 用于确认实际收到的确实是当前发送的数据。
 */
static void test_data_generate(ZigbeeData_TypeDef *data, uint32_t sequence)
{
    int16_t offset;

    offset = (int16_t)(sequence % 100U);

    data->chassis.speed_vx = (int16_t)(100 + offset);
    data->chassis.speed_vy = (int16_t)(-50 - offset);
    data->chassis.omega = (int16_t)(sequence % 3U);
    data->joint.front_back = (int16_t)(10 + offset);
    data->joint.up_down = (int16_t)(-20 - offset);
    data->joint.flip = (int16_t)(30 + offset);
    data->command.grab = (uint8_t)((sequence / 2U) & 0x01U);
    data->command.emergency_stop = ((sequence % 100U) == 99U) ? 1U : 0U;
}


/*
 * 比较发送数据和接收数据。
 */
static uint8_t test_data_compare(const ZigbeeData_TypeDef *tx_data, const ZigbeeData_TypeDef *rx_data)
{
    if (tx_data == NULL || rx_data == NULL)
        return 0U;
    
    if (tx_data->chassis.speed_vx != rx_data->chassis.speed_vx)
        return 0U;
    
    if (tx_data->chassis.speed_vy != rx_data->chassis.speed_vy)
        return 0U;
    
    if (tx_data->chassis.omega != rx_data->chassis.omega)
        return 0U;
    
    if (tx_data->joint.front_back != rx_data->joint.front_back)
        return 0U;
    

    if (tx_data->joint.up_down != rx_data->joint.up_down)
        return 0U;

    if (tx_data->joint.flip != rx_data->joint.flip)

        return 0U;
    
    if (tx_data->command.grab != rx_data->command.grab)
        return 0U;
    

    if (tx_data->command.emergency_stop != rx_data->command.emergency_stop)
        return 0U;
    
    return 1U;
}


/*
 * 初始化ZigBee回环测试。
 */
HAL_StatusTypeDef Test_Zigbee_Init(void)
{
    HAL_StatusTypeDef status;

    memset(&test_zigbee, 0, sizeof(test_zigbee));
    memset(&test_tx_data, 0, sizeof(test_tx_data));
    memset(&test_rx_data, 0, sizeof(test_rx_data));
    memset(&test_status, 0, sizeof(test_status));

    status = Zigbee_Init(&test_zigbee);

    if (status != HAL_OK)
    {
        test_status.error_count++;
        return status;
    }

    test_status.last_send_tick = HAL_GetTick();

    return HAL_OK;
}


/*
 * ZigBee回环测试周期任务。
 *
 * 建议在主循环中持续调用：
 *
 * while (1)
 * {
 *     Test_Zigbee_Task();
 * }
 */
void Test_Zigbee_Task(void)
{
    HAL_StatusTypeDef status;
    uint32_t now;

    now = HAL_GetTick();

    status = Zigbee_Receive(&test_zigbee, &test_rx_data);

    if (status == HAL_OK)
    {
        test_status.receive_count++;

        if (test_data_compare(&test_tx_data, &test_rx_data ) != 0U)
            test_status.pass_count++;
        else
            test_status.fail_count++;
    }

    /*
     * 定期发送新的测试帧。
     */
    if ((now - test_status.last_send_tick) < TEST_ZIGBEE_SEND_PERIOD_MS)
    {
        Zigbee_ErrorHandler(&test_zigbee);
        return;
    }

    test_status.last_send_tick = now;

    test_data_generate(&test_tx_data, test_status.send_count);

    status = Zigbee_Send(&test_zigbee, &test_tx_data);

    if (status == HAL_OK)
        test_status.send_count++;
    else if (status == HAL_BUSY)
        test_status.busy_count++;
    else
        test_status.error_count++;

    /*
     * 处理通信状态和超时。
     */
    Zigbee_ErrorHandler(&test_zigbee);
}


/*
 * 获取测试统计数据。
 */
const TestZigbeeStatus_TypeDef * Test_Zigbee_GetStatus(void)
{
    return &test_status;
}


/*
 * USART空闲线接收事件回调。
 *
 * HAL_UARTEx_ReceiveToIdle_DMA()
 * 收到一段数据后，会进入此回调。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &ZIGBEE_UART_HANDLE)
        return;

    Zigbee_RxEventHandler( &test_zigbee,huart, Size);
}
