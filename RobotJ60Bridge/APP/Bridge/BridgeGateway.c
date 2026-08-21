#include "BridgeGateway.h"
#include <string.h>

static UART_HandleTypeDef *bridge_uart;
static FDCAN_HandleTypeDef *bridge_can;
static uint8_t uart_rx[BRIDGE_FRAME_SIZE];
static uint8_t uart_tx[BRIDGE_FRAME_SIZE];

static uint8_t BridgeCrc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0U;
    uint16_t index;

    for (index = 0U; index < length; ++index)
        crc ^= data[index];

    return crc;
}

static uint32_t BridgeDlcToHal(uint8_t dlc)
{
    static const uint32_t dlc_table[9] = {
        FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8};

    return dlc_table[dlc];
}

void BridgeGateway_Init(UART_HandleTypeDef *huart, FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    bridge_uart = huart;
    bridge_can = hfdcan;

    /* 接收所有标准数据帧；J60 反馈 ID 由主控在回传后自行解析。 */
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_RANGE;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0x7FFU;
    HAL_FDCAN_ConfigFilter(bridge_can, &filter);
    HAL_FDCAN_ActivateNotification(bridge_can, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    HAL_FDCAN_Start(bridge_can);

    HAL_UARTEx_ReceiveToIdle_DMA(bridge_uart, uart_rx, BRIDGE_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(bridge_uart->hdmarx, DMA_IT_HT);
}

void BridgeGateway_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if (huart == bridge_uart && size == BRIDGE_FRAME_SIZE && uart_rx[0] == BRIDGE_SOF0 &&
        uart_rx[1] == BRIDGE_SOF1 && uart_rx[2] == BRIDGE_TYPE_CAN_TX && uart_rx[5] <= 8U &&
        uart_rx[BRIDGE_FRAME_SIZE - 1U] == BridgeCrc8(&uart_rx[2], BRIDGE_FRAME_SIZE - 3U))
    {
        /* 主控传来的是原始 J60 标准 CAN 帧，网关不修改 CAN ID、DLC 或数据。 */
        header.Identifier = (uint16_t)uart_rx[3] | ((uint16_t)uart_rx[4] << 8U);
        header.IdType = FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = BridgeDlcToHal(uart_rx[5]);
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        HAL_FDCAN_AddMessageToTxFifoQ(bridge_can, &header, &uart_rx[6]);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(bridge_uart, uart_rx, BRIDGE_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(bridge_uart->hdmarx, DMA_IT_HT);
}

void BridgeGateway_CanRxEvent(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_its)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (hfdcan != bridge_can || (rx_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U ||
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        return;

    /* 回传格式与主控命令帧一致，仅 TYPE 改为 CAN_RX，CAN DLC 从 HAL 编码还原。 */
    memset(uart_tx, 0, sizeof(uart_tx));
    uart_tx[0] = BRIDGE_SOF0;
    uart_tx[1] = BRIDGE_SOF1;
    uart_tx[2] = BRIDGE_TYPE_CAN_RX;
    uart_tx[3] = (uint8_t)header.Identifier;
    uart_tx[4] = (uint8_t)(header.Identifier >> 8U);
    uart_tx[5] = (uint8_t)(header.DataLength >> 16U);
    memcpy(&uart_tx[6], data, uart_tx[5]);
    uart_tx[BRIDGE_FRAME_SIZE - 1U] = BridgeCrc8(&uart_tx[2], BRIDGE_FRAME_SIZE - 3U);
    HAL_UART_Transmit_DMA(bridge_uart, uart_tx, BRIDGE_FRAME_SIZE);
}
