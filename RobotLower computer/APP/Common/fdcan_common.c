#include "fdcan_common.h"

// 列表模式过滤器配置,以及中断的启用和fdcan的开启,过滤器序号与目标FIFO均可指定
void FDCANFilterInit(FDCAN_HandleTypeDef *FDCAN_Handle, uint32_t FilterIndex,
                      int StartID, int EndID, uint32_t FilterConfig)
{
    FDCAN_FilterTypeDef sfilter = {0};
    sfilter.IdType = FDCAN_STANDARD_ID;
    sfilter.FilterIndex = FilterIndex;
    sfilter.FilterType = FDCAN_FILTER_RANGE;
    sfilter.FilterConfig = FilterConfig;
    sfilter.FilterID1 = StartID;
    sfilter.FilterID2 = EndID;
    HAL_FDCAN_ConfigFilter(FDCAN_Handle, &sfilter);

    HAL_FDCAN_ActivateNotification(FDCAN_Handle,
        (FilterConfig == FDCAN_FILTER_TO_RXFIFO1) ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                                                    : FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0);

    HAL_FDCAN_Start(FDCAN_Handle);
}

// 默认使用过滤器0,目标fifo0
void FDCANStandardInit(FDCAN_HandleTypeDef *FDCAN_Handle, int StartID, int EndID)
{
    FDCANFilterInit(FDCAN_Handle, 0, StartID, EndID, FDCAN_FILTER_TO_RXFIFO0);
}

void FDCANSendStandard(FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t std_id, uint8_t *data, uint8_t length)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier = std_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = (length == 8) ? FDCAN_DLC_BYTES_8 : (length == 7) ? FDCAN_DLC_BYTES_7
                                                          : (length == 6)   ? FDCAN_DLC_BYTES_6
                                                          : (length == 5)   ? FDCAN_DLC_BYTES_5
                                                          : (length == 4)   ? FDCAN_DLC_BYTES_4
                                                          : (length == 3)   ? FDCAN_DLC_BYTES_3
                                                          : (length == 2)   ? FDCAN_DLC_BYTES_2
                                                          : (length == 1)   ? FDCAN_DLC_BYTES_1
                                                                            : FDCAN_DLC_BYTES_0;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_Handle, &TxHeader, data);
}
