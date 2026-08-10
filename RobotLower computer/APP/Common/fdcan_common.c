#include "fdcan_common.h"

//列表模式过滤器配置,以及中断的启用和fdcan的开启,
void FDCANStandardInit(FDCAN_HandleTypeDef *FDCAN_Handle,int StartID,int EndID)
{
    FDCAN_FilterTypeDef sfilter0 = {0};
    sfilter0.IdType = FDCAN_STANDARD_ID;
    sfilter0.FilterIndex = 0;
    sfilter0.FilterType = FDCAN_FILTER_RANGE;
    sfilter0.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sfilter0.FilterID1 = StartID;
    sfilter0.FilterID2 = EndID;
    HAL_FDCAN_ConfigFilter(FDCAN_Handle, &sfilter0);

    HAL_FDCAN_ActivateNotification(FDCAN_Handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    HAL_FDCAN_Start(FDCAN_Handle);
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
