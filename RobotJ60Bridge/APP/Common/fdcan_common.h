#ifndef __FDCAN_COMMON_H_
#define __FDCAN_COMMON_H_

#include "fdcan.h"

void FDCANSendStandard(FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t std_id, uint8_t *data, uint8_t length);
void FDCANStandardInit(FDCAN_HandleTypeDef *FDCAN_Handle, int StartID, int EndID);
void FDCANFilterInit(FDCAN_HandleTypeDef *FDCAN_Handle, uint32_t FilterIndex, int StartID, int EndID, uint32_t FilterConfig);
void FDCANExtendedFilterInit(FDCAN_HandleTypeDef *FDCAN_Handle, uint32_t FilterIndex, uint32_t StartID, uint32_t EndID, uint32_t FilterConfig);
void FDCANExtendedInit(FDCAN_HandleTypeDef *FDCAN_Handle, uint32_t ExtID);

#endif
