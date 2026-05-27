#ifndef __BOARD_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#define __BOARD_LINK_H__

#include "fdcan.h"
#include "stdint.h"

typedef enum
{
    BOARD_LINK_RX_UNHANDLED = 0,
    BOARD_LINK_RX_HANDLED,
} BoardLinkDispatchStatus_e;

void BoardLink_Init(void);
void BoardLink_TxStep(void);
uint8_t BoardLink_RxDispatch(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t data[8]);

#ifdef __cplusplus
}
#endif
#endif
