/**
 * @file    valve_tx.h
 * @brief   电磁阀控制帧（0x18FF1D60）组包
 */
#ifndef __VALVE_TX_H
#define __VALVE_TX_H

#include <stdint.h>
#include "../CAN/can_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VALVE_FRAME_ID   0x18FF1D60U

int  valve_tx_init(void);
void valve_tx_pack(can_tx_frame_t *frame);
void valve_tx_notify_change(void);   /* [线程安全] valve_output 变化时调用 */

#ifdef __cplusplus
}
#endif
#endif /* __VALVE_TX_H */
