// can_comm.h
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include <stdint.h>
#include "protocol.h"

void can_init(void);
int  can_send_Nbyte(uint16_t std_id, const uint8_t *data, uint8_t len);
int  can_send_report(const report_t *rpt);/* 0x101 — hareket telemetrisi */
int  can_send_bms_report(const bms_report_t *bms);/* 0x102 — BMS telemetrisi  */

//yukarıda ros tarafına gönderilen veri paketleri var bunlara filtrelenme uygulanması lazım.

/* CAN hata sayaçları */
uint32_t can_get_tx_error_count(void);
uint32_t can_get_rx_error_count(void);
uint32_t can_get_protocol_error_count(void);

#endif

