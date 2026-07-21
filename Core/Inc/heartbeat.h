#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include <stdint.h>
#include "system_config.h"   /* HEARTBEAT_CAN_ID, HEARTBEAT_TIMEOUT_MS, HEARTBEAT_EXPECTED_MS buradan gelir */

typedef struct {
    uint32_t last_rx_tick;      /* Son heartbeat alınan zaman (ms)    */
    uint32_t rx_count;          /* Toplam alınan heartbeat sayısı     */
    uint32_t timeout_count;     /* Kaç kez timeout oldu?              */
    uint32_t last_sequence;     /* Son alınan sıra numarası           */
    uint8_t  is_alive;          /* 1: ROS hayatta, 0: bağlantı kopuk  */
    uint8_t  initialized;       /* İlk mesaj geldi mi?                */
} heartbeat_t;

void     heartbeat_init(heartbeat_t *hb);
void     heartbeat_kick(heartbeat_t *hb, const uint8_t *data);
void     heartbeat_check(heartbeat_t *hb);
uint8_t  heartbeat_is_alive(const heartbeat_t *hb);
uint32_t heartbeat_get_timeout_count(const heartbeat_t *hb);

#endif /* HEARTBEAT_H */
