/*
   heartbeat.c — ROS Heartbeat İzleyici Implementasyonu

   2) main.c:
      #include "heartbeat.h"
      heartbeat_t hb;
      heartbeat_init(&hb);         — init bloğunda
      heartbeat_check(&hb);        — ana döngüde failsafe_check()'ten sonra
    */

#include "heartbeat.h"
#include "failsafe.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/*
   YARDIMCI: 4 byte little-endian → uint32
   */
static uint32_t read_u32_le(const uint8_t *in)
{
    return ((uint32_t)in[0] <<  0) |
           ((uint32_t)in[1] <<  8) |
           ((uint32_t)in[2] << 16) |
           ((uint32_t)in[3] << 24);
}

/*
   heartbeat_init()
    */
void heartbeat_init(heartbeat_t *hb)
{
    hb->last_rx_tick   = HAL_GetTick();
    hb->rx_count       = 0U;
    hb->timeout_count  = 0U;
    hb->last_sequence  = 0U;
    hb->is_alive       = 0U;
    hb->initialized    = 0U;
}

/*
   heartbeat_kick()
   can_process() içinden çağrılır.
   Her gelen heartbeat mesajında zamanlayıcıyı sıfırlar.
   */
void heartbeat_kick(heartbeat_t *hb, const uint8_t *data)
{
    uint32_t seq = read_u32_le(data);

    hb->last_rx_tick  = HAL_GetTick();
    hb->last_sequence = seq;
    hb->rx_count++;
    hb->is_alive    = 1U;
    hb->initialized = 1U;
}

/*
   heartbeat_check()
   Ana döngüde her iterasyonda çağrılır.

   İlk heartbeat gelmeden timeout başlatmaz —
   sistem yeni açıldıysa ROS henüz başlamamış olabilir.

   Timeout olursa:
     → failsafe_force() çağrılır (hard latch)
     → Motorlar anında kilitlenir
     → Sadece MCU reseti açar
    */
void heartbeat_check(heartbeat_t *hb)
{
    /* İlk heartbeat henüz gelmedi — beklemeye devam */
    if (!hb->initialized)
        return;

    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - hb->last_rx_tick;

    if (elapsed > HEARTBEAT_TIMEOUT_MS)
    {
        hb->is_alive = 0U;
        hb->timeout_count++;

        /* Motorları anında kilitle — hard latch */
        failsafe_force();
    }
}

/* ============================================================
   Yardımcı sorgular
   ============================================================ */
uint8_t heartbeat_is_alive(const heartbeat_t *hb)
{
    return hb->is_alive;
}

uint32_t heartbeat_get_timeout_count(const heartbeat_t *hb)
{
    return hb->timeout_count;
}
