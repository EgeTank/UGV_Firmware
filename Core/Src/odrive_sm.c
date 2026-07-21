#include "odrive_sm.h"
#include "state_machine.h"
#include "can_comm.h"
#include "stm32h7xx_hal.h"

/* ============================================================
   YARDIMCI: ODrive'a axis state komutu gönder
   Set_Axis_State mesajı: Cmd_ID = 0x007, 4 byte (u32)
   ============================================================ */
static void send_axis_state(uint32_t requested_state)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)((requested_state >>  0) & 0xFFu);
    buf[1] = (uint8_t)((requested_state >>  8) & 0xFFu);
    buf[2] = (uint8_t)((requested_state >> 16) & 0xFFu);
    buf[3] = (uint8_t)((requested_state >> 24) & 0xFFu);

    can_send_Nbyte(
        ODRIVE_CAN_ID(ODRIVE_NODE_ID, 0x007U),  /* Set_Axis_State */
        buf, 4);
}


void odrive_sm_init(odrive_sm_t *sm)
{
    sm->state        = ODRIVE_SM_IDLE;
    sm->request_tick = 0U;
    sm->timeout_ms   = ODRIVE_SM_TIMEOUT_MS;
    sm->error_count  = 0U;

    /* Başlangıçta ODrive'ı IDLE'a al */
    send_axis_state(ODRIVE_AXIS_STATE_IDLE);
}

/* DÜZELTME: Fonksiyon artık freertos.c'den gelen vehicle_state parametresini kullanıyor */
void odrive_sm_update(odrive_sm_t *sm, vehicle_state_t vehicle_state)
{
    const odrive_state_t *odrv    = odrive_get_state();
    uint32_t now                  = HAL_GetTick();

    switch (sm->state)
    {
        /* ── IDLE ──────────────────────────────────────────── */
        case ODRIVE_SM_IDLE:
            /* Araç DRIVING moduna geçtiyse CLOSED_LOOP iste */
            if (vehicle_state == STATE_DRIVING)
            {
                send_axis_state(ODRIVE_AXIS_STATE_CLOSED_LOOP);
                sm->request_tick = now;
                sm->state        = ODRIVE_SM_REQUESTING_CLOSED_LOOP;
            }
            break;

        /* ── CLOSED_LOOP İSTEĞİ BEKLENİYOR ────────────────── */
        case ODRIVE_SM_REQUESTING_CLOSED_LOOP:
            /* Araç DRIVING'den çıktıysa isteği iptal et */
            if (vehicle_state != STATE_DRIVING)
            {
                send_axis_state(ODRIVE_AXIS_STATE_IDLE);
                sm->request_tick = now;
                sm->state        = ODRIVE_SM_REQUESTING_IDLE;
                break;
            }

            /* ODrive CLOSED_LOOP'a girdi mi? */
            if (odrv->heartbeat.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP)
            {
                sm->state = ODRIVE_SM_CLOSED_LOOP;
                break;
            }

            /* Timeout kontrolü */
            if ((now - sm->request_tick) > sm->timeout_ms)
            {
                sm->error_count++;
                send_axis_state(ODRIVE_AXIS_STATE_IDLE);
                sm->state = ODRIVE_SM_IDLE;
            }
            break;

        /* ── CLOSED_LOOP ────────────────────────────────────── */
        case ODRIVE_SM_CLOSED_LOOP:
            /* ODrive hata verdiyse IDLE'a dön */
            if (odrv->heartbeat.axis_error != 0U)
            {
                sm->error_count++;
                send_axis_state(ODRIVE_AXIS_STATE_IDLE);
                sm->request_tick = now;
                sm->state        = ODRIVE_SM_REQUESTING_IDLE;
                break;
            }

            /* Araç DRIVING'den çıktıysa IDLE iste */
            if (vehicle_state != STATE_DRIVING)
            {
                send_axis_state(ODRIVE_AXIS_STATE_IDLE);
                sm->request_tick = now;
                sm->state        = ODRIVE_SM_REQUESTING_IDLE;
            }
            break;

        /* ── IDLE İSTEĞİ BEKLENİYOR ───────────────────────── */
        case ODRIVE_SM_REQUESTING_IDLE:
            /* ODrive IDLE'a girdi mi? */
            if (odrv->heartbeat.axis_state == ODRIVE_AXIS_STATE_IDLE)
            {
                sm->state = ODRIVE_SM_IDLE;
                break;
            }

            /* Timeout kontrolü */
            if ((now - sm->request_tick) > sm->timeout_ms)
            {
                sm->error_count++;
                sm->state = ODRIVE_SM_ERROR;
            }
            break;

        /* ── HATA ───────────────────────────────────────────── */
        case ODRIVE_SM_ERROR:
            /* Sürekli IDLE komutu gönder — kurtarma dene */
            if ((now - sm->request_tick) > 1000U)
            {
                send_axis_state(ODRIVE_AXIS_STATE_IDLE);
                sm->request_tick = now;

                /* ODrive IDLE'a döndüyse hatadan çık */
                if (odrv->heartbeat.axis_state == ODRIVE_AXIS_STATE_IDLE &&
                    odrv->heartbeat.axis_error == 0U)
                {
                    sm->state = ODRIVE_SM_IDLE;
                }
            }
            break;

        default:
            sm->state = ODRIVE_SM_IDLE;
            break;
    }
}

uint8_t odrive_sm_is_closed_loop(const odrive_sm_t *sm)
{
    return (sm->state == ODRIVE_SM_CLOSED_LOOP) ? 1u : 0u;
}

uint32_t odrive_sm_get_error_count(const odrive_sm_t *sm)
{
    return sm->error_count;
}

void odrive_sm_force_idle(odrive_sm_t *sm)
{
    send_axis_state(ODRIVE_AXIS_STATE_IDLE);
    sm->request_tick = HAL_GetTick();
    sm->state        = ODRIVE_SM_REQUESTING_IDLE;
}
