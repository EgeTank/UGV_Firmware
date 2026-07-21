/*
 * ============================================================
 * failsafe.c — Hard/Soft Latch + EV200 Kontaktör Kontrolü
 * ============================================================
 * GÜNCELLEME: failsafe_force() içine EV200 kontaktör açma eklendi.
 * Hard latch tetiklendiğinde 48V batarya hattı da kesilir.
 * Siyah Kutu (Black Box) kaza anı kaydı entegre edildi.
 * ============================================================
 */

#include "failsafe.h"
#include "stm32h7xx_hal.h"
#include "system_config.h"
#include "motor_control.h"
#include "brake_control.h"
#include "fdcan.h"

/* --- SİYAH KUTU EKLENTİSİ İÇİN GEREKEN KÜTÜPHANELER --- */
#include "blackbox.h"
#include "state_machine.h"
#include "encoder_reader.h"
#include "bms_emus_can.h"
#include "odrive_sm.h"  /* ODrive durum yapılarını okuyabilmek için */


/* --- DIŞARIDAN OKUNACAK GLOBAL DEĞİŞKENLER --- */
extern uint8_t g_errorMask;
extern float g_targetSpeed_mm_s;
extern BMS_Data_t g_bmsData;
extern volatile float g_imu_angle_deg;
extern vehicle_state_t g_vehicleState;
/* -------------------------------------------- */

volatile uint32_t last_command_rx_time = 0;
#define COMM_TIMEOUT_MS CMD_TIMEOUT_MS

static volatile uint8_t soft_latch = 0;
static volatile uint8_t hard_latch = 0;

static void apply_brakes_and_stop(void)
{
    motor_apply_fail_safe(1);
    brake_apply();
}

/* ============================================================
   FAIL-SAFE INIT
   EV200 kontaktörü başlangıçta KAPALI (bağlı) tutulur.
============================================================ */
void failsafe_init(void)
{
    soft_latch = 0;
    hard_latch = 0;

    /* EV200 kontaktörü kapat — sisteme 48V ver */
    HAL_GPIO_WritePin(EV200_GPIO_PORT, EV200_PIN, GPIO_PIN_SET);

    apply_brakes_and_stop();
}

/* ============================================================
   failsafe_kick() — CAN komutu geldiğinde çağrılır
   SADECE soft_latch'i sıfırlar, hard_latch'e DOKUNMAZ.
============================================================ */
void failsafe_kick(void)
{
    last_command_rx_time = HAL_GetTick();

    if (!hard_latch)
    {
        soft_latch = 0;
    }
}

/* ============================================================
   failsafe_force() — E-STOP / Kritik Hata (ISR'dan çağrılabilir)

   Sıralama kritik:
     1) EV200 aç  → 48V batarya hattı fiziksel olarak kesilir
     2) Motor durdur + Frenler kilitle
     3) Siyah Kutu kaydı oluştur
     4) hard_latch set → yazılımdan kurtarma engellenir
   ============================================================ */
void failsafe_force(void)
{
    /* --- 1) ODRIVE'I ACİL OLARAK IDLE MODUNA AL (ISR GÜVENLİ) --- */
    /* Bu fonksiyon EXTI kesmesinden çağrılabileceği için RTOS Mutex
       kullanan can_send_Nbyte() fonksiyonunu kullanamayız! Doğrudan donanıma yazarız. */
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    TxHeader.Identifier          = ODRIVE_CAN_ID(ODRIVE_NODE_ID, 0x007U); /* 0x007 = Set_Axis_State */
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = FDCAN_DLC_BYTES_4;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_ON;
    TxHeader.FDFormat            = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;

    uint8_t idle_data[4] = {1, 0, 0, 0}; /* 1 = ODRIVE_AXIS_STATE_IDLE */
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, idle_data);

    /* --- 2) MOTOR DURDUR + FRENLERİ KİLİTLE --- */
    apply_brakes_and_stop();

    /* --- 3) CAN MESAJININ İLETİLMESİ İÇİN KISA BEKLEME (ISR GÜVENLİ) --- */
    /* ISR içinde olduğumuz için HAL_Delay() kullanılamaz, CPU'yu kısa süre meşgul ediyoruz */
    for (volatile uint32_t i = 0; i < 50000; i++) {
        __NOP();
    }

    /* --- 4) EN SON EV200 KONTAKTÖRÜ AÇ — 48V BATARYAYI KES --- */
    HAL_GPIO_WritePin(EV200_GPIO_PORT, EV200_PIN, GPIO_PIN_RESET);

    /* --- 5) SİYAH KUTU KAYDI --- */
    if (!hard_latch) { /* Sadece ilk tetiklenişte kaydet */
        blackbox_record_t crash_data = {0};
        crash_data.crash_timestamp_ms  = HAL_GetTick();
        crash_data.error_mask          = g_errorMask;
        crash_data.vehicle_state       = g_vehicleState;
        crash_data.target_speed_mm_s   = g_targetSpeed_mm_s;

        /* Kaza anında hızı TIM3 yerine ODrive üzerinden kaydet */
        const odrive_state_t *odrv_fb = odrive_get_state();
        crash_data.measured_speed_mm_s = (odrv_fb != NULL) ? (odrive_turns_per_s_to_mps(odrv_fb->encoder.vel_estimate) * 1000.0f) : 0.0f;

        crash_data.imu_angle_deg       = g_imu_angle_deg;
        crash_data.bus_voltage_v       = BMS_GetAvgCellVoltage(&g_bmsData) * (float)g_bmsData.cells_received;
        crash_data.current_a           = g_bmsData.current_A; /* EKLENEN AKIM */
        crash_data.max_cell_temp_c     = (int8_t)BMS_GetMaxCellTemp(&g_bmsData);

        /* ODRIVE VERİSİNİ GERÇEK HALİYLE KAYDET */
        if (odrv_fb != NULL) {
            crash_data.odrive_axis_error   = odrv_fb->heartbeat.axis_error;
            crash_data.odrive_axis_state   = odrv_fb->heartbeat.axis_state;
        } else {
            crash_data.odrive_axis_error   = 0;
            crash_data.odrive_axis_state   = 0;
        }

        blackbox_lock_and_save_crash(&crash_data);
    }

    /* 6) HARD LATCH — sadece MCU reseti temizler */
    hard_latch = 1;
}

/* ============================================================
   failsafe_check() — Main loop'ta periyodik olarak çağrılır
   Soft latch EV200'ü AÇMAZ — sadece motoru durdurur.
============================================================ */
void failsafe_check(void)
{
    if (hard_latch)
        return;

    uint32_t now = HAL_GetTick();

    if ((now - last_command_rx_time) > COMM_TIMEOUT_MS)
    {
        soft_latch = 1;
        apply_brakes_and_stop();
    }
}

uint8_t failsafe_is_active(void)
{
    return (soft_latch || hard_latch) ? 1u : 0u;
}

uint8_t failsafe_is_hard_locked(void)
{
    return hard_latch;
}
