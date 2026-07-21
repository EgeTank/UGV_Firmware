#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#include <stdint.h>

/*
 * BIT HARİTASI:
 *
 *  Bit 0 — ERR_FAILSAFE_ACTIVE   : Soft veya hard latch aktif
 *  Bit 1 — ERR_CAN_TX            : CAN gönderme hatası
 *  Bit 2 — ERR_CAN_RX            : CAN alma hatası
 *  Bit 3 — ERR_CAN_PROTOCOL      : CRC veya paket format hatası
 *  Bit 4 — ERR_ODRIVE_FAULT      : ODrive hata durumunda (axis_error != 0)
 *  Bit 5 — ERR_ESTOP_ACTIVE      : Fiziksel E-STOP butonuna basıldı
 *  Bit 6 — ERR_BMS_FAULT         : BMS hata sinyali aktif
 *  Bit 7 — ERR_HARD_LATCH        : Hard latch — MCU reseti gerekir
 *
 * GENIŞLETILMIŞ HATA DETAYLARI (error_detail alanı için — gelecekte):
 *
 *  Bit 0 — ERR_DETAIL_LOW_VOLTAGE      : Bus voltajı < 42V
 *  Bit 1 — ERR_DETAIL_HIGH_TEMP        : Motor sıcaklığı > 85°C
 *  Bit 2 — ERR_DETAIL_CURRENT_LIMIT    : Akım limiti devreye girdi
 *  Bit 3 — ERR_DETAIL_ENCODER_ANOMALY  : Encoder anomalisi tespit edildi
 *  Bit 4 — ERR_DETAIL_HEARTBEAT_LOST   : ROS heartbeat kesildi
 *  Bit 5 — ERR_DETAIL_WATCHDOG_RESET   : IWDG reset nedeniyle başlatıldı
 *  Bit 6 — ERR_DETAIL_ODRIVE_TIMEOUT   : ODrive CLOSED_LOOP geçiş timeout
 *  Bit 7 — ERR_DETAIL_CAN_TIMEOUT      : CAN komutu 500ms gelmedi
 * ============================================================
 */

/* -------------------------------------------------------
   Ana Hata Maskesi (hata alanı — 8 bit)
   ------------------------------------------------------- */
#define ERR_FAILSAFE_ACTIVE     (1u << 0)   /* Soft veya hard latch aktif          */
#define ERR_CAN_TX              (1u << 1)   /* CAN gönderme hatası                 */
#define ERR_CAN_RX              (1u << 2)   /* CAN alma hatası                     */
#define ERR_CAN_PROTOCOL        (1u << 3)   /* CRC veya paket format hatası        */
#define ERR_ODRIVE_FAULT        (1u << 4)   /* ODrive axis_error != 0              */
#define ERR_ESTOP_ACTIVE        (1u << 5)   /* Fiziksel E-STOP basıldı             */
#define ERR_BMS_FAULT           (1u << 6)   /* BMS hata sinyali aktif              */
#define ERR_HARD_LATCH          (1u << 7)   /* Hard latch — MCU reseti gerekir     */

/*
   Genişletilmiş Hata Detay Maskesi (error_detail — 8 bit)

   */
#define ERR_DETAIL_LOW_VOLTAGE      (1u << 0)   /* Bus voltajı < CURRENT_LIM_VOLTAGE_MIN  */
#define ERR_DETAIL_HIGH_TEMP        (1u << 1)   /* Sıcaklık > CURRENT_LIM_TEMP_MAX        */
#define ERR_DETAIL_CURRENT_LIMIT    (1u << 2)   /* Akım limiti devreye girdi              */
#define ERR_DETAIL_ENCODER_ANOMALY  (1u << 3)   /* Encoder anomalisi tespit edildi        */
#define ERR_DETAIL_HEARTBEAT_LOST   (1u << 4)   /* ROS heartbeat kesildi                  */
#define ERR_DETAIL_WATCHDOG_RESET   (1u << 5)   /* IWDG reset nedeniyle başlatıldı        */
#define ERR_DETAIL_ODRIVE_TIMEOUT   (1u << 6)   /* ODrive geçiş timeout                   */
#define ERR_DETAIL_CAN_TIMEOUT      (1u << 7)   /* CAN komutu 500ms gelmedi               */

/*
   Yardımcı Makrolar
   */

/* Hata maskesine bit ekle */
#define ERR_SET(mask, bit)      ((mask) |= (bit))

/* Hata maskesinden bit temizle */
#define ERR_CLEAR(mask, bit)    ((mask) &= ~(bit))

/* Bit aktif mi? */
#define ERR_IS_SET(mask, bit)   (((mask) & (bit)) != 0u)

/* Tüm hataları temizle */
#define ERR_CLEAR_ALL(mask)     ((mask) = 0u)

/* Kritik hata var mı? (Hard latch veya E-STOP) */
#define ERR_IS_CRITICAL(mask)   (ERR_IS_SET(mask, ERR_HARD_LATCH) || \
                                  ERR_IS_SET(mask, ERR_ESTOP_ACTIVE))

#endif /* ERROR_CODES_H */
