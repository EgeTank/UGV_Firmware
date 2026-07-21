#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

// ===============================
// CAN ID'LERİ
// ===============================
#define CAN_ID_DRIVE_CMD    0x100U
#define CAN_ID_DRIVE_RPT    0x101U
#define CAN_ID_BMS_RPT      0x102U   /* BMS telemetri paketi (MCU → ROS) */
#define CAN_ID_STATUS       0x110U

// ===============================
// ZAMANLAMA
// ===============================
#define REPORT_INTERVAL_MS  10U
#define PID_INTERVAL_MS     10U

// ===============================
// PWM / MOTOR DONANIM
// ===============================
#define MAX_DUTY_CYCLE      3200
#define PWM_FREQUENCY_HZ    20000
#define MOTOR_PWM_TIM       htim1
#define MOTOR_PWM_CHANNEL   TIM_CHANNEL_1
#define FDCANx              hfdcan1

// ===============================
// CRC
// ===============================
#define CRC_OK              1
#define CRC_FAIL            0

// ===============================
// FAIL-SAFE TIMEOUT
// CAN komutu gelmezse soft latch devreye girer
// ===============================
#define CMD_TIMEOUT_MS      500U

// ===============================
// SAFETY INPUT PINS
// ===============================

// E-STOP (Active LOW)
#define ESTOP_GPIO_PORT     GPIOA
#define ESTOP_PIN           GPIO_PIN_0

// BMS FAULT (Active LOW)
#define BMS_GPIO_PORT       GPIOA
#define BMS_PIN             GPIO_PIN_1

// EV200 Kontaktör — 48V Batarya Ana Anahtarı (Active HIGH = KAPALI/Bağlı)
#define EV200_GPIO_PORT     GPIOB
#define EV200_PIN           GPIO_PIN_8

// Elektromanyetik Fren Kontrolü
#define BRAKE_GPIO_PORT     GPIOB
#define BRAKE_PIN           GPIO_PIN_5

#define BRAKE_BUTTON_GPIO_PORT GPIOB
#define BRAKE_BUTTON_PIN       GPIO_PIN_6

// ===============================
// MEKANİK PARAMETRELER
// Donanım gelince buradan güncellenir.
// unit_conversion.h ve encoder_reader.c bu değerleri kullanır.
// ===============================
#define WHEEL_DIAMETER_M        0.33f   /* Tekerlek çapı (metre)          */
#define GEAR_REDUCTION_RATIO    19.0f   /* Motor devri / tekerlek devri   */

// ===============================
// PID KATSAYILARI
// Ürün gelince burada optimize edilir,
// main.c'ye dokunmaya gerek kalmaz.
// ===============================
#define PID_KP              1.2f
#define PID_KI              0.05f
#define PID_KD              0.01f
#define PID_OUT_MIN        -100.0f
#define PID_OUT_MAX         100.0f
#define PID_HOLD_OFFSET     10.0f   /* %45 eğimde geri kaçmayı önleyen holding tork */

// ===============================
// GRAVITY FEEDFORWARD
// %45 eğim için deneysel besleme — donanımda ölçülerek güncellenecek
// ===============================
#define GRAVITY_FEEDFORWARD 30.0f

// ===============================
// HIZ PROFİLİ (S-CURVE / TRAPEZOID)
// Donanım gelince tune edilir.
// ===============================
#define VEL_PROFILE_MAX_ACCEL_MM_S2    500.0f   /* Maksimum ivme      (mm/s²) */
#define VEL_PROFILE_MAX_DECEL_MM_S2    800.0f   /* Maksimum yavaşlama (mm/s²) */
#define VEL_PROFILE_MAX_JERK_MM_S3    2000.0f   /* Maksimum jerk      (mm/s³) */

// ===============================
// AKIM LİMİTLEYİCİ
// 48V sistem için donanım koruma sabitleri.
// ===============================
#define CURRENT_LIM_NOMINAL_A       40.0f   /* Normal operasyon limiti (A)  */
#define CURRENT_LIM_PEAK_A          60.0f   /* Kısa süreli peak limiti (A)  */
#define CURRENT_LIM_MIN_A            5.0f   /* Minimum işletme akımı   (A)  */
#define CURRENT_LIM_VOLTAGE_NOMINAL 48.0f   /* Nominal bus voltajı     (V)  */
#define CURRENT_LIM_VOLTAGE_MIN     42.0f   /* Altında limit düşer     (V)  */
#define CURRENT_LIM_TEMP_NOMINAL    60.0f   /* Tam limit sıcaklığı    (°C)  */
#define CURRENT_LIM_TEMP_MAX        85.0f   /* Minimum limit sıcaklığı(°C)  */
#define CURRENT_LIM_SOFTSTART_MS   500U     /* Soft-start süresi       (ms) */

// ===============================
// HEARTBEAT İZLEYİCİ
// ROS bağlantı kontrol sabitleri.
// ===============================
#define HEARTBEAT_CAN_ID        0x0FFU   /* ROS heartbeat mesaj ID'si   */
#define HEARTBEAT_TIMEOUT_MS    500U     /* 3 mesaj kaçırılırsa timeout */
#define HEARTBEAT_EXPECTED_MS   100U     /* ROS gönderim periyodu  (ms) */


#define BMS_MIN_CELL_VOLTAGE_V  3.0f
#define BMS_MAX_CELL_TEMP_C     65.0f //Saha testlerinden sonra güncellenmesi gerekebilir.

// ===============================
// ODrive DURUM MAKİNESİ
// ===============================
#define ODRIVE_SM_TIMEOUT_MS    2000U   /* CLOSED_LOOP geçiş timeout (ms) */

//================================
//IMU VE SIMÜLASYON PAKETLERİ
#define IMU_FILTER_ALPHA   0.98f  /* Tamamlayıcı filtre katsayısı (Genelde 0.95 - 0.98 arası idealdir) */
#define IMU_SIM_NOISE_SCALE  1.5 /* Simülasyonda eklenecek gürültü miktarı (derece) */
#endif /* SYSTEM_CONFIG_H */
