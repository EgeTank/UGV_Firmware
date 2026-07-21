#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdint.h>

#define BLACKBOX_BKP_SRAM_ADDR    ((uint32_t)0x38800000U)
#define BLACKBOX_MAGIC_WORD       0xB1ACBB01U

#define BLACKBOX_MAX_RECORDS      100U

#pragma pack(push, 1)

typedef struct {
    uint32_t crash_timestamp_ms;  /* Kazanın HAL_GetTick() zamanı */
    uint8_t  error_mask;          /* g_errorMask değeri */
    uint8_t  vehicle_state;       /* Kaza anındaki STATE */
    float    target_speed_mm_s;   /* İstenen hız */
    float    measured_speed_mm_s; /* Okunan gerçek hız */
    float    imu_angle_deg;       /* Eğim/Devrilme durumu */
    float    bus_voltage_v;       /* BMS'den gelen voltaj */
    float    current_a;           /* EKLENEN: BMS Anlık Akım */
    int8_t   max_cell_temp_c;     /* Maksimum hücre sıcaklığı */
    uint32_t odrive_axis_error;   /* ODrive hata kodu */
    uint8_t  odrive_axis_state;   /* ODrive o anki state'i */
    uint8_t  checksum;            /* Veri bütünlüğü için XOR Checksum */
} blackbox_record_t;

typedef struct {
    uint32_t magic_word;
    uint16_t head_idx;
    uint8_t  is_locked;
    uint8_t  padding;
} blackbox_header_t;

#pragma pack(pop)

void    blackbox_init(void);
void    blackbox_log_step(const blackbox_record_t *record);
void    blackbox_lock_and_save_crash(const blackbox_record_t *final_record);
uint8_t blackbox_read_and_clear(blackbox_record_t *out_records, uint16_t *out_count);

#endif
