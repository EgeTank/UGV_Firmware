#include "blackbox.h"
#include "stm32h7xx_hal.h"
#include <string.h>

static uint8_t calculate_checksum(const uint8_t *data, uint16_t len) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len; i++) { sum ^= data[i]; }
    return sum;
}

/* SRAM'deki Header ve Data alanlarının adresleri */
#define BKP_HEADER_PTR ((blackbox_header_t*)BLACKBOX_BKP_SRAM_ADDR)
#define BKP_RECORDS_PTR ((blackbox_record_t*)(BLACKBOX_BKP_SRAM_ADDR + sizeof(blackbox_header_t)))

void blackbox_init(void)
{
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPRAM_CLK_ENABLE();

    /* Bellek ilk kez kullanılıyorsa (Magic Word yoksa) formatla */
    if (BKP_HEADER_PTR->magic_word != BLACKBOX_MAGIC_WORD) {
        memset((void*)BLACKBOX_BKP_SRAM_ADDR, 0, 4096);
        BKP_HEADER_PTR->magic_word = BLACKBOX_MAGIC_WORD;
        BKP_HEADER_PTR->head_idx = 0;
        BKP_HEADER_PTR->is_locked = 0;
    }
}

/* Her 100ms'de çağrılıp halka tampona yazar */
void blackbox_log_step(const blackbox_record_t *record)
{
    if (BKP_HEADER_PTR->is_locked) return; /* Kilitliyse geçmişi ezme */

    uint16_t data_len = sizeof(blackbox_record_t) - 1;
    blackbox_record_t safe_copy;
    memcpy(&safe_copy, record, sizeof(blackbox_record_t));
    safe_copy.checksum = calculate_checksum((const uint8_t*)&safe_copy, data_len);

    uint16_t idx = BKP_HEADER_PTR->head_idx;
    memcpy(&BKP_RECORDS_PTR[idx], &safe_copy, sizeof(blackbox_record_t));

    /* İndeksi ilerlet (Ring Buffer) */
    BKP_HEADER_PTR->head_idx = (idx + 1) % BLACKBOX_MAX_RECORDS;
}

/* Kaza anında (failsafe_force) çağrılır */
void blackbox_lock_and_save_crash(const blackbox_record_t *final_record)
{
    if (BKP_HEADER_PTR->is_locked) return;
    blackbox_log_step(final_record); /* Son anı da kaydet */
    BKP_HEADER_PTR->is_locked = 1;   /* Sistemi kilitle */
}

/* Okuma ve temizleme */
uint8_t blackbox_read_and_clear(blackbox_record_t *out_records, uint16_t *out_count)
{
    if (BKP_HEADER_PTR->magic_word != BLACKBOX_MAGIC_WORD) return 0;

    if (out_records != NULL) {
        memcpy(out_records, BKP_RECORDS_PTR, sizeof(blackbox_record_t) * BLACKBOX_MAX_RECORDS);
    }
    if (out_count != NULL) {
        *out_count = BLACKBOX_MAX_RECORDS;
    }

    memset((void*)BLACKBOX_BKP_SRAM_ADDR, 0, 4096);
    return 1;
}
