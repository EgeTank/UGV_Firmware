#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdint.h>


void failsafe_init(void);

/* CAN mesajı gelince çağrılır — sadece soft_latch'i temizler */
void failsafe_kick(void);

/* Main loop'ta timeout kontrolü */
void failsafe_check(void);

/* Genel durum sorgusu: soft VEYA hard latch aktifse 1 döner */
uint8_t failsafe_is_active(void);

/* Hard latch aktif mi? (E-STOP / kritik hata) */
uint8_t failsafe_is_hard_locked(void);

/* ZORUNLU FAIL-SAFE — hard_latch'i set eder (ISR'dan çağrılabilir)
   Sistem resetine kadar CAN mesajlarıyla AÇILAMAZ.              */
void failsafe_force(void);

#endif /* FAILSAFE_H */
