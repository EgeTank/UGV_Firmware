//Elektromanyetik fren sistemi için yazılım.


#ifndef BRAKE_CONTROL_H
#define BRAKE_CONTROL_H

#include <stdint.h>

/* Başlatma: sistem açılır açılmaz fren KAPALI */
void brake_init(void);

/* Freni AÇ (sadece güvenliyse çağrılmalı) */
void brake_release(void);

/* Freni KAPAT (fail-safe, e-stop, timeout) */
void brake_apply(void);

/* Fren durumu */
uint8_t brake_is_applied(void);

#endif
