#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "protocol.h" // for command_t

void motor_init(void);
void motor_apply_drive_new(const command_t *cmd);
int  read_safety_status(void);
void motor_apply_fail_safe(uint8_t brake_active);

void motor_set_target_speed(float speed_mm_s);

float motor_get_target_speed(void);

void motor_set_output(float pid_output);


#endif // motor_control.h

