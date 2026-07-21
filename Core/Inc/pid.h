#ifndef PID_H
#define PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float prev_error;
    float integral;

    float out_min;
    float out_max;
} PID_Controller_t;

void PID_Init(PID_Controller_t *pid,
              float kp, float ki, float kd,
              float out_min, float out_max);

float PID_Compute(PID_Controller_t *pid,
                  float setpoint,
                  float measurement,
                  float dt);

void PID_Reset(PID_Controller_t *pid);

void PID_SetHoldOffset(float offset);

float PID_Compute_WithFeedForward(PID_Controller_t *pid,
    float setpoint,
    float measurement,
    float dt,
    float feedforward);

#endif
