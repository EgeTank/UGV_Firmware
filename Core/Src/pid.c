#include "pid.h"

static float holding_offset = 0.0f; // %45 eğim için minimum tutma torku

void PID_Init(PID_Controller_t *pid,
              float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;

    pid->out_min = out_min;
    pid->out_max = out_max;
}

/* ================= PID RESET ================= */
void PID_Reset(PID_Controller_t *pid)
{
    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
}

/* ================= HOLD OFFSET AYARI ================= */
/* %45 eğimde geri kaçmayı önleyecek minimum PWM */
void PID_SetHoldOffset(float offset)
{
    holding_offset = offset;
}

/* ================= PID COMPUTE ================= */
float PID_Compute(PID_Controller_t *pid,
                  float setpoint,
                  float measurement,
                  float dt)
{
    if (dt <= 0.0f)
        return 0.0f;

    float error = setpoint - measurement;

    /* ---------- Integral (anti-windup clamp) ---------- */
    pid->integral += error * dt;

    const float INTEGRAL_LIMIT = 200.0f;
    if (pid->integral > INTEGRAL_LIMIT)
        pid->integral = INTEGRAL_LIMIT;
    else if (pid->integral < -INTEGRAL_LIMIT)
        pid->integral = -INTEGRAL_LIMIT;

    float derivative = (error - pid->prev_error) / dt;

    float output =
        (pid->Kp * error) +
        (pid->Ki * pid->integral) +
        (pid->Kd * derivative);

    /* ---------- %45 EĞİM KOMPANZASYONU ---------- */
    if (setpoint != 0.0f)
    {
        output += holding_offset;
    }

    /* ---------- Output clamp ---------- */
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < pid->out_min)
        output = pid->out_min;

    pid->prev_error = error;
    return output;
}

float PID_Compute_WithFeedForward(PID_Controller_t *pid,
    float setpoint,
    float measurement,
    float dt,
    float feedforward)
{
    float pid_out = PID_Compute(pid, setpoint, measurement, dt);
    float output =  pid_out + feedforward ;
    if(output > pid->out_max)
    {
                output = pid->out_max;

    }
    else if (output < pid->out_min)
    {
        output = pid->out_min;
    }
        return output ;

}
