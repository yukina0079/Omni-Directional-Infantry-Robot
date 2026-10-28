#include "Lowpass.h"
#include "systime.h"

LowPassFilter  LPF_current, LPF_velocity;

void LOWPass_Init(void)
{
    LPF_current.timestamp_prev = micros();
    LPF_current.y_prev = 0.0f;
    LPF_current.Tf = 0.05f;

    LPF_velocity.timestamp_prev = micros();
    LPF_velocity.y_prev = 0.0f;
    LPF_velocity.Tf = 0.01f;
}

float LowPassFilter_operator(LowPassFilter *Lfi, float x)
{

    float Ts = systime_delta_s(&Lfi->timestamp_prev);

    float alpha = Lfi->Tf / (Lfi->Tf + Ts);
    float y = alpha * Lfi->y_prev + (1.0f - alpha) * x;

    Lfi->y_prev = y;
    return y;
}
