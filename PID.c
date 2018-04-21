#include "PID.h"

void pid_init(struct PID_cfg_s *PID_cfg, float *I_term, float *D_term)
{
    PID_cfg->Kp     = pid_Kp;
    PID_cfg->Kd     = pid_Kd;
    PID_cfg->Ki     = pid_Ki;
    PID_cfg->I_term = *I_term;
    PID_cfg->D_term = *D_term;
    PID_cfg->PWM    = 255;
    PID_cfg->PWM_rdy = 0;
}

/*
 * Calculate regulating coefficient for PWM
 * Parameters:
 * 1 - measured period of sense impulse
 * 2 - impulse period that correspond desired FLow rate
 * 3 - PID configuration - coefficients and terms memory allocation
 */
void pid_task(unsigned long Measured, long Set, struct PID_cfg_s *PID)
{   
    float Real = 0;
    Real = calc_measure(Measured);
    
    float P_term = 0;
    P_term = (PID->Kp) * (Set - Real);
    PID->I_term = PID->I_term + PID->Ki * (Set - Real);
    PID->D_term = (Set - Real) - PID->D_term;
    PID->D_term = PID->Kd * PID->D_term;
    PID->PWM = PID->PWM + P_term + PID->D_term + PID->I_term;
    PID->PWM = PID->PWM / 1000;
    if(PID->PWM > 255) // constrain PWM range
        PID->PWM = 255;
    if(PID->PWM < 1)
        PID->PWM = 1;
    PID->PWM = 256 - PID->PWM;  // convert from duty cycle to duty ratio 
    PID->PWM_rdy = 1;
}

float calc_measure(unsigned long result)
{
    float temp = 0;
    temp = 60*2000000*11.35/(float)result;
    return temp;
}
