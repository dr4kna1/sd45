#include "PID.h"

void pid_init(struct PID_cfg_s *PID_cfg, float *I_term, float *D_term)
{
    PID_cfg->Kp     = pid_Kp;
    PID_cfg->Kd     = pid_Kd;
    PID_cfg->Ki     = pid_Ki;
    PID_cfg->I_term = *I_term;
    PID_cfg->D_term = *D_term;
    PID_cfg->PWM    = 255;
}

/*
 * Calculate regulating coefficient for PWM
 * Parameters:
 * 1 - measured period of sense impulse
 * 2 - impulse period that correspond desired FLow rate
 * 3 - PID configuration - coefficients and terms memory allocation
 */
void pid_task(unsigned long Measured, unsigned long Set, struct PID_cfg_s *PID)
{
    float P_term = 0;
    P_term = (PID->Kp) * (Set - Measured);
    PID->I_term = PID->I_term + PID->Ki * (Set - Measured);
    PID->D_term = (Set - Measured) - PID->D_term;
    PID->D_term = PID->Kd * PID->D_term;
    
    
}

