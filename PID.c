#include "PID.h"

void pid_init(struct PID_cfg_s *PID_cfg, float *I_term, float *D_term, float *P_term)
{
    PID_cfg->Kp     = pid_Kp;
    PID_cfg->Kd     = pid_Kd;
    PID_cfg->Ki     = pid_Ki;
    PID_cfg->I_term = *I_term;
    PID_cfg->D_term = *D_term;
    PID_cfg->P_term = *P_term;
    PID_cfg->PWM    = 255;
    PID_cfg->PWM_rdy = 0;
    PID_cfg->stage  = REAL;
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
    switch(PID->stage)
    {
        case REAL :
            RealQ = calc_measure(Measured, Set);
            PID->stage = DIFF;
            break;
        case DIFF :
            Diff = Set - RealQ;
            PID->stage = PTERM1;
            break;
        case PTERM1 :
            PID->P_term = (PID->Kp) * Diff;
            if(PID->P_term > 5000)
                PID->P_term = 5000;
            PID->stage = ITERM1;
            break;
        case ITERM1 :
            PID->I_term = PID->I_term + PID->Ki * Diff;
            if(PID->I_term > 5000)
                PID->I_term = 5000;
            PID->stage = DTERM1;
            break;
        case ITERM2 : break;
        case DTERM1 :
            PID->D_term = Diff - PID->D_term;
            if(PID->D_term > 5000)
                PID->D_term = 5000;
            PID->stage = DTERM2;
            break;
        case DTERM2 :
            PID->D_term = PID->Kd * PID->D_term;
            PID->stage = PWM;
            break;
        case PWM :
            PID->PWM = PID->PWM + (int)P_term + (int)PID->D_term + (int)PID->I_term;
            PID->stage = SCALE;
            break;
        case SCALE :
            PID->PWM = PID->PWM >> 5; // div by 32/1024
            PID->stage = SCORE;
            break;
        case SCORE :
            if(PID->PWM > 255) // constrain PWM range
                PID->PWM = 255;
            if(PID->PWM < 1)
                PID->PWM = 1;
            PID->PWM = 256 - PID->PWM;  // convert from duty cycle to duty ratio
            PID->PWM_rdy = 1;
            PID->P_term = 0;
            RealQ = 0;
            PID->stage = REAL;
            break;
        default : PID->stage = REAL; break;
    }
}

float calc_measure(unsigned long result, long Set)
{
    float temp = 0;
    float K = 11.35;
    if(Set > 4200 && Set < 7500)
        K = 10;
    else if (Set > 1200 && Set < 4300)
        K = 11.5;
    else
        K = 11.35;
    temp = 60*2000000*K/(float)result;
    return temp;
}
