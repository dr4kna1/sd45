/* 
 * File:   PID.h
 * Author: Ruslan
 *
 * Created on 13 ?????? 2018 ?., 20:46
 */

#ifndef PID_H
#define	PID_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* PID_H */

#define pid_Kp  8        // proportional coefficient
#define pid_Ki  0.03    // integral coefficient
#define pid_Kd  0.5     // differential coefficient

struct PID_cfg_s
{
    float Kp;
    float Ki;
    float Kd;
    float I_term;
    float D_term;
    long  PWM;
    char  PWM_rdy;
};

float I_term = 0;
float D_term = 0;
struct PID_cfg_s PID_cfg;

void pid_init(struct PID_cfg_s *PID_cfg, float *I_term, float *D_term);
void pid_task(unsigned long Measured, long Set, struct PID_cfg_s *PID);
float calc_measure(unsigned long result);
