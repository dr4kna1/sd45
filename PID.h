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

#define pid_Kp  4        // proportional coefficient
#define pid_Ki  0.001    // integral coefficient
#define pid_Kd  0.03     // differential coefficient

struct PID_cfg_s
{
    float Kp;
    float Ki;
    float Kd;
    float I_term;
    float D_term;
    float P_term;
    long  PWM;
    char  PWM_rdy;
    enum {REAL = 0, DIFF = 1, PTERM1 = 2, ITERM1 = 3, ITERM2 = 4, DTERM1 = 5,
    DTERM2 = 6, PWM = 7, SCALE = 8, SCORE = 9} stage;
};

float I_term = 0;
float D_term = 0;
float P_term = 0;
float  Diff = 0;
float RealQ = 0;
struct PID_cfg_s PID_cfg;

void pid_init(struct PID_cfg_s *PID_cfg, float *I_term, float *D_term, float *P_term);
void pid_task(unsigned long Measured, long Set, struct PID_cfg_s *PID);
float calc_measure(unsigned long result, long Set);
