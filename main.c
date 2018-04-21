#if defined(__XC)
    #include <xc.h>        /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>       /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>   /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */

#endif

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include "macros.h"
#include "SPI_ex.h"
#include "tables.h"        /* calibrated flow, PWM and other tabe values */
#include "PID.h"

#include <stdio.h>
#include <stdlib.h>

/* Main cycle and interrupt routine */

unsigned char fUP = 1;
unsigned char fDOWN = 1;
unsigned char fSET = 1;
unsigned char fMAN = 1;
unsigned char fAUTO = 1;
unsigned char f_measured = 0;


char mode_MAN = 0;
char mode_AUTO = 0;
char mode_SET = 0;

void main(void)
{
    arr_p = (void *)meash_arr;
    InitApp();
    initSPI();
 //   pid_init(&PID_cfg, &I_term, &D_term);
    get_settings();
    mode_MAN = 0;                                       // Init main control modes
    mode_AUTO = 0;
    mode_SET = 0;
    while(1)
    {
        prcd_but();                                     // get buttons state
        if(PWR_ON)
        {
            measure();                                      // calculate flow based on feedback
            if(f_measured == 1)                             // flow calculated
                arr_num = binarySearch(arr_p,RESLT,767,0);  // set digit display with flow value
            else
            if(f_measured == 0 && ACTV == 2)                // flow measure timed-out/no signal from sensor
                arr_num = 767;                              // null flow
            if(mode_SET)                                    // write chosen flow rate to ROM
            {
                if(norm_num != prev_norm_num)
                    ROM_WR(0x10,norm_num);
            }
            prev_norm_num = norm_num;
            lit_led(norm_arr[norm_num],rate_arr[arr_num],adc_conv_cnt);
            ADC_task(&ADC_data);
            drive_pump(pwm_p,per_pwm_p);
        }
        else
        {
            PORTD = 0xFF;   // switch off screen
            disable_pump();
            indAUTO = 0;
            indMANUAL = 0;
            indSET  = 0;
            indPUMP = 0;
            mode_MAN = 0;
            mode_AUTO = 0;
            mode_SET  = 0;
        }
    }
}

/* IRQ routine */
void interrupt sys_irq (void)
{
  if (PIR2bits.TMR3IF)
    {
        irq_tmr3();                                 // full timer3 cycle
    }
  else if (PIR2bits.CCP2IF)
  {
      irq_ccp2();                                   // timer3 capture with event

  }
  else if (PIR1bits.TMR1IF)                         // main timer, drives leds
  {
     irq_tmr1();
  }
  else if (PIR1bits.SSPIF)
  {
      PIR1bits.SSPIF = 0;
  }

}
