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
#include "tables.h"        /* calibrated flow, PWM and other tabs values */
#include "PID.h"
#include "display_7seg.h"

#include <stdio.h>
#include <stdlib.h>

/* Main cycle and interrupt routine */

void main(void)
{
//    PWR_ON = 0b1;
    arr_p = (void *)meash_arr;
    mcu_init();
    spi_init();
    pid_init(&PID_cfg, &I_term, &D_term, &P_term);
    get_settings();
    mode_MAN = 0;                                       // Init main control modes
    mode_AUTO = 0;
    mode_SET = 0;
    while(1)
    {
        prcd_but();                                     // get buttons state
        if(PWR_ON)
        {
            arr_num = binarySearch(arr_p,RESLT,767,0);      // get decimal flow via table
            if(mode_SET)                                    // write chosen flow rate to ROM
            {
                if(norm_num != prev_norm_num)
                    ROM_WR(0X10,norm_num);
            }
            Set_Flow = set_Q[norm_num];
            if((mass_locked && mode_AUTO) || mode_MAN)
            {
                if(!manpwm_info)
                    pid_task(RESLT,Set_Flow,&PID_cfg);      // Calculate PWM value
                else
                    pid_reset(&PID_cfg);
            }
            else
            {
                RESLT = 2840238;
            }
            prev_norm_num = norm_num;
            lit_led(norm_arr[norm_num],rate_arr[arr_num]);
            ADC_task(&ADC_data);
            set_PWM();                                      // drive pump with calculated PWM
            meter_task();
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
            PID_timer = 32;
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
}
