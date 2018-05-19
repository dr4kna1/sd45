/* 
 * File:   display_7seg.c
 * Author: r.d.
 *
 * Created on 13 may 2018, 17:32
 */

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#elif defined(__18CXX)
    #include <p18cxxx.h>    /* C18 General Include File */
#endif

#if defined(__XC) || defined(HI_TECH_C)

#include <stdint.h>         /* For uint8_t definition */
#include <stdbool.h>        /* For true/false definition */

#endif

#include "display_7seg.h"
#include "macros.h"
#include "user.h"
#include "tables.h"         /* calibrated flow, PWM and other tabs values */
#include "PID.h"

/* Trailing one through anodes port*/
void prcd_led1(void)
{
     AN0 = 1; AN1 = 0; AN2 = 0; AN3 = 0; AN4 = 0; AN5 = 0;
}
void prcd_led2(void)
{
     AN0 = 0; AN1 = 1; AN2 = 0; AN3 = 0; AN4 = 0; AN5 = 0;
}
void prcd_led3(void)
{
     AN0 = 0; AN1 = 0; AN2 = 1; AN3 = 0; AN4 = 0; AN5 = 0;
}
void prcd_led4(void)
{
     AN0 = 0; AN1 = 0; AN2 = 0; AN3 = 1; AN4 = 0; AN5 = 0;
}
void prcd_led5(void)
{
     AN0 = 0; AN1 = 0; AN2 = 0; AN3 = 0; AN4 = 1; AN5 = 0;
}
void prcd_led6(void)
{
     AN0 = 0; AN1 = 0; AN2 = 0; AN3 = 0; AN4 = 0; AN5 = 1;
}

/* Consequentially light up 7-seg LEDs;
 * input:
 * str1 - desired chosen flow rate,
 * str2 - measured flow rate from sensor  */
#if PCB_rev == 0
void lit_led(unsigned int str1,unsigned int str2)
{
    int nstr;
    // light up modes LEDs
    indAUTO     = mode_AUTO;
    indMANUAL   = mode_MAN;
    indSET      = mode_SET;

    // 7-seg LEDs
    if(led_state < 7)
        {
            PORTD = 0xFF;
            switch (led_state)
            {
                case 1  :    
                    // LED1 on, other off
                    if (service_info)
                    {
                        prcd_led1();
                        nstr = (str2&0x0F00)>>8;
                        PORTD = decode_str(nstr); break;                                          // lit '1'
                    }
                    else
                        break;
                case 2  :
                    if (service_info)
                    {
                        prcd_led2(); PORTD = decode_str((str2&0x00F0)>>4); break;
                    }
                    else
                        break;
                case 3  :
                    if (service_info)
                    {
                        prcd_led3(); PORTD = decode_str(str2&0x000F); break;
                    }
                    else
                        break;
                case 4  :
                    prcd_led4(); PORTD = decode_str((str1&0x0F00)>>8); break;
                case 5  :
                    prcd_led5(); PORTD = decode_str((str1&0x00F0)>>4); break;
                case 6  :
                    prcd_led6(); PORTD = decode_str(str1&0x000F); break;
                default: break;
            }
        }
        else
            led_state = 0;
    
       led_state++;
}
#elif PCB_rev == 1
void lit_led(unsigned int str1,unsigned int str2)
{
    int nstr;
    // light up modes LEDs
    indAUTO     = mode_AUTO;
    indMANUAL   = mode_MAN;
    indSET      = mode_SET;
    
    if(calibration_info == 0 & manpwm_info == 0)
    {
        // 7-seg LEDs
        if(led_state < 7)
          {
              PORTD = 0xFF;
              switch (led_state)
                {
                    case 1  :    
                        // LED1 on, other off
                        if (service_info)
                        {
                            prcd_led1();
                            nstr = (str2&0x0F00)>>8;
                            PORTD = decode_str(nstr); break;                                          // lit '1'
                        }
                        else
                            break;
                    case 2  :
                        if (service_info)
                        {
                            prcd_led2(); PORTD = decode_str((str2&0x00F0)>>4) | 0x10; break;    // suppress dot 0x10
                        }
                        else
                            break;
                    case 3  :
                        if (service_info)
                        {
                            prcd_led3(); PORTD = decode_str(str2&0x000F) | 0x10; break;
                        }
                        else
                            break;
                    case 4  :
                        prcd_led4(); PORTD = decode_str((str1&0x0F00)>>8); break;
                    case 5  :
                        prcd_led5(); PORTD = decode_str((str1&0x00F0)>>4) | 0x10; break;
                    case 6  :
                        prcd_led6(); PORTD = decode_str(str1&0x000F) | 0x10; break;
                    default: break;
                }
          }
            else
                led_state = 0;
    
       led_state++;
    }
    else if (calibration_info == 0 & manpwm_info == 1)  // Manual PWM
    {
        // 7-seg LEDs
        if(led_state < 7)
          {
              PORTD = 0xFF;
              switch (led_state)
                {
                    case 1  :    
                        // LED1 on, other off
                        prcd_led1(); PORTD = decode_str((PID_cfg.PWM&0x0F00)>>8) | 0x10; break;
                    case 2  :
                        prcd_led2(); PORTD = decode_str((PID_cfg.PWM&0x00F0)>>4) | 0x10; break;    // suppress dot 0x10
                    case 3  :
                        prcd_led3(); PORTD = decode_str(PID_cfg.PWM&0x000F) | 0x10; break;
                    case 4  :
                        prcd_led4(); PORTD = 0x44 | 0x10; break;    //"P"
                    case 5  :
                        prcd_led5(); PORTD = 0xA0 | 0x10; break;    //"Y"
                    case 6  :
                        prcd_led6(); PORTD = 0xE0 | 0x10; break;    //"?"
                    default: break;
                }
          }
            else
                led_state = 0;
    
       led_state++;        
    }
    else if (calibration_info == 1)
    {
        if(led_state < 7)
        {
            PORTD = 0xFF;
            switch (led_state)
            {
                case 1  :    
                // LED1 on, other off
                        prcd_led1(); PORTD = decode_str((arr_hexdec_p[adc_conv_cnt]&0x0F00)>>8) | 0x10; break;
                    case 2  :
                        prcd_led2(); PORTD = decode_str((arr_hexdec_p[adc_conv_cnt]&0x00F0)>>4) | 0x10; break;
                    case 3  :
                        prcd_led3(); PORTD = decode_str((arr_hexdec_p[adc_conv_cnt]&0x000F)) | 0x10; break;
                    case 4  :
                        prcd_led4(); PORTD = 0x07 | 0x10; break; // display 'C'
                    case 5  :
                        prcd_led5(); PORTD = 0x27 | 0x10; break;
                    case 6  :
                        prcd_led6(); PORTD = 0x21 | 0x10; break;
                    default: break;
                }
          }
            else
                led_state = 0;
    
       led_state++;
    }
}
#endif

/* Decode hex int to 7-seg digit*/
int decode_str(int str)
{
#if PCB_rev == 0
    int dec_str = 0xDF;
    if     (str == 0x0)         dec_str = 0x20;
    else if(str == 0x1)         dec_str = 0x79;
    else if(str == 0x2)         dec_str = 0x44;
    else if(str == 0x3)         dec_str = 0x50;
    else if(str == 0x4)         dec_str = 0x19;
    else if(str == 0x5)         dec_str = 0x12;
    else if(str == 0x6)         dec_str = 0x02;
    else if(str == 0x7)         dec_str = 0x38;
    else if(str == 0x8)         dec_str = 0x00;
    else if(str == 0x9)         dec_str = 0x10;
    else
         dec_str = 0xDF;
    return dec_str;
#elif PCB_rev == 1
    int dec_str = 0xFD;
    if     (str == 0x0)         dec_str = 0x02;
    else if(str == 0x1)         dec_str = 0xFA;
    else if(str == 0x2)         dec_str = 0x0C;
    else if(str == 0x3)         dec_str = 0x88;
    else if(str == 0x4)         dec_str = 0xE0;
    else if(str == 0x5)         dec_str = 0x81;
    else if(str == 0x6)         dec_str = 0x01;
    else if(str == 0x7)         dec_str = 0xC2;
    else if(str == 0x8)         dec_str = 0x00;
    else if(str == 0x9)         dec_str = 0x80;
    else if(str == 0xA)         dec_str = 0x40;
    else if(str == 0xB)         dec_str = 0x21;
    else if(str == 0xC)         dec_str = 0x07;
    else if(str == 0xD)         dec_str = 0x28;
    else if(str == 0xE)         dec_str = 0x05;
    else if(str == 0xF)         dec_str = 0x45; //0x45;
    else if(str == 0x10)        dec_str = 0x27; // symbol 'L'
    else
         dec_str = 0xFD;
    return dec_str;    
#endif
}

void display_task(void)
{
    unsigned int upper_string = 0,
                 lower_string = 0;
// Decide what to display
    if(calibration_info == 0 & manpwm_info == 0)
    {
        if(service_info)
            lower_string = rate_arr[arr_num];
        else
            lower_string = 0xFF;    // lights off
        upper_string = norm_arr[norm_num];
    }
    else if (calibration_info == 0 & manpwm_info == 1)  // Manual PWM
    {
        upper_string = 0x17;    // 'CAL'
        lower_string = arr_hexdec_p[adc_conv_cnt];   
    }
    else if (calibration_info == 1)
    {
        upper_string = 0x17;    // 'CAL'
        lower_string = arr_hexdec_p[adc_conv_cnt];
    }
}
