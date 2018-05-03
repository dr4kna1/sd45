/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

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

#include "user.h"
#include "macros.h"
#include "tables.h"
#include "SPI_ex.h"
#include "PID.h"

unsigned int i=0;
unsigned int j = 1;
unsigned char ki = 0;
//int k = 0;                              // cntr index
int i1 = 0;                               // buttons delay cntrs
int i2 = 0;
int i3 = 0;
int i4 = 0;
int i5 = 0;
extern char mode_MAN;
extern char mode_AUTO;
extern char mode_SET;

extern int mes_num ;
extern char ACTV;

unsigned int string = 0x0;
unsigned int dig_i = 0;
unsigned char flash = 0;
unsigned int norma_10b = 916;
unsigned int norma_8b = 228;

extern unsigned char f_measured;
extern unsigned char fUP;
unsigned char prev_UP = 1;
extern unsigned char fDOWN;
unsigned char prev_DOWN = 1;
extern unsigned char fSET;
unsigned char prev_SET = 1;
extern unsigned char fMAN;
unsigned char prev_MAN = 1;
extern unsigned char fAUTO;
unsigned char prev_AUTO = 1;

                                //0    1    2    3    4    5    6    7    8    9
const unsigned int digit[10] = {0x20,0x79,0x44,0x50,0x19,0x12,0x02,0x38,0x00,0x10};
unsigned int ioch_reg = 0;
extern unsigned char state_up ;
extern unsigned char auto_state;
extern unsigned char manual_state;
extern unsigned char pump_led;
extern unsigned char set_led;
extern unsigned char led_state;
unsigned char pwm_state = 0;
/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

void InitApp(void)
{
    OSCCONbits.IRCF = 0b111;            // 8 MHz INTOSC
    PORTA = 0x00;
    TRISA = 0x00;

    PORTB = 0x00;
    TRISB = 0b11111000;

    PORTC = 0x00;
    TRISC = 0b10010011;
    TRISCbits.RC1 = 1;          // CCP2 input

    PORTD = 0x0;
    TRISD = 0x0;

    PORTE = 0x0;
    TRISE = 0;
    TRISEbits.RE0 = 0;

    /* interrupts */
    INTCON2 = 0b00000000;
    INTCONbits.GIEH = 1;	// 1- enable all unmasked intr
    INTCONbits.GIEL = 1;	// 1 - enable peripheral intr
    INTCONbits.TMR0IE = 0;      // TMR0 intr

    INTCONbits.TMR0IE = 0;      // irq TMR0
    PIE1bits.TMR1IE = 0;
    PIE2bits.TMR3IE = 1;            // irq enabled

    PIE2bits.CCP2IE = 1;            // enable irq CCP2
    PIE1bits.SSPIE = 1;

    /* TIMER0 */
    T0CONbits.T08BIT = 0;       // 0 - 16 bit mode; 1 - 8 bit
    T0CONbits.T0CS	= 0;    //1 = Transition on T0CKI pin; 0 = Internal instruction cycle clock (CLKO)
    T0CON = 0x00; // prescaler on, tmr0 off, prescale ratio 1:2
    T0CONbits.TMR0ON = 0;

    /* TIMER1 */
    T1CONbits.T1RUN = 0;	// additional osc source
    T1CONbits.T1CKPS = 0b11;	// prescaler
    T1CONbits.T1OSCEN = 0;		// osc off
    T1CONbits.TMR1CS = 0;		// internal clk Fosc/4
    T1CONbits.TMR1ON = 0;       // 1 = on

    /* Timer2 */
    T2CON = 0b01111011;     // set prescale and postscale tmr2
    PR2 = 0xFF;

     /* Timer3 */
    T3CONbits.T3CKPS = 0b00;             // prescaler 1:1
    T3CONbits.T3CCP2 = 0;           // TmR3 clk src for CCP2
    T3CONbits.T3CCP1 = 1;           // TmR1 clk src for CCP1
    T3CONbits.TMR3ON = 1;

    /*CCP1*/
    CCPR1L  = 0x0F;
    CCP1CON = 0;      // Enable PWM 1

   /*CCP2*/
    CCP2CONbits.CCP2M = 0x4;       // CCP2 Capture rising edge

}

/* TMR3 full cycle triggered */
void irq_tmr3(void)
{
    PIR2bits.TMR3IF = 0;
        PID_timer++;
        if(PID_timer > 32)
            PID_timer = 32;
        PER0 = PER0 + 0xFFFF;
        if(PER0>0x4AC000)
        {
            RESLT = 2840238;
            ACTV = 2;
            f_measured = 0;
        }
}

/* Capture event on input and load TMR3 value */
void irq_ccp2(void)
{
    //clear CCP2 irq flag
    PIR2bits.CCP2IF = 0;
    if((mes_num) > 0)
    {
        ACTV = 1;
    }                       // period measurment started

    if((mes_num <= measure_num) & ACTV==1)
    {
        PER0 = PER0+((CCPR2H<<8) + CCPR2L);
        PER1[mes_num-1] = PER0;
        PER0 = 0;

         /*Enable irqs there if needed*/
        j = !j;
        indPUMP =  j;
    }
    TMR3H = 0;                      // reload timer
    TMR3L = 0;

    mes_num++;                      // incr meashurments cntr

}

void irq_tmr1()
{
   PIR1bits.TMR1IF = 0;
   TMR1H = 0xFE;
   TMR1L = 0x00;
 //  led_state++;

   fAUTO = AUTO;
   fDOWN = DOWN;
   fMAN = MANUAL;
   fUP = UP;
   fSET = SET;
}

void measure(void)
{
    
    if(mes_num == measure_num)
        {
        int k = 0;
        // disable irqs, as we don't need new data in PER1
            PIE2bits.TMR3IE = 0;
            PIE2bits.CCP2IE = 0;
            f_measured = 1;
            ACTV = 0;
            PER2 = sumarr(PER1);
            PER2 = PER2>>grade;             // divide by grade
            RESLT = PER2;
            if(RESLT < 173821)
                RESLT = 173821;
            k = 0;
             while(k < 8)                 // reset buffer per1
            {
                PER1[k] = 0;
                k++;
            }
            mes_num = 0;                // end measurment session
            // return back irqs
            PIE2bits.TMR3IE = 1;
            PIE2bits.CCP2IE = 1;
        }
}

unsigned long sumarr(unsigned long arr[])
{
    int k = 0;
    unsigned long rslt = 0;
    for(k=0;k < measure_num;k++)
    {
        rslt = rslt + arr[k];
    }
    return rslt;
}

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
void lit_led(unsigned int str1,unsigned int str2, unsigned int adc_cnt)
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
    else if (calibration_info == 0 & manpwm_info == 1)
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
                        prcd_led4(); PORTD = 0x44 | 0x10; break;
                    case 5  :
                        prcd_led5(); PORTD = 0xA0 | 0x10; break;
                    case 6  :
                        prcd_led6(); PORTD = 0xE0 | 0x10; break;
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
                        prcd_led1(); PORTD = decode_str((arr_hexdec_p[adc_cnt]&0x0F00)>>8) | 0x10; break;
                    case 2  :
                        prcd_led2(); PORTD = decode_str((arr_hexdec_p[adc_cnt]&0x00F0)>>4) | 0x10; break;
                    case 3  :
                        prcd_led3(); PORTD = decode_str((arr_hexdec_p[adc_cnt]&0x000F)) | 0x10; break;
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
    else if(str == 0xA)         dec_str = 0xB0;
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

int binarySearch(unsigned long *tab1, unsigned long key, int high, int low)
{
	int middle;
	while(low <= high)
	{
	middle = (low+high)>>1;
	if(key == tab1[middle])
		return middle;
	else if(key < tab1[middle])
		if((middle-low)==1)
			if((tab1[middle]-key) >= (key - tab1[low]))
				return middle;
			else
				return low;
		else
			high = middle;
	else if(key > tab1[middle])
		if(high - middle == 1)
			if((tab1[high]-key >= (key - tab1[middle])))
				return high;
			else
				return low;
		else
			low = middle;

	}
	return -1;
}

void prcd_but(void)
{

    const int ps = 200;
    const int ts = 200;

    fAUTO = AUTO;
    fDOWN = DOWN;
    fMAN  = MANUAL;
    fUP   = UP;
    fSET  = SET;
    
    pwron_task();
    if(PWR_ON)
    {
		if(!fUP & !fDOWN)   // UP and DOWN pushed simultaneously
		{
			service_cnt += 1;
		}
		if (service_cnt == 1500)
		{
			service_info = !service_info;
			service_cnt=0;
		}
		
		indUP = !fUP;
		if(fUP > prev_UP)
		{
			if(mode_SET & !calibration_info)
			{
				norm_num++;
				pwm_state = 0;
				if(norm_num >= 71)
						norm_num = 70;
			}
		}
		prev_UP = fUP;
/*-----------------------------------------------------------------------*/
		indDOWN = !fDOWN;
		if(fDOWN > prev_DOWN)
		{
			if(mode_SET & !calibration_info)
			{
				pwm_state = 0;
				if(norm_num <= 0)
					norm_num = 0;
				else
					norm_num--;
			}
		}
		prev_DOWN = fDOWN;
	
/*-----------------------------------------------------------------------*/
		if(fSET > prev_SET)
		{
            if(!calibration_info)
            {
                mode_SET = !mode_SET;
                prev_SET = 1;
            }
            else
            {
                mode_SET = 0;
            }
		}
		prev_SET = fSET;
/*-----------------------------------------------------------------------*/
        
        if(!fAUTO & !AUTO_forbid)
        {
            auto_cnt += 1;
            cal_info_prev = 0;
        }
        else
            auto_cnt = 0;
        if(auto_cnt == 3000)
        {   
            AUTO_btn_hold = 0b1;
            auto_cnt = 0;
            AUTO_rlsd = 0b0;
        }
        // AUTO button was hold during period and then released
        if(AUTO_btn_hold)
        {
            AUTO_btn_hold = 0;
            cal_info_prev = calibration_info;
            calibration_info = !calibration_info;
            if(adc_conv_cnt == 64)
                calibration_act = 0b0;
            else
                calibration_act = 0b1;
            adc_conv_cnt = 0;
            auto_cnt = 0;
        }
        if(!AUTO_rlsd)
            AUTO_forbid = 0b1;
        else
            AUTO_forbid = 0b0;
        
		if(fAUTO > prev_AUTO)
		{
            if(!calibration_info & !cal_info_prev)
            {
                mode_AUTO = !mode_AUTO;
                mode_MAN = 0;
            }
            AUTO_rlsd = 0b1;
		}
            prev_AUTO = fAUTO;
/*-----------------------------------------------------------------------*/
        if(!fMAN & !MAN_forbid)
        {
            man_cnt++;
            manpwm_info_prev = 0b0;
        }
        else
            man_cnt = 0;
        if(man_cnt == 3000)
        {
            MAN_btn_hold = 0b1;
            MAN_rlsd     = 0b0;
            man_cnt      = 0;
        }
        if(MAN_btn_hold)
        {
            MAN_btn_hold = 0b0;
            manpwm_info_prev = manpwm_info;
            manpwm_info = !manpwm_info;
            man_cnt = 0;
        }
        if(!MAN_rlsd)
            MAN_forbid = 0b1;
        else
            MAN_forbid = 0b0;
            
		if(fMAN > prev_MAN)
		{
            if(!manpwm_info & !calibration_info)
            {
    			mode_MAN = !mode_MAN;
        		mode_AUTO = 0;
            }
            MAN_rlsd = 0b1;
		}
		prev_MAN = fMAN;
    }

}

#if PWM_capacity == 8
void drive_pump( unsigned int *num,  unsigned long *table)
{
    const unsigned char ps = 1;//60;
    
 // drive pump if mass present or manual guidance
    if(((mass_locked && mode_AUTO) || mode_MAN)&&PWR_ON)
    {
        TRISCbits.RC2 = 0;                          // set PWM output
        CCP1CONbits.DC1B = 0b11;                    // 2 LSB of CCP1 reg = 3, so we have 8 bit DUTY_CYCLE resolution  
            PR2 = 0xFF;                                 // max tmr2 period (485HZ @ 8MHz)
        T2CONbits.TMR2ON = 1;                       // start tmr2
        CCP1CONbits.CCP1M = 0xF;                    // enable PWM on CCP1

        if(pwm_state == 0)                          // if new norm_rate set
        {
            norma_8b = num[norm_num];               // then borrowing DUTY_CYCLE from table
            CCPR1L = norma_8b;
            pwm_state = 1;                          // next DC from compare RESLT and table per_pwm_arr
        }
        else if(pwm_state == 1)
        {
            if(RESLT >= table[0])                   // measured flow rate smaller than lowest selected rate
            {
                pwm_state = 0;
                norma_8b = num[norm_num];
                CCPR1L = norma_8b;
            }
            if(f_measured == 1)
            {
                f_measured = 0;
                if(RESLT > table[norm_num])
                {
                    ki++;
                    if(ki == ps)
                    {
                        norma_8b = norma_8b - 1;
                        if(norma_8b <= 2)   //if(norma_8b <= 10)
                            norma_8b = 2;  //norma_8b = 10;
                        CCPR1L = norma_8b;
                        ki = 0;
                    }
                }
                else if(RESLT < table[norm_num] )
                {
                    ki++;
                    if(ki == ps)
                    {
                        norma_8b = norma_8b + 1;
                        if(norma_8b > 231)
                            norma_8b = 231;
                        CCPR1L = norma_8b;
                        ki = 0;
                    }
                }
            }
        }
    }
    else
    {
        disable_pump();
    }
}
#elif PWM_capacity == 10
void drive_pump( unsigned int *num,  unsigned long *table)
{
    const unsigned char ps = 1;//60;

    if(mode_AUTO || mode_MAN)
    {
        TRISCbits.RC2 = 0;                          // set PWM output
        CCP1CONbits.DC1B = 0b11;                    // 2 LSB of CCP1 reg = 3, so we have 8 bit DUTY_CYCLE resolution
        PR2 = 0xFF;                                 // max tmr2 period (485HZ @ 8MHz)
        T2CONbits.TMR2ON = 1;                       // start tmr2
        CCP1CONbits.CCP1M = 0xF;                    // enable PWM on CCP1

        if(pwm_state == 0)                          // if new norm_rate set
        {
            norma_10b = num[norm_num];
            CCP1CONbits.DC1B = norma_10b & 0b11;    // Set LSb[1:0] of PWM DUTY_CYCLE by masking 2 lower bits of table value
            CCPR1L = norma_10b>>2;                  // then set MSb[9:2] of DUTY_CYCLE from table
            pwm_state = 1;                          // next DC from compare RESLT and table per_pwm_arr
        }
        else if(pwm_state == 1)
        {
            if(RESLT >= table[0])
            {
                pwm_state = 0;
                norma_10b = num[norm_num];
                CCP1CONbits.DC1B = norma_10b & 0b11;
                CCPR1L = norma_10b>>2;
            }
            if(f_measured == 1)
            {
                  f_measured = 0;
                if(RESLT > table[norm_num])
                {
                    ki++;
                    if(ki == ps)
                    {
                        norma_10b--;
                        if(norma_10b <= 200)
                            norma_10b = 200;
                        CCP1CONbits.DC1B = norma_10b & 0b11;
                        CCPR1L = norma_10b>>2;
                        ki = 0;
                    }
                }
                else if(RESLT < table[norm_num] )   // measured period smaller => flow grater then desired
                {
                    ki++;
                    if(ki == ps)
                    {
                        norma_10b++;
                        if(norma_10b > 920)
                            norma_10b = 920;
                        CCP1CONbits.DC1B = norma_10b & 0b11;
                        CCPR1L = norma_10b>>2;
                        ki = 0;
                    }
                }
            }
        }
    }
    else
    {
        TRISCbits.RC2 = 1;          // set PWM as input to disable pin
        CCP1CONbits.CCP1M = 0x0;    // disable PWM module
        T2CONbits.TMR2ON = 0;
        pwm_state = 0;
    }
}
#endif

void ROM_WR(unsigned int adr, unsigned int data)
{
    unsigned char INTCON_SAVE;
    EEADR = adr;
    EEDATA = data;

    EECON1bits.EEPGD= 0;                 // 0 = Access data EEPROM memory
    EECON1bits.CFGS = 0;                  // 0 = Access Flash program or DATA EEPROM memory
    EECON1bits.WREN = 1;                 // enable writes to internal EEPROM

    INTCON_SAVE=INTCON;             // Save INTCON register contants
    INTCON=0;                        // Disable interrupts, Next two lines SHOULD run without interrupts

    EECON2=0x55;                     // Required sequence for write to internal EEPROM
    EECON2=0xaa;                     // Required sequence for write to internal EEPROM

    EECON1bits.WR=1;                 // begin write to internal EEPROM
    INTCON=INTCON_SAVE;              //Now we can safely enable interrupts if previously used
    Nop();
    while (PIR2bits.EEIF==0)            //Wait till write operation complete
    {
        Nop();
    }

    EECON1bits.WREN=0;                  // Disable writes to EEPROM on write complete (EEIF flag on set PIR2 )
    PIR2bits.EEIF=0;                 //Clear EEPROM write complete flag. (must be cleared in software. So we do it here)

}

unsigned char ROM_RD(unsigned char adr)
{
    EEADR=adr;
    EECON1bits.EEPGD= 0; // 0 = Access data EEPROM memory
    EECON1bits.CFGS = 0; // 0 = Access Flash program or DATA EEPROM memory
    EECON1bits.RD   = 1; // EEPROM Read
    return EEDATA;       // return data
}

void pwron_task(void)
{
    unsigned int level = PWROFF_time;
    
    if(PWR_ON)
        level = PWRON_time;
    else
        level = PWROFF_time;
    
    if(!fSET)
        pwron_cnt += 1;
    else
        pwron_cnt = 0;
    if(pwron_cnt == level)
    {
        PWR_ON = ~PWR_ON;
        pwron_cnt = 0;
    }
}

void disable_pump(void)
{
        TRISCbits.RC2 = 1;          // set PWM as input to disable pin
        CCP1CONbits.CCP1M = 0x0;    // disable PWM module
        T2CONbits.TMR2ON = 0;
        pwm_state = 0;
}

void get_settings(void)
{
    // verify ADC status (not used)
    ADC_ID = reset_ADC();
    if (ADC_ID == 0x5B | ADC_ID == 0x5A)
        ADC_err = 0;
    else
        ADC_err = 1;
    // retrieve set flow rate if any
    norm_num = ROM_RD(0x10);
    if(norm_num > 71)                                   // check for 1st ROM read
        norm_num = 0x14;
    // get calibration threshold
    unsigned long temp[4] = {0};
    int k = 0;
    for(k = 0; k < 4; k++)
        temp[k] = ROM_RD(ADC_THR_ADR + k);
    if(temp[0] == 0xFF & temp[1] == 0xFF & temp[2] == 0xFF & temp[3] == 0xFF)   // Calibration ROM wasn't initialized
        ADC_THR_v = 0x0081B320;                                                 // default threshold
    else
        ADC_THR_v = temp[0]<<24 | temp[1]<<16 | temp[2]<<8 | temp[3];           // previous calibration data
}

void set_PWM(void)
{
    if(((mass_locked && mode_AUTO) || mode_MAN)&&PWR_ON)
    {
        if(PID_cfg.PWM_rdy && PID_timer >= 32)
        {
        TRISCbits.RC2 = 0;                          // set PWM output
        CCP1CONbits.DC1B = 0b11;                    // 2 LSB of CCP1 reg = 3, so we have 8 bit DUTY_CYCLE resolution  
            PR2 = 0xFF;                                 // max tmr2 period (485HZ @ 8MHz)
        T2CONbits.TMR2ON = 1;                       // start tmr2
        CCP1CONbits.CCP1M = 0xF;                    // enable PWM on CCP1
        

            CCPR1L = PID_cfg.PWM;
            PID_timer = 0;
        }
        PID_cfg.PWM_rdy = 0;
    }
    else
        disable_pump();
}