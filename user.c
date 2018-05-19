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
#include "display_7seg.h"

unsigned int i=0;
unsigned int j = 1;
unsigned char ki = 0;
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

void mcu_init(void)
{
    OSCCONbits.IRCF = _PRESC;            // 8 MHz INTOSC
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

    INTCONbits.TMR0IE = 0;      // irq TMR0
    PIE1bits.TMR1IE = 0;
    PIE2bits.TMR3IE = 1;            // irq enabled

    PIE2bits.CCP2IE = 1;            // enable irq CCP2

    /* TIMER0 */
    T0CONbits.T08BIT = 0;       // 0 - 16 bit mode; 1 - 8 bit
    T0CONbits.T0CS	= 0;    //1 = Transition on T0CKI pin; 0 = Internal instruction cycle clock (CLKO)
    // prescaler on, tmr0 off, prescale ratio 1:256
    T0CONbits.PSA    = 0;
    T0CONbits.T0PS2  = 1;
    T0CONbits.T0PS1  = 1;
    T0CONbits.T0PS0  = 1;
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
    T3CONbits.T3CKPS = _tmr3_presc;             // prescaler 1:1
    T3CONbits.T3CCP2 = 0;           // TmR3 clk src for CCP2
    T3CONbits.T3CCP1 = 1;           // TmR1 clk src for CCP1
    T3CONbits.TMR3ON = 1;

    /*CCP1*/
    CCPR1L  = 0x0F;
    CCP1CON = 0;      // Enable PWM 1

   /*CCP2*/
    CCP2CONbits.CCP2M = 0x4;       // CCP2 Capture rising edge

}

// tmr0 operates on Fsys/(4*256), so 48765 ticks equal to 6 seconds
void irq_tmr0(void)
{
    INTCONbits.T0IF = 0;
    TMR0H = 0x41;           // reinit tmr 0
    TMR0L = 0x82;
    time_cnt++;
}

/* TMR3 full cycle triggered */
void irq_tmr3(void)
{
    PIR2bits.TMR3IF = 0;
        PID_timer++;
        if(PID_timer > PID_period)
            PID_timer = PID_period;
        PER0 = PER0 + 0xFFFF;
}

/* Capture event on input and load TMR3 value */
void irq_ccp2(void)
{
    //clear CCP2 irq flag
    PIR2bits.CCP2IF = 0;
    //disable irqs
    PIE2bits.TMR3IE = 0;
    PIE2bits.CCP2IE = 0;
    flow_acc++;
#if decimate_msre == 1
    if(tmr_counting == 0)
    {
        T3CONbits.TMR3ON = 1;
        PER0 = 0;
//        if(mes_num > measure_num)
            tmr_counting = 1;
        mes_num++;
    }
    else
    {
        T3CONbits.TMR3ON = 0;
        active_evn++;
        unsigned temp = ((CCPR2H<<8) + CCPR2L);
        PER0 = PER0 + temp;
        if(PER0 > 0x4AC000 | PER0 < 172500)
        {
            RESLT = RESLT;
            tmr_overflow_evn++;
            PER0 = 0;
        }
        else
            RESLT = PER0;
        TMR3H = TMR3L = 0;
        tmr_counting = 0;
        mes_num = 0;
    }
#elif decimate_msre == 0
    T3CONbits.TMR3ON = 1;
    PER0 |= ((CCPR2H<<8) | CCPR2L);
    if(PER0 > 0x4AC000)
        RESLT = 2840238;
    else
        RESLT = PER0;
    TMR3H = TMR3L = 0;
    PER0 = 0;
#endif
    j = !j;
    indPUMP =  j;
    PIE2bits.TMR3IE = 1;
    PIE2bits.CCP2IE = 1;
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
/*-----------------------------------------------------------------------*/
        if (!fUP)
            expend_cnt += 1;
        if (expend_cnt == 1500)
		{
			expend_info = !expend_info;
			expend_cnt=0;
		}
/*-----------------------------------------------------------------------*/
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
            if(manpwm_info)
            {   
                if(PID_cfg.PWM < 255)
                    PID_cfg.PWM++;
                else
                    PID_cfg.PWM = 255;
                PID_cfg.PWM_rdy = 1;
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
            if(manpwm_info)
            {
                PID_cfg.PWM--;
                PID_cfg.PWM_rdy = 1;
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

void ROM_32WR(unsigned int adr, unsigned long data)
{
    ROM_WR(adr,     (0xFF000000 & data) >> 24);
    ROM_WR(adr + 1, (0x00FF0000 & data) >> 16);
    ROM_WR(adr + 2, (0x0000FF00 & data) >> 8);
    ROM_WR(adr + 3, (0x000000FF & data));
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
    norm_num = ROM_RD(FLOW_RATE_ADR);
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
    for(k = 0; k < 4; k++)                                                      // read out saved expended flow mass
        temp[k] = ROM_RD(FLOW_ACC_ADR + k);
    flow_acc = temp[0]<<24 | temp[1]<<16 | temp[2]<<8 | temp[3];
    prev_flow_acc = flow_acc;
}

void set_PWM(void)
{
    if(((mass_locked && mode_AUTO) || mode_MAN || manpwm_info)&&PWR_ON)
    {
        if(PID_cfg.PWM_rdy)// && PID_timer >= PID_period)
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

// Looks after tmr0 and accumulates expended flow
void meter_task(void)
{
    if(!mode_AUTO && !mode_MAN)  // pump not active
    {
        if (prev_flow_acc != flow_acc)
        {
            ROM_32WR(FLOW_ACC_ADR,flow_acc);
            prev_flow_acc = flow_acc;
            if (expend_info)
            {
                current_expend = flow_acc * 11.35;
                if (current_expend > 10000 && current_expend < 1000000)
                    current_expend_cast = (long)(current_expend/10000);
                else if (current_expend >= 1000000 && current_expend < 1000000000)
                    current_expend_cast = (long)(current_expend/1000000);
                else if (current_expend >= 1000000000 && current_expend < 9990000000)
                    current_expend_cast = (long)(current_expend/1000000000);
            }
        }
    }
}
