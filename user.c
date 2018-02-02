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
//extern int norm_num;

unsigned int string = 0x0;
unsigned int dig_i = 0;
unsigned char flash = 0;

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
extern const unsigned int pwm_arr[71];
extern const  unsigned int rate_arr[768];
extern const  unsigned long meash_arr[768];
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
    PORTA = 0x20;
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
    PIE1bits.TMR1IE = 1;
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
    T1CONbits.TMR1ON = 1;       // on

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
   led_state++;

   fAUTO = AUTO;
   fDOWN = DOWN;
   fMAN = MANUAL;
   fUP = UP;
   fSET = SET;


}

void measure(void)
{
    if(mes_num==9)
        {
        int k = 0;
            PIE2bits.TMR3IE = 0;
            PIE2bits.CCP2IE = 0;
            f_measured = 1;
            ACTV = 0;
//            PER2 = PER1[0]+PER1[1]+PER1[2]+PER1[3]+PER1[4]+PER1[5]+PER1[6]+PER1[7];
//            k=0;
//            while(k<8)
//            {
//                PER2 = PER2 + PER1[k];
//                k++;
//            }

            PER2 = sumarr(PER1);
            PER2 = PER2>>grade;             // divide by grade
//            PER2 = PER1[4];
            RESLT = PER2;
            PER2 = 0;                   // reset buffer

            k = 0;
             while(k<8)                 // reset buffer per1
            {
                PER1[k] = 0;
                k++;
            }

            mes_num = 0;                // end measurment session


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
void lit_led(unsigned int str1,unsigned int str2)
{
    int nstr;
    if(led_state < 7)
        {
            PORTD = 0xFF;
            switch (led_state)
            {
                case 1  :                                                                   // LED1 on, other off
                   prcd_led1();
                   nstr = (str1&0x0F00)>>8;

                   PORTD = decode_str(nstr); break;                                          // lit '1'
                case 2  :
                   prcd_led2(); PORTD = decode_str((str1&0x00F0)>>4); break;
                case 3  :
                   prcd_led3(); PORTD = decode_str(str1&0x000F); break;
                case 4  :
                   prcd_led4(); PORTD = decode_str((str2&0x0F00)>>8); break;
                case 5  :
                   prcd_led5(); PORTD = decode_str((str2&0x00F0)>>4); break;
                case 6  :
                   prcd_led6(); PORTD = decode_str(str2&0x000F); break;
                default: break;
            }
        }
        else
            led_state = 0;
}

/* Decode hex int to 7-seg digit*/
int decode_str(int str)
{
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
    indUP = !fUP;
    if(fUP > prev_UP)
    {
        if(mode_SET)
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
        if(mode_SET)
        {
            norm_num--;
            pwm_state = 0;
            if(norm_num <= 0)
                norm_num = 0;
        }
    }
    prev_DOWN = fDOWN;

/*-----------------------------------------------------------------------*/
    if(fSET > prev_SET)
    {
        mode_SET = !mode_SET;
        prev_SET = 1;
    }
    prev_SET = fSET;
/*-----------------------------------------------------------------------*/
    if(fAUTO > prev_AUTO)
    {
        mode_AUTO = !mode_AUTO;
        mode_MAN = 0;
    }
    prev_AUTO = fAUTO;
/*-----------------------------------------------------------------------*/
    if(fMAN > prev_MAN)
    {
        mode_MAN = !mode_MAN;
        mode_AUTO = 0;
    }
    prev_MAN = fMAN;


}

void drive_pump( unsigned int *num,  unsigned long *table)
{

    const unsigned char ps = 1;//60;

    if(mode_AUTO || mode_MAN)
    {
        CCP1CONbits.DC1B = 0b11;                    // 2 LSB of CCP1 reg = 3, so we have 8 bit DUTY_CYCLE resolution
        PR2 = 0xFF;                                 // max tmr2 period (485HZ @ 8MHz)
        T2CONbits.TMR2ON = 1;                       // start tmr2
        CCP1CONbits.CCP1M = 0xF;                    // enable PWM on CCP1

        if(pwm_state == 0)                          // if new norm_rate set
        {
            CCPR1L = num[norm_num];                     // then borrowing DUTY_CYCLE from table
            pwm_state = 1;                          // next DC from compare RESLT and table per_pwm_arr
        }
        else if(pwm_state == 1)
        {
            if(RESLT >= table[0])
            {
                pwm_state = 0;
                CCPR1L = num[norm_num];
            }
            if(f_measured == 1)
            {
                f_measured = 0;
                if(RESLT > table[norm_num])
                {
                    ki++;
                    if(ki == ps)
                    {
                        CCPR1L--;
                        if(CCPR1L <= 50)
                            CCPR1L = 50;
                        ki = 0;
                    }
                }
                else if(RESLT < table[norm_num] )
                {
                    ki++;
                    if(ki == ps)
                    {
                        CCPR1L++;
                        if(CCPR1L > 231)
                            CCPR1L = 231;
                        ki = 0;
                    }
                }
            }
        }

    }
    else
    {
        CCP1CONbits.CCP1M = 0x0;
        T2CONbits.TMR2ON = 0;
        pwm_state = 0;
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

unsigned char ROM_RD(unsigned char adr)
{
    EEADR=adr;
    EECON1bits.EEPGD= 0; // 0 = Access data EEPROM memory
    EECON1bits.CFGS = 0; // 0 = Access Flash program or DATA EEPROM memory
    EECON1bits.RD   = 1; // EEPROM Read
    return EEDATA;       // return data
}
