
#define grade 3                     /* grade for array */
#define measure_num 1<<grade        /* number of consequential measurments */
#define ADC_threshold 0x81B320      /* threshold for mass bridge voltage */
#define PWRON_time    2700          /* time to hold SET button to enable main cycle */
#define PWROFF_time   54000
#define ADC_THR_ADR   0x20
#define ADC_CONV_TR   63            /* number of sequential ADC ocnversion during calibration */
#define PID_period    16            /* delayin applying PID regulation [TMR3 periods] */
#define decimate_msre 1             /* get every second impulse */
#define OSC_prescale  1             /* internalosc prescaler value */
#define tmr3_prescale 1             /* timer3 prescaler */

#if OSC_prescale == 1
    #define _PRESC  0b111;            // 8 MHz INTOSC
    #define Fsys    2000000           // sys clock/4
#elif OSC_prescale == 2
    #define _PRESC  0b110;            // 4 MHz INTOSC
    #define Fsys    1000000           // sys clock/4
#elif OSC_prescale == 4
    #define _PRESC   0b101;           // 2 MHz INTOSC
    #define Fsys    500000           // sys clock/4
#elif OSC_prescale == 8
    #define _PRESC   0b100;           // 1 MHz INTOSC
    #define Fsys     250000           // sys clock/4
#else
    #define _PRESC  0b111;            // 8 MHz default
    #define Fsys    2000000           // sys clock/4
#endif

#if tmr3_prescale == 1
    #define _tmr3_presc 0              //  1:1
    #define tmr3_freq   Fsys/1
#elif tmr3_prescale == 2
    #define _tmr3_presc 1              //  1:2
    #define tmr3_freq   Fsys/2
#elif tmr3_prescale == 4
    #define _tmr3_presc 2              //  1:4
    #define tmr3_freq   Fsys/4
#elif tmr3_prescale == 8
    #define _tmr3_presc 3              //  1:8
    #define tmr3_freq   Fsys/8
#else
    #define _tmr3_presc 0              //  1:1
#endif
void InitApp(void);                 /* I/O and Peripheral Initialization */
void irq_tmr3(void);                /* timer 3 irq handler */
void irq_tmr1(void);
void irq_tmr0(void);
void irq_ccp2(void);
void irq2_ccp2(void);
void irq_ioch(void);
void lit_led(unsigned int str1,unsigned int str2,unsigned int adc_cnt);
void prcd_led1(void);
void prcd_led2(void);
void prcd_led3(void);
void prcd_led4(void);
void prcd_led5(void);
void prcd_led6(void);
void prcd_but(void);
void measure(void);
void measure0(void);
void pwron_task(void);
void disable_pump(void);
void ROM_WR(unsigned int adr, unsigned int data);
unsigned char ROM_RD(unsigned char adr);
void drive_pump( unsigned int *num,  unsigned long *table);
int decode_str(int str);
int binarySearch( unsigned long *tab1, unsigned long key, int high, int low);
unsigned long sumarr(unsigned long arr[]);
void InterruptHandlerHigh (void);
void get_settings(void);
void set_PWM(void);

unsigned int norm_num;
volatile unsigned long PER0 = 0;                     // buffer #1 for active measurment
unsigned long PER1[measure_num] = {0};
unsigned long PER2 = 0;                     // sum of 8 measurments
volatile unsigned long RESLT = 0;                    // normal result of 8 measures
char ACTV = 0b0;                            // controller state
int mes_num    = 0;                         // number of measurments
int arr_num = 767;                          // array index for measured flow table
unsigned int prev_norm_num = 0;
unsigned long *arr_p;// = meash_arr;
unsigned char led_state = 0;
unsigned char ADCStatus = 0;
unsigned long ADC_data = 0;
unsigned char ADC_ID = 0;
unsigned char ADC_err = 0;
unsigned char service_info = 0;
unsigned int  service_cnt = 0;
unsigned int  auto_cnt = 0;
bit calibration_info = 0b0;
bit cal_info_prev = 0b0;
bit calibration_act  = 0b0;
unsigned long cal_acc = 0;                  // calibration data accumulator
unsigned int adc_conv_cnt  = 0;
bit PWR_ON = 0;
unsigned int pwron_cnt = 0;

unsigned char  AUTO_rlsd = 0b1;
unsigned char  AUTO_btn_hold = 0b0;
unsigned char  AUTO_forbid = 0b0;
unsigned char  PID_timer = 32;

unsigned char  MAN_rlsd = 0b1;
unsigned char  MAN_btn_hold = 0b0;
unsigned char  MAN_forbid = 0b0;
unsigned int   man_cnt = 0;
bit            manpwm_info = 0b0;
bit            manpwm_info_prev = 0b0;
volatile unsigned char tmr_counting = 0;
unsigned int tmr_overflow_evn = 0;
unsigned int active_evn       = 0;
