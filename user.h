
#define grade 3                     /* grade for array */
#define measure_num 1<<grade        /* number of consequential measurments */
#define ADC_threshold 0x81B320      /* threshold for mass bridge voltage */
#define PWRON_time    2700          /* time to hold SET button to enable main cycle */
#define PWROFF_time   54000
#define FLOW_RATE_ADR 0x10
#define ADC_THR_ADR   0x20
#define FLOW_ACC_ADR  0x30
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

void mcu_init(void);                 /* I/O and Peripheral Initialization */
void irq_tmr3(void);                /* timer 3 irq handler */
void irq_tmr1(void);
void irq_tmr0(void);
void irq_ccp2(void);
void irq2_ccp2(void);
void prcd_but(void);
void pwron_task(void);
void disable_pump(void);
void ROM_WR(unsigned int adr, unsigned int data);
void ROM_32WR(unsigned int adr, unsigned long data);
unsigned char ROM_RD(unsigned char adr);
int binarySearch( unsigned long *tab1, unsigned long key, int high, int low);
unsigned long sumarr(unsigned long arr[]);
void get_settings(void);
void set_PWM(void);
void meter_task(void);
void meter_reset(void);

unsigned int norm_num;

// buttons state temp vars
unsigned char fUP = 1;
unsigned char fDOWN = 1;
unsigned char fSET = 1;
unsigned char fMAN = 1;
unsigned char fAUTO = 1;

char mode_MAN;
char mode_AUTO;
char mode_SET;
volatile unsigned long PER0 = 0;            // buffer #1 for active measurment
unsigned long PER1[measure_num] = {0};
volatile unsigned long RESLT = 0;           // normal result of 8 measures
int mes_num    = 0;                         // number of measurments
int arr_num = 767;                          // array index for measured flow table
unsigned int prev_norm_num = 0;
unsigned long *arr_p;// = meash_arr;
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
long Set_Flow = 0;
unsigned int   time_cnt = 0;
unsigned long  flow_acc;
unsigned long  prev_flow_acc;
unsigned long  pulse_cnt;
float          current_expend;
long           current_expend_cast;
unsigned int   expend_cnt = 0;
unsigned char  expend_info = 0;
unsigned int   flwrst_cnt = 0;
unsigned char  flwrst_info = 0;