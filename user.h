
#define grade 3                     /* grade for array */
#define measure_num 1<<grade        /* number of consequential measurments */

void InitApp(void);                 /* I/O and Peripheral Initialization */
void irq_tmr3(void);                /* timer 3 irq handler */
void irq_tmr1(void);
void irq_tmr0(void);
void irq_ccp2(void);
void irq2_ccp2(void);
void irq_ioch(void);
void lit_led(unsigned int str1,unsigned int str2);
void prcd_led1(void);
void prcd_led2(void);
void prcd_led3(void);
void prcd_led4(void);
void prcd_led5(void);
void prcd_led6(void);
void prcd_but(void);
void measure(void);
void measure0(void);
void ROM_WR(unsigned int adr, unsigned int data);
unsigned char ROM_RD(unsigned char adr);
void drive_pump( unsigned int *num,  unsigned long *table);
int decode_str(int str);
int binarySearch( unsigned long *tab1, unsigned long key, int high, int low);
unsigned long sumarr(unsigned long arr[]);
void InterruptHandlerHigh (void);
char btn_debounce(unsigned char button, unsigned char cnt);

unsigned int norm_num;
unsigned long PER0 = 0;                     // buffer #1 for active measurment
unsigned long PER1[measure_num] = {0};
unsigned long PER2 = 0;                     // sum of 8 measurments
unsigned long RESLT = 0;                    // normal result of 8 measures
char ACTV = 0b0;                            // controller state
int mes_num    = 0;                         // number of measurments
int arr_num = 767;                          // array index for measured flow table
unsigned int prev_norm_num = 0;
unsigned long *arr_p;// = meash_arr;
unsigned char led_state = 0;
unsigned char ADCStatus = 0;
unsigned int ADCData = 0;
