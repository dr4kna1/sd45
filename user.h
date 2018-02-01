

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

unsigned int norm_num;