
#ifndef SPI_EX_H
#define	SPI_EX_H

//#include <pic18.h>

//*********************************************************************

//#define CS LATAbits.LATA5
#define CS PORTAbits.RA5
#define RDY 0b10000000
#define ERR 0b01000000

//*********************************************************************

//======================= прототипы функций ===========================

void spi_init(void);
unsigned char writeSPI(unsigned char data);
unsigned char readSPI(void);
unsigned long  readDataReg(void);
unsigned char readStatReg(void);
void ADC_task(unsigned long *ADCData);
unsigned char readSPI_adr(unsigned char adr);
unsigned char reset_ADC(void);

bit ADC_wait = 0;
bit mass_locked = 0;
unsigned char mass_lock_cnt = 0;
unsigned long ADC_THR_v = 0x0081B320;

//=====================================================================
// ADC registers map
#define ADC_COM_REG  0x0
#define ADC_STS_REG  0x0
#define ADC_MOD_REG  0x1
#define ADC_CFG_REG  0x2
#define ADC_DAT_REG  0x3
#define ADC_ID_REG   0x4
#define ADC_OFS_REG  0x6
#define ADC_FSC_REG  0x7


#endif	/* SPI_EX_H */
