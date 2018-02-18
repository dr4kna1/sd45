
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

void initSPI(void);
unsigned char writeSPI(unsigned char data);
unsigned char readSPI(void);
unsigned int readDataReg(void);
unsigned char readStatReg(void);
void prc_SPI(void);
unsigned char readSPI_adr(unsigned char adr);

//=====================================================================

#endif	/* SPI_EX_H */
