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
#include "SPI_ex.h"

extern unsigned char ADCStatus;
extern unsigned int ADCData;

void initSPI(void)
{
	//************* SSPSTAT *****************
	
	SSPSTATbits.SMP = 0;
	SSPSTATbits.CKE = 0;	//Transmit occurs on transition from Idle to active clock state
	
	//************* SSPCON1 *****************
	
	SSPCON1bits.CKP = 0;		//Idle state for clock is a low level
	SSPCON1bits.SSPM0 = 0;
	SSPCON1bits.SSPM1 = 0;
	SSPCON1bits.SSPM2 = 1;
	SSPCON1bits.SSPM3 = 0;		//SPI Master mode, clock = FOSC/64
	SSPCON1bits.SSPEN = 1;		//Enables serial port and configures SCK, SDO, SDI and SS as serial port pins
}

unsigned char writeSPI(unsigned char data)
{
	SSPBUF = data;
	while(!SSPSTATbits.BF);
	return SSPBUF;
}

unsigned char readSPI(void)
{
	unsigned char data = 0;
	
	data = writeSPI(0);
	
	return data;
}

unsigned int readDataReg(void)
{
	unsigned int dataValue = 0;
	
	CS = 0;
	writeSPI(0x58);		// ?????? ? COMMUNICATION REGISTER ?? ?????? Data Register
	dataValue = readSPI();// ?????? ???????? ????? Data Register
	dataValue <<= 8;
	dataValue |= readSPI();// ?????? ???????? ????? Data Register
	CS = 1;
	
	return dataValue;
}	

unsigned char readStatReg(void)
{
	unsigned char data = 0;
	
	CS = 0;
	writeSPI(0x40);		//  ?????? ? COMMUNICATION REGISTER ?? ?????? Status Register
	data = readSPI();	// ?????? Status Register
	CS = 1;
	
	return data;
}	

void prc_SPI(void)
{
    CS = 1;
    ADCStatus = readStatReg();          // ?????? ??????? ???
    if(!(ADCStatus & RDY))		// ???? ????????? ?????????????? ?????
        ADCData = readDataReg();
}
