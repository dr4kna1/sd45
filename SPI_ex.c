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

void initSPI(void)
{
	//************* SSPSTAT *****************
	
	SSPSTATbits.SMP = 0;
	SSPSTATbits.CKE = 0;	//Transmit occurs on transition from Idle to active clock state
	
	//************* SSPCON1 *****************
	
	SSPCON1bits.CKP = 1;		//Idle state for clock is a high level
	SSPCON1bits.SSPM0 = 0;
	SSPCON1bits.SSPM1 = 1;
	SSPCON1bits.SSPM2 = 0;
	SSPCON1bits.SSPM3 = 0;		//SPI Master mode, clock = FOSC/4
	SSPCON1bits.SSPEN = 1;		//Enables serial port and configures SCK, SDO, SDI and SS as serial port pins
    CS = 0;                     //Active bus
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

unsigned long readDataReg(void)
{
	unsigned long dataValue = 0;
    
    if(ADC_wait == 0b0)
    {
        writeSPI(0x58);         // COM reg addres DATA reg
        ADC_wait = 0b1;
    }
    if (PORTCbits.SDI == 0) // nRDY as SDI signals that conversion completed
    {   
        ADC_wait = 0b0;
        dataValue = readSPI();  // MSB Data Register
        dataValue <<= 8;
        dataValue |= readSPI(); // mid Data Register	
        dataValue <<= 8;
        dataValue |= readSPI(); // LSB 
    }
	return dataValue;
}	

unsigned char readStatReg(void)
{
	unsigned char data = 0;
	
	writeSPI(0x40);		//  ?????? ? COMMUNICATION REGISTER ?? ?????? Status Register
	data = readSPI();	// ?????? Status Register
	
	return data;
}	

/* Read data register after ADC conversion*/
void ADC_task(unsigned long *ADCData)
{
    unsigned long temp = 0;
    
    temp = readDataReg();
    if(temp)
        * ADCData = temp;
}

unsigned char readSPI_adr(unsigned char adr)
{
    unsigned char data = 0;
    unsigned char rs = 0;
    // ADC regs address range is 3 bits, adr position [5:3]
    rs = (1<<6) | (adr & 0x7)<<3;     // put 1 do indicate read from adr                   
    
//    CS = 0;
    writeSPI(rs);                    // write to COMREG
    data = readSPI();
//    CS = 1;
    return data;
}

/* Reset ADC logic and read ID */
unsigned char reset_ADC(void)
{
    unsigned char ID = 0;
    writeSPI(0xFF);
    writeSPI(0xFF);
    writeSPI(0xFF);
    writeSPI(0xFF);
    writeSPI(0xFF);
    ID = readSPI_adr(ADC_ID_REG);
    return ID;
}
