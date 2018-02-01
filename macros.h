/*
 * File:   macros.h
 * Author: Руслан
 *
 * Created on ??????, 2, ???????? 2013, 5.10
 */

#ifndef MACROS_H
#define	MACROS_H

#ifdef	__cplusplus
extern "C" {
#endif

// LED segments bus
#define ledA PORTDbits.RD0
#define ledB PORTDbits.RD1
#define ledC PORTDbits.RD2
#define ledD PORTDbits.RD3
#define ledE PORTDbits.RD4
#define ledG PORTDbits.RD5
#define ledF PORTDbits.RD6
#define ledDP PORTDbits.RD7
// LED anode
#define AN0 PORTAbits.RA7
#define AN1 PORTAbits.RA6
#define AN2 PORTCbits.RC6
#define AN3 PORTBbits.RB2
#define AN4 PORTBbits.RB1
#define AN5 PORTBbits.RB0
//panel buttons
#define SET PORTBbits.RB3
#define UP  PORTBbits.RB4
#define DOWN PORTBbits.RB5
#define MANUAL PORTBbits.RB6
#define AUTO PORTBbits.RB7
//panel leds
#define indUP PORTAbits.RA0
#define indDOWN PORTAbits.RA1
#define indSET PORTAbits.RA2
#define indAUTO PORTAbits.RA3
#define indMANUAL PORTEbits.RE0
#define indPUMP PORTEbits.RE1



#ifdef	__cplusplus
}
#endif

#endif	/* MACROS_H */
