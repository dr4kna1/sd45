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

#define PCB_rev 0

#if PCB_rev == 0
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

#elif PCB_rev == 1
    // LED segments bus
#define ledA PORTDbits.RD5
#define ledB PORTDbits.RD0
#define ledC PORTDbits.RD2
#define ledD PORTDbits.RD6
#define ledE PORTDbits.RD7
#define ledG PORTDbits.RD3
#define ledF PORTDbits.RD1
#define ledDP PORTDbits.RD4
// LED anode
#define AN0 PORTAbits.RA7
#define AN1 PORTAbits.RA6
#define AN2 PORTCbits.RC6
#define AN3 PORTBbits.RB2
#define AN4 PORTBbits.RB1
#define AN5 PORTBbits.RB0
//panel buttons
#define SET PORTBbits.RB6
#define UP  PORTBbits.RB3
#define DOWN PORTBbits.RB4
#define MANUAL PORTBbits.RB7
#define AUTO PORTBbits.RB5
//panel leds
#define indUP PORTAbits.RA1
#define indDOWN PORTAbits.RA0
#define indSET PORTAbits.RA3
#define indAUTO PORTAbits.RA2
#define indMANUAL PORTEbits.RE1
#define indPUMP PORTEbits.RE0
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* MACROS_H */
