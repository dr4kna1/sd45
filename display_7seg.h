/* 
 * File:   display_7seg.h
 * Author: r.d.
 *
 * Created on 13 may 2018, 17:28
 */

#ifndef DISPLAY_7SEG_H
#define	DISPLAY_7SEG_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* DISPLAY_7SEG_H */

void lit_led(unsigned int str1,unsigned int str2);
void prcd_led1(void);
void prcd_led2(void);
void prcd_led3(void);
void prcd_led4(void);
void prcd_led5(void);
void prcd_led6(void);
int decode_str(int str);
void display_task(void);

unsigned char led_state = 0;

