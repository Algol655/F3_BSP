/*
 * TestEnv.h
 *
 *  Created on: 17 nov 2020
 *  Author: Tommaso Sabatini
 */

#ifndef APPLICATION_TESTENV_H_
#define APPLICATION_TESTENV_H_

#define INPUT_TEST		(1)
#define OUTPUT_TEST		(1)
#define ANALOG_TEST		(1)
#define CAN1_TEST		(1)
#define CAN2_TEST		(0)
#define FW_UPDATE		(1)
#define UTILITIES		(1)
#define SET_DATE_TIME	(1)
#define RTC_CALIB		(0)
#define UWB_MODE		(0)
#define PWR_NODE_MODE	(0)
#define RTLS_FW			(0)
#define RANGING_FW		(0)
#define WRITE_FLASH		(1)

extern const uint8_t CDC_delay;
extern bool conversion_ended;
extern void read_analogs(void);
extern void usb_run(void);
extern void send_usbmessage(uint8_t *string, int len);

#ifndef SWITCH_CASE_INIT
#define SWITCH_CASE_INIT
    #define SWITCH(X)	for (char* __switch_p__ = X, __switch_next__=1 ; __switch_p__ ; __switch_p__=0, __switch_next__=1) { {
    #define CASE(X)			} if (!__switch_next__ || !(__switch_next__ = strcmp(__switch_p__, X))) {
    #define DEFAULT		  	} {
    #define END			}}
#endif

void top_menu(void);
void L10_menu(void);
void L11_menu(void);
void L12_menu(void);
void L20_menu(void);
void L30_menu(void);
void L40_menu(void);
void L50_menu(void);
void L50_menu(void);
void L60_menu(void);
void L70_menu(void);

#endif /* APPLICATION_TESTENV_H_ */
