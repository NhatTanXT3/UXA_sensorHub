/*
 * ZMP.h
 *
 *  Created on: Jun 22, 2017
 *      Author: Nhat Tan
 */

#ifndef ZMP_H_
#define ZMP_H_
#include "mySerial.h"

#define UART_ZMP_LEFT_ 	UART6_BASE
#define UART_ZMP_RIGHT_ 	UART1_BASE
//============== id define=========


#define MCU2ZMP_TERMINATOR_ 	'\n'
#define MCU2ZMP_HEADER_		0xFF
#define SEND_RAW_DATA_           'a'
#define SEND_FILTERED_DATA_      'b'
#define SEND_PROCESSED_DATA_     'c'
#define SENDING_KEY_LEFT_ 0xCC
#define SENDING_KEY_RIGHT_ 0xAA
typedef struct {
	unsigned int rawForceSensor[4];
	unsigned int filterForceSensor[4];
	unsigned int posX;
	unsigned int posY;
	unsigned int amp;
} MyZMP;

extern MySerial serialZMPLeft;
extern MySerial serialZMPRight;
extern MyZMP zmpLeft;
extern MyZMP zmpRight;

extern void getRawZMPData(uint32_t ui32Base);
extern void getRawZMPLeft();
extern void ZMP_send_bytes(uint32_t ui32Base_, char *data, unsigned char size);
extern void UART7_Init();
extern void UART1_Init();
extern void UART6_Init();


#endif /* ZMP_H_ */
