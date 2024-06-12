/*
 * BNOStatemachine.h
 *
 *  Created on: Jan 28, 2024
 *      Author: win 10
 */

#ifndef BNOSTATEMACHINE_H_
#define BNOSTATEMACHINE_H_
#include <stdio.h>
#include <stdbool.h>
#include "main.h"
typedef enum {
	BNO_INIT,
    BNO_READY,
    BNO_GETDATA,
    BNO_CHECK_SIGNAL,
    BNO_SEND_ERROR,
    BNO_SEND_DATA,
    BNO_RECOVERY, // State to exit the state machine
} BNO_State;

typedef struct {
	uint8_t CAN_Error_SenDataCounter;
	uint8_t CAN_Error_SenErrorCounter;
	uint8_t BNO_Error_DataCounter;
}ErrCounter;
void BNO_GetData();
void BNO_CheckSignal();
void BNO_SendError();
void BNO_SendData();
void BNO_Recovery();

#endif /* BNOSTATEMACHINE_H_ */
