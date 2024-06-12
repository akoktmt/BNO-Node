/*
 * BNOStatemachine.c
 *
 *  Created on: Jan 28, 2024
 *      Author: win 10
 */
#include <stdio.h>
#include <stdbool.h>
#include "BNOStatemachine.h"
#include "bno055.h"
#include "bno_config.h"
#include <stdio.h>
#include "Calib.h"
#include <string.h>
#include "CAN_OSI.h"
#include <math.h>
#include "BNOStatemachine.h"
#include "can.h"
#include "usart.h"
extern uint8_t Data[8];
extern bno055_euler_t eul;
extern bno055_t bno;
extern error_bno err;
extern BNO_State currentState;
extern ErrCounter ErrCnt;
extern bno055_vec3_t acc;
extern uint8_t Yaw[4];
extern uint8_t AccX;
//extern uint8_t AccY;
uint8_t CheckFlag=0;
extern CANBufferHandleStruct ApplicationBuff;
extern CANConfigIDTxtypedef pIDtype;
void BNO_GetData() {
	 bno.euler(&bno, &eul);
	 bno.acc(&bno, &acc);
	 currentState=BNO_CHECK_SIGNAL;
}

void BNO_CheckSignal() {
//Euler Data
	if(eul.yaw < 360 && eul.yaw > -360){
	  if (eul.yaw > 180) {
		eul.yaw = -(360 - eul.yaw);
	}
	 AccX = compress_float_to_uint8(acc.x, -20.0f, 20.0f);
//	 AccY = compress_float_to_uint8(acc.y, -20.0f, 20.0f);
	 float2Bytes(Yaw,eul.yaw);
	 currentState=BNO_SEND_DATA;
	}
	else
	{
	ErrCnt.BNO_Error_DataCounter++;
	 if(ErrCnt.BNO_Error_DataCounter==5){
		currentState=BNO_RECOVERY;
	 }
	 else{
	 currentState=BNO_SEND_ERROR;
	 }
	}
}

void BNO_SendError() {
    // Code to handle errors
	if(CAN_Send_Application(&ApplicationBuff, &pIDtype, Data, sizeof(Data))==HAL_OK){
			currentState=BNO_GETDATA;
		}
		else
		{
			ErrCnt.CAN_Error_SenErrorCounter++;
			if(ErrCnt.CAN_Error_SenErrorCounter==3||ErrCnt.CAN_Error_SenErrorCounter==6){
			currentState=BNO_RECOVERY;
			}
			else{
			currentState=BNO_GETDATA;
			}
		}
}

void BNO_SendData() {
	Data[0]=Yaw[0];
	Data[1]=Yaw[1];
	Data[2]=Yaw[2];
	Data[3]=Yaw[3];
	Data[4]=AccX;
	if(CAN_Send_Application(&ApplicationBuff, &pIDtype, Data, sizeof(Data))==HAL_OK){
		currentState=BNO_GETDATA;
	}
	else
	{
		ErrCnt.CAN_Error_SenDataCounter++;
		if(ErrCnt.CAN_Error_SenDataCounter==3||ErrCnt.CAN_Error_SenDataCounter==6){
			currentState=BNO_RECOVERY;
		}
		else{
		currentState=BNO_GETDATA;
		}
	}
}

void BNO_Recovery() {
    // Code for recovery
	if(ErrCnt.BNO_Error_DataCounter==5){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_Delay(2000);
		NVIC_SystemReset();
	}
	else if(ErrCnt.CAN_Error_SenDataCounter==3){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_Delay(2000);
		HAL_CAN_ResetError(&hcan);
		currentState=BNO_GETDATA;
	}
	else if(ErrCnt.CAN_Error_SenErrorCounter==3){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_Delay(2000);
		HAL_CAN_ResetError(&hcan);
		currentState=BNO_GETDATA;
	}
	else if(ErrCnt.BNO_Error_DataCounter==5||ErrCnt.CAN_Error_SenDataCounter==6||ErrCnt.CAN_Error_SenErrorCounter==6){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET);
		HAL_Delay(2000);
		NVIC_SystemReset();
	}
}
