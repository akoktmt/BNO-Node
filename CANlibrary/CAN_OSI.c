#include <CAN_OSI.h>
#include <string.h>
#include "CRC.h"
#include "CAN_Flag.h"
#include <stdlib.h>
extern CAN_HandleTypeDef hcan;
void CAN_Receive_Error_Handle(FlagRecNotification *FlagNoti,
		FlagFrameHandle *FlagHandle) {
	if (*FlagNoti == REC_FRAMEDATA_ERROR) {
		for (uint8_t FrameType = 0; FrameType < FlagHandle->NumberOfFrame;
				FrameType++) {
			if (FlagHandle->FlagID[FlagHandle->ID].FrameError[FrameType] == 1) {
				CAN_Send_Response(FlagHandle->ID, FRAME_ERROR, FrameType);
			}
		}
	}
	if (*FlagNoti == REC_PACKET_ERROR) {
		CAN_Send_Response(FlagHandle->ID, PACKET_ERROR, 0x55);

	}
}
uint16_t CAN_Send_Response(uint8_t ID, uint8_t Opcode, uint8_t FrameType) {
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t Txmailbox;
	uint8_t OpcodeData[8] = { Opcode, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55 };
	uint8_t StID = 0x00;
	StID |= ID;
	StID = (StID << 3) | FrameType;
	CAN_TXHeaderConfig(&TxHeader, StID);
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, OpcodeData, &Txmailbox)
			!= HAL_OK) {
		Error_Handler();
	}
	while (HAL_CAN_IsTxMessagePending(&hcan, Txmailbox))
		;
	return HAL_OK;
}

void CAN_Receive_Response(CAN_RxHeaderTypeDef *RxHeader, uint8_t *Data) {
	uint8_t GetLevel=HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
	if(GetLevel>0)
	{
		if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, RxHeader, Data)!=HAL_OK)
		{
			Error_Handler();
		}
	}
}
void CAN_Send_Error_Handle(CANBufferHandleStruct *Buffer,
		CANConfigIDTxtypedef *pIDType) {
	uint8_t Data[8] = { 0 };
	CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	uint32_t Txmailbox;
	CAN_Receive_Response(&RxHeader, Data);
	uint8_t StId = 0;
	StId = (StId << 3) | Buffer->FrameType_Index;
	StId = (Buffer->SenderID << 3) | Data[1];
	CAN_TXHeaderConfig(&TxHeader, StId);
	if (Data[0] == FRAME_ERROR) {
		if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, Buffer->Buffer[Data[1]],
				&Txmailbox) != HAL_OK) {
			Error_Handler();
		}
		while (HAL_CAN_IsTxMessagePending(&hcan, Txmailbox))
			;
	}
	if (Data[0] == PACKET_ERROR) {
		CAN_Send_Physical_Send(Buffer, Data, pIDType);
	}
}
void CAN_ProcessRxBuffer(FlagFrameHandle *FlagHandle, uint8_t ID,
		CANBufferHandleStruct *RxBuffer, uint8_t *DataPhysical,
		FlagRecNotification *FlagRecHandle) {
	uint8_t FrameType = 0;
	RxBuffer->NodeHandle[ID].NodeIndex++;
	for (; FrameType < RxBuffer->NodeHandle[ID].NumberOfFrame; FrameType++) {
		CAN_ProcessFrame(FlagHandle, ID, RxBuffer, FrameType, DataPhysical);
	}
	if (RxBuffer->NodeHandle[ID].NodeIndex
			== RxBuffer->NodeHandle[ID].NumberOfFrame) {
		if (FlagHandle->FlagID[ID].SumOfFlag
				== RxBuffer->NodeHandle[ID].NumberOfFrame) {
			*FlagRecHandle = REC_FRAMEDATA_SUCCESS;
			RxBuffer->NodeHandle[ID].NodeIndex = 0;
			RxBuffer->NodeHandle[ID].DuplicateFrame = 0;
			FlagHandle->FlagID[ID].SumOfFlag = 0;
			for (FrameType = 0;
					FrameType < RxBuffer->NodeHandle[ID].NumberOfFrame;
					FrameType++) {
				FlagHandle->FlagID[ID].FlagFrameFull[FrameType] = 0;
			}
		} else {
			*FlagRecHandle = REC_FRAMEDATA_ERROR;
			for (FrameType = 0;
					FrameType <= RxBuffer->NodeHandle[ID].NumberOfFrame;
					FrameType++) {
				if (FlagHandle->FlagID[ID].FlagFrameFull[FrameType] == 0) {
					FlagHandle->FlagID[ID].FrameError[FrameType] = 1;
					FlagHandle->ID = ID;
					FlagHandle->NumberOfFrame =
							RxBuffer->NodeHandle[ID].NumberOfFrame;
					CAN_Receive_Error_Handle(FlagRecHandle, FlagHandle);
				}
			}
		}
	}
}

void CAN_ProcessFrame(FlagFrameHandle *FlagHandle, uint8_t ID,
		CANBufferHandleStruct *RxBuffer, uint8_t FrameType, uint8_t *Data) {
	if (RxBuffer->NodeHandle[ID].FrameType == FrameType
			&& FlagHandle->FlagID[ID].FlagFrameFull[FrameType] == 0) {
		memcpy(
				RxBuffer->NodeHandle[ID].NodeBuffer[RxBuffer->NodeHandle[ID].FrameType],
				Data, CAN_MAX_DATA);

		FlagHandle->FlagID[ID].FlagFrameFull[FrameType] = 1;
		FlagHandle->FlagID[ID].SumOfFlag +=
				FlagHandle->FlagID[ID].FlagFrameFull[FrameType];
	}
}

uint8_t CAN_Send_Application(CANBufferHandleStruct *AppBuffer,
		CANConfigIDTxtypedef *pStID, uint8_t *Data, uint8_t DataLength) {
	return CAN_Send_Network_Packet(AppBuffer, Data, DataLength, pStID);
}

uint8_t CAN_Send_Network_Packet(CANBufferHandleStruct *TxBuffer, uint8_t *Data,
		uint8_t DataLength, CANConfigIDTxtypedef *pStID) {
	TxBuffer->PacketDataLength = DataLength + 2;
	TxBuffer->CRCValue = crc_8(Data, DataLength);
	TxBuffer->Buffer_Index = DataLength;
	if (TxBuffer->PacketDataLength % 8 == 0) {
		TxBuffer->NumberOfFrame = (TxBuffer->PacketDataLength / 8);
	} else {
		TxBuffer->NumberOfFrame = (TxBuffer->PacketDataLength / 8) + 1;
	}
	memcpy(TxBuffer->NetworkBuffer, Data, DataLength);
	TxBuffer->NetworkBuffer[TxBuffer->Buffer_Index] =
			TxBuffer->PacketDataLength;
	TxBuffer->NetworkBuffer[TxBuffer->Buffer_Index + 1] = TxBuffer->CRCValue;
	TxBuffer->Buffer_Index = 0;
	return CAN_Send_DataLink_Separate(TxBuffer, Data, pStID);
}
uint8_t CAN_Send_DataLink_Separate(CANBufferHandleStruct *TxBuffer,
		uint8_t *Data, CANConfigIDTxtypedef *pStID) {
	uint8_t PacketLength = TxBuffer->PacketDataLength;
	uint8_t NumberOfFrame = TxBuffer->NumberOfFrame;
	TxBuffer->Buffer[NumberOfFrame - 1][6] = PacketLength;
	TxBuffer->Buffer[NumberOfFrame - 1][7] = TxBuffer->CRCValue;
	for (int i = 0; i < NumberOfFrame; i++) {
		for (TxBuffer->Buffer_Index = 0; TxBuffer->Buffer_Index < 8;
				TxBuffer->Buffer_Index++) {
			TxBuffer->Buffer[i][TxBuffer->Buffer_Index] =
					TxBuffer->NetworkBuffer[i * 8 + TxBuffer->Buffer_Index];
			PacketLength--;
			if (PacketLength == 2) {
				break;
			}
		}
		if (PacketLength == 2) {
			break;
		}
	}
	TxBuffer->Buffer_Index = 0;
	return CAN_Send_Physical_Send(TxBuffer, Data, pStID);
}
uint8_t CAN_Send_Physical_Send(CANBufferHandleStruct *TxBuffer, uint8_t *Data,
		CANConfigIDTxtypedef *pIDtype) {
	uint32_t Txmailbox;
	CAN_TxHeaderTypeDef Txheader;
	uint8_t Message_ID = pIDtype->MessageType;
	uint8_t Sender_ID = pIDtype->SenderID;
	uint8_t FrameType = TxBuffer->FrameType_Index;
	uint8_t NumberOfFrame = TxBuffer->NumberOfFrame;
	uint16_t StdId = 0x00;

	StdId |= Message_ID;
	StdId = (StdId << 4) | Sender_ID;
	TxBuffer->SenderID = StdId;
	StdId = (StdId << 3) | TxBuffer->FrameType_Index;
	Txheader.DLC = 8;
	Txheader.RTR = CAN_RTR_DATA;
	Txheader.IDE = CAN_ID_STD;

	for (int8_t i = NumberOfFrame - 1; i >= 0; i--) {
		Txheader.StdId = StdId;
		if (HAL_CAN_AddTxMessage(&hcan, &Txheader, TxBuffer->Buffer[i],
				&Txmailbox) != HAL_OK) {
			Error_Handler();
		}
		while (HAL_CAN_IsTxMessagePending(&hcan, Txmailbox))
			;

		StdId = StdId >> 3;
		FrameType++;
		StdId = (StdId << 3) | FrameType;

	}
	return HAL_OK;
}

void CAN_Recieve_Physical_FIFO0(CAN_RxHeaderTypeDef *RxHeader, uint8_t *Data) {

	while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0)
		;
	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, RxHeader, Data) != HAL_OK) {
		Error_Handler(); //get message from RAM;
	}
}

uint8_t CAN_Receive_DataLink(FlagFrameHandle *FlagHandle,
		CANBufferHandleStruct *RxBuffer, FlagRecNotification *FlagNotiHandle) {
	CAN_RxHeaderTypeDef RxHeader;
	*FlagNotiHandle = REC_DATA;
	uint8_t DataPhysical[CAN_MAX_DATA] = { 0 }; // init local DataPhysical for get data from receive
	uint16_t StdID = 0;
	uint8_t ID = 0;
	CAN_Recieve_Physical_FIFO0(&RxHeader, DataPhysical);
	//CAN_Recieve_Physical_FIFO1(&RxHeader,DataPhysical);
	StdID = RxHeader.StdId;
	ID = (StdID >> 3) & 15;
	RxBuffer->RecvID = ID;
	RxBuffer->NodeHandle[ID].FrameType = StdID & 7; // get frame type store into Rxbuffer struct with Node ID manage frame type
	if (RxBuffer->NodeHandle[ID].FrameType == SET_UP_FRAME
			&& RxBuffer->NodeHandle[ID].DuplicateFrame != 1) { // check if frame type = SET_UP_FRAME
		RxBuffer->NodeHandle[ID].DuplicateFrame = 1; // check send multiple SET_UP_frame
		RxBuffer->NodeHandle[ID].PacketLength = DataPhysical[6];
		RxBuffer->NodeHandle[ID].CRCValue = DataPhysical[7];
		if (RxBuffer->NodeHandle[ID].PacketLength % 8 == 0) {
			RxBuffer->NodeHandle[ID].NumberOfFrame =
					(RxBuffer->NodeHandle[ID].PacketLength / 8);
		} else {
			RxBuffer->NodeHandle[ID].NumberOfFrame =
					(RxBuffer->NodeHandle[ID].PacketLength / 8) + 1;
		}
	} else {
		if (RxBuffer->NodeHandle[ID].FrameType == SET_UP_FRAME) {
			*FlagNotiHandle = REC_FRAMEDATA_ERROR;
			FlagHandle->FlagID[ID].FrameError[RxBuffer->NodeHandle[ID].FrameType] =
					1;
			CAN_Receive_Error_Handle(FlagNotiHandle, FlagHandle);
		}
	}
	CAN_ProcessRxBuffer(FlagHandle, ID, RxBuffer, DataPhysical, FlagNotiHandle);
	return HAL_OK;
}
uint8_t CAN_Receive_Network(CANBufferHandleStruct *NetBuffer,
		FlagFrameHandle *NetworkFlag, FlagRecNotification *FlagNotiHandle) {
	CAN_Receive_DataLink(NetworkFlag, NetBuffer, FlagNotiHandle);
	uint8_t FrameLength = 0;
	uint8_t FrameType = 0;
	uint8_t NetBufferIndex = 0;
	uint8_t DataLength = 0;
	uint8_t CRCValue = 0;
	uint8_t *NetData;
	FrameLength = NetBuffer->NodeHandle[NetBuffer->RecvID].NumberOfFrame;
	uint8_t NumberofFrame = FrameLength;
	FrameType = NetBuffer->NodeHandle[NetBuffer->RecvID].FrameType;
	if (*FlagNotiHandle == REC_FRAMEDATA_SUCCESS) {
		for (; FrameLength > 0; FrameLength--) {
			memcpy(NetBuffer->Buffer[NetBufferIndex],
					NetBuffer->NodeHandle[NetBuffer->RecvID].NodeBuffer[FrameType],
					CAN_MAX_DATA);
			NetBufferIndex++;
			FrameType--;
		}
		DataLength = NetBuffer->NodeHandle[NetBuffer->RecvID].PacketLength - 2;
		NetData = (uint8_t*) malloc(DataLength * sizeof(uint8_t));
		for (NetBufferIndex = 0; NetBufferIndex <= NumberofFrame;
				NetBufferIndex++) {
			for (int j = 0; j < 8; j++) {
				NetData[NetBufferIndex * 8 + j] =
						NetBuffer->Buffer[NetBufferIndex][j];
			}
		}
		CRCValue = crc_8(NetData, DataLength);
		if (CRCValue == NetBuffer->NodeHandle[NetBuffer->RecvID].CRCValue) {
			*FlagNotiHandle = REC_PACKET_SUCCESS;
			memcpy(NetBuffer->NetworkBuffer, NetData, DataLength);
		} else {
			*FlagNotiHandle = REC_PACKET_ERROR;
			CAN_Receive_Error_Handle(FlagNotiHandle, NetworkFlag);
		}
		free(NetData);
	}
	return HAL_OK;
}

uint8_t CAN_Receive_Application(CANBufferHandleStruct *AppBuffer, uint8_t *Data,
		FlagFrameHandle *FlagFrame, FlagRecNotification *FlagNotification) {
	uint8_t AppDataLength =
			AppBuffer->NodeHandle[AppBuffer->RecvID].PacketLength - 2;
	CAN_Receive_Network(AppBuffer, FlagFrame, FlagNotification);
	if (*FlagNotification == REC_PACKET_SUCCESS) {
		memcpy(Data, AppBuffer->NetworkBuffer, AppDataLength);
		*FlagNotification = REC_SUCCESS;
	}
	return HAL_OK;
}

uint32_t CAN_Config_filtering(uint8_t FIFO) {
	CAN_FilterTypeDef Can_filter_init;
	Can_filter_init.FilterActivation = ENABLE;
	Can_filter_init.FilterBank = 0;
	Can_filter_init.FilterFIFOAssignment = FIFO;
	Can_filter_init.FilterIdHigh = 0x0000;
	Can_filter_init.FilterIdLow = 0x0000;
	Can_filter_init.FilterMaskIdHigh = 0x0000;
	Can_filter_init.FilterMaskIdLow = 0x0000;
	Can_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	Can_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;
	if (HAL_CAN_ConfigFilter(&hcan, &Can_filter_init) != HAL_OK) {
		Error_Handler();
	}
	return HAL_OK;
}
uint8_t compress_float_to_uint8(float value, float minRange, float maxRange) {
    if (value < minRange) {
        return 0;
    } else if (value > maxRange) {
        return 255;
    } else {
        float normalizedValue = (value - minRange) / (maxRange - minRange);
        return (uint8_t)(normalizedValue * 255.0f + 0.5f);
    }
}

void float2Bytes( uint8_t bytes_temp[4],float float_variable){
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	CAN_RxHeaderTypeDef RxHeader;
//	uint8_t Data[8]={0};
//	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Data) != HAL_OK) {
//				Error_Handler(); //get message from RAM;
//			}
//}
