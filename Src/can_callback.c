#include "ringbuf.h"
#include "stm32f3xx_hal.h"
#include "can.h"
#include <string.h>

extern CAN_HandleTypeDef hcan;
CanTxMsgTypeDef txMsg;
enum { TIMEOUT_CAN_SEND = 3}; //time in wich message must be send


uint32_t txGetTimeMillis(void){
	return (uint32_t)HAL_GetTick();
}
int txHandleCANMessage(uint32_t timestamp, CAN_ID_t id, const void* data, size_t length){
	return 0;
}

static int SendCANMessage(CAN_HandleTypeDef* hcan, CAN_ID_t id, const void* data, size_t length) {
	memset(&txMsg, 0, sizeof(CanTxMsgTypeDef) );
	txMsg.IDE = CAN_ID_STD;
	txMsg.RTR = CAN_RTR_DATA;
	txMsg.StdId = id;
	txMsg.DLC = length;
	memcpy(txMsg.Data, data, length);

	hcan->pTxMsg = &txMsg;

	if (HAL_CAN_Transmit(hcan, TIMEOUT_CAN_SEND) == HAL_OK) {
		//status_post(STATUS_OK_CAN_SEND);
		return 1;
	}
	else {
		//status_post(STATUS_WARN_CAN_SEND_FAIL);
		return 0;
	}
}
int txSendCANMessage(CAN_ID_t id, const void* data, size_t length)
{
   return SendCANMessage(&hcan, id, data, length);
}
