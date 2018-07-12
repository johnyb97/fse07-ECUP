#ifndef EFORCE_CAN_INIT
#define EFORCE_CAN_INIT

#include <string.h>
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_can.h"

int CAN_eforce(CAN_HandleTypeDef *hcan,CAN_TypeDef *Instance,CanRxMsgTypeDef *canRxMsg);

#endif
