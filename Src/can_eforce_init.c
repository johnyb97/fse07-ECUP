#include "can_eforce_init.h"
int CAN_eforce(CAN_HandleTypeDef *hcan,CAN_TypeDef *Instance,CanRxMsgTypeDef*canRxMsg)
{
    CanTxMsgTypeDef txMsg; //added
    hcan->Instance = Instance; //CAN1, CAN2
    hcan->Init.Prescaler = 9;
    hcan->Init.Mode = CAN_MODE_SILENT_LOOPBACK;
    hcan->Init.SJW = CAN_SJW_1TQ;
    hcan->Init.BS1 = CAN_BS1_6TQ;
    hcan->Init.BS2 = CAN_BS2_1TQ;
    hcan->Init.TTCM = 0;
    hcan->Init.ABOM = 1;
    hcan->Init.AWUM = 0;
    hcan->Init.NART = 0;
    hcan->Init.RFLM = 0;
    hcan->Init.TXFP = 0;
    HAL_CAN_Init(hcan);

    if (hcan->ErrorCode != HAL_CAN_ERROR_NONE)
    {
        return 0;
    }
    CAN_FilterConfTypeDef canFilter;
    memset(&canFilter, 0, sizeof(CAN_FilterConfTypeDef) );
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0; //fixed
    canFilter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(hcan, &canFilter) != HAL_OK)
    {
        return 0;
    }


    // Do a selftest
    memset(canRxMsg, 0, sizeof(CanRxMsgTypeDef));
    hcan->pRxMsg = canRxMsg;

    memset(&txMsg, 0, sizeof(CanTxMsgTypeDef) );
    txMsg.StdId = 0x42;
    txMsg.IDE = CAN_ID_STD;
    txMsg.RTR = CAN_RTR_DATA;
    txMsg.DLC = 2;
    txMsg.Data[0] = 0xCA;
    txMsg.Data[1] = 0xFE;
    hcan->pTxMsg = &txMsg;

    // send message
    if (HAL_CAN_Transmit(hcan, 1000) != HAL_OK)
    {
        return 0;
    }

    // recieve message and validate
    if (HAL_CAN_Receive(hcan, CAN_FIFO0, 1000) != HAL_OK ||
        canRxMsg->StdId != txMsg.StdId ||
        canRxMsg->DLC != txMsg.DLC ||
        memcmp(&txMsg.Data, canRxMsg->Data, sizeof(uint32_t) * txMsg.DLC) )
    {
        return 0;
    }

    // If selftest passed -> reinit in normal mode
    hcan->Init.Mode = CAN_MODE_NORMAL;
    //hcan->Init.Mode = CAN_MODE_SILENT_LOOPBACK;hcan->Init.Mode =
    //canHandle.Init.Mode = CAN_MODE_LOOPBACK;

    HAL_CAN_Init(hcan);
    if (hcan->ErrorCode != HAL_CAN_ERROR_NONE)
    {
        return 0;
    }

    // enable interrupts

    HAL_NVIC_EnableIRQ(CAN_SCE_IRQn);
    HAL_NVIC_SetPriority(CAN_SCE_IRQn, 4, 0);

    HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN_RX0_IRQn, 3, 0);


    return 1;
}
