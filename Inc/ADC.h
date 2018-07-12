
#ifndef __ADC_H__
#define __ADC_H__

#include "stm32f3xx_hal.h"
#include "main.h"
#include "can_ECUP.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/*initialization */
void init_measurements(void);
void calib_init(void);
void start_ADC(ADC_HandleTypeDef* ADC_handle,uint8_t witch);
/*end of initialization*/

/*transfer from raw data*/
void Pedals_process(CAN_HandleTypeDef *hcan);

/*end of transfer from raw data*/

void ERROR_check_pressure(void);
void stop_ADC(ADC_HandleTypeDef* ADC_handle);
void ERROR_check_acc_and_percent_transfer(void);

/*sending all can messages*/
void Send_can_messagess(CAN_HandleTypeDef *hcan);
/*end of sending all can messages*/


/*debug*/
void BLIK_LED(GPIO_TypeDef * port,uint16_t pin);
/*end of debug*/

/*shut down of ECUP i guess useless*/
void stop_ADC(ADC_HandleTypeDef* ADC_handle);
/*end of shut down of ECUP i guess useless*/

#endif

/* END OF FILE */
