#ifndef TEST_CONNECT
#define TEST_CONNECT
#include "stm32f3xx_hal.h"
#include "main.h"
#include "can_ECUP.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

void test_conection(uint16_t pin,GPIO_TypeDef *port,uint8_t* error1,uint8_t* error2,ECUP_Status_t *ECUP_stat);


#endif
