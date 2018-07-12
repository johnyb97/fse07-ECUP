#include "test_sensors_connection.h"
void test_mode_pin(uint16_t pin,GPIO_TypeDef *port){ // zmena modu gpio pinu na cteci 
	GPIO_InitTypeDef pullup;
	pullup.Mode = GPIO_MODE_INPUT;
	pullup.Pin = pin;
	pullup.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(port, &pullup);

}

void reset_mode_pin(uint16_t pin,GPIO_TypeDef *port){// zmena modu gpio pinu na zapis
	GPIO_InitTypeDef pullup;
	pullup.Mode = GPIO_MODE_OUTPUT_PP;
	pullup.Pin = pin;
	pullup.Pull = GPIO_NOPULL;
	pullup.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(port, &pullup);
}

void test_conection(uint16_t pin,GPIO_TypeDef *port,uint8_t* error1,uint8_t* error2,ECUP_Status_t *ECUP_stat){ // testovani odpojeni senzoru dle navodu patrika 
	volatile uint32_t time;
	volatile uint32_t nowtime;
	HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET);//nastaveni pinu na 1
	HAL_Delay(2);
	test_mode_pin(pin,port); // zmena nasraveni pinu
	HAL_Delay(2);
	if(HAL_GPIO_ReadPin(port,pin) == 1 ){//ve chvili kdyz tam je 1 hlasim error nepripojeni senzoru
		ECUP_stat->FT_ANY = 1;
		*error1 = 0x1;
		*error2 = 0x1;
	}
	else{
		ECUP_stat->FT_ANY = 1;
		*error1 = 0;
		*error2 = 0;
	}
	reset_mode_pin(pin,port);
}
