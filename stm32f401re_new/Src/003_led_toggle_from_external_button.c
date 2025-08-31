/*
 * led_toggle_from_external_button.c
 *
 *  Created on: Aug 9, 2025
 *      Author: Shivaprasad
 */


#include<stdio.h>
#include<stdint.h>
#include "stm32f401xx.h"
#define LOW DISABLE
void delay(){
	for (uint8_t i=0;i<5000;i++);
}
int main(void)
{
	GPIO_HANDLE_t Led_toggle,button_control;
	Led_toggle.pGPIOx=GPIOA;
	Led_toggle.GPIO_PINCONFIG.GPIO_PIN_NUMBER=GPIO_PIN_NO_5;
	Led_toggle.GPIO_PINCONFIG.GPIO_PIN_MODE=GPIO_MODE_OUT;
	Led_toggle.GPIO_PINCONFIG.GPIO_PIN_SPEED=GPIO_OP_VHSP;
	Led_toggle.GPIO_PINCONFIG.GPIO_OUTPUT_TYPE=GPIO_OP_TYPE_PP;
	Led_toggle.GPIO_PINCONFIG.GPIO_PULLUP_PULLDOWN=GPIO_PIN_NO_PUPD;


	button_control.pGPIOx=GPIOA;
	button_control.GPIO_PINCONFIG.GPIO_PIN_NUMBER=GPIO_PIN_NO_8;
	button_control.GPIO_PINCONFIG.GPIO_PIN_MODE=GPIO_MODE_IN;
	button_control.GPIO_PINCONFIG.GPIO_PIN_SPEED=GPIO_OP_VHSP;
	button_control.GPIO_PINCONFIG.GPIO_OUTPUT_TYPE=GPIO_OP_TYPE_PP;
	button_control.GPIO_PINCONFIG.GPIO_PULLUP_PULLDOWN=GPIO_PIN_PU;
	GPIO_PeriClKCtrl(GPIOA, ENABLE);
	GPIO_Init(&Led_toggle);
	GPIO_Init(&button_control);
	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_8)==LOW)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

		}

	}

}
