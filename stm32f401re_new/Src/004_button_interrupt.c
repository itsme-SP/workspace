/*
 * button_interrupt.c
 *
 *  Created on: Aug 19, 2025
 *      Author: Shivaprasad
 */


#include<stdio.h>
#include <stm32f401xx.h>
void delay(){
	for(int i=0;i<500000/2;i++);
}
int main(void){
	GPIO_HANDLE_t GPIO_led,GPIO_button_control;
	/*we are setting the structure variables to zero because garbage values will come if we
	 *  dont initialize some structure variables just like in the GPIO_button_control where we did not
	 *  specify gpio_output type aand it will cause an error by keeping some garbage value that
	 *  will create a bug in the program
	 *  memset belongs to standard library that is from string.h and we have included it in our microcontroller specific file
	 *  lecture number 115
	*/

	memset(&GPIO_led,0,sizeof(GPIO_led));
	memset(&GPIO_button_control,0,sizeof(GPIO_button_control));

	GPIO_led.pGPIOx=GPIOA;
	GPIO_led.GPIO_PINCONFIG.GPIO_PIN_NUMBER=GPIO_PIN_NO_5;
	GPIO_led.GPIO_PINCONFIG.GPIO_PIN_MODE=GPIO_MODE_OUT;
	GPIO_led.GPIO_PINCONFIG.GPIO_PIN_SPEED=GPIO_OP_HSP;
	GPIO_led.GPIO_PINCONFIG.GPIO_OUTPUT_TYPE=GPIO_OP_TYPE_PP;
	GPIO_led.GPIO_PINCONFIG.GPIO_PULLUP_PULLDOWN=GPIO_PIN_NO_PUPD;
	GPIO_PeriClKCtrl(GPIOA, ENABLE);
	GPIO_Init(&GPIO_led);

	GPIO_button_control.pGPIOx=GPIOC;
	GPIO_button_control.GPIO_PINCONFIG.GPIO_PIN_NUMBER=GPIO_PIN_NO_13;
	GPIO_button_control.GPIO_PINCONFIG.GPIO_PIN_MODE=GPIO_MODE_IT_FT;
	GPIO_button_control.GPIO_PINCONFIG.GPIO_PIN_SPEED=GPIO_OP_HSP;
	GPIO_button_control.GPIO_PINCONFIG.GPIO_PULLUP_PULLDOWN=GPIO_PIN_PU;

	GPIO_PeriClKCtrl(GPIOC, ENABLE);
	GPIO_Init(&GPIO_button_control);
	//irq configurations
	GPIO_IRQ_Priority_Config(IRQ_NO_EXTI0,IRQ_PRIO15);
	GPIO_IRQ_Interrupt_Config(IRQ_NO_EXTI0,ENABLE);
}
void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);//clear the pending event from the exti line
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

}
