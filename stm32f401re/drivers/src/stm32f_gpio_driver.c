/*
 * stm32f_gpio_driver.c
 *
 *  Created on: Aug 3, 2025
 *      Author: Shivaprasad
 */


#include "stm32f401xx.h"

/*
 * Peripheral clock control
 */

/*
 * @fn                           -GPIO_PeriClKCtrl
 *
 * @brief                        -this function will either enable the peripheral clock or disable it
 *
 * GPIO_Register_Definition_t   -base address of the GPIO peripheral
 * EN_OR_DI                     - ENABLE OR DISABLE MACROS
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_PeriClKCtrl(GPIO_Register_Definition_t *pGPIOx,uint8_t EN_OR_DI){
 if(EN_OR_DI==ENABLE)
 {
	 if(pGPIOx==GPIOA){
		 GPIOA_PCLK_EN();
	 }else if(pGPIOx==GPIOB){
		 GPIOB_PCLK_EN();
	 }
	 else if(pGPIOx==GPIOC)
	 {
		 GPIOC_PCLK_EN();

	 }
	 else if(pGPIOx==GPIOD)
	 	 {
	 		 GPIOD_PCLK_EN();

	 	 }
	 else if(pGPIOx==GPIOE)
	 	 {
	 		 GPIOE_PCLK_EN();

	 	 }
	 else
	 	 {
	 		 GPIOH_PCLK_EN();

	 	 }
 }
 else{
	 if(pGPIOx==GPIOA){
			 GPIOA_PCLK_DI();
		 }else if(pGPIOx==GPIOB){
			 GPIOB_PCLK_DI();
		 }
		 else if(pGPIOx==GPIOC)
		 {
			 GPIOC_PCLK_DI();

		 }
		 else if(pGPIOx==GPIOD)
		 	 {
		 		 GPIOD_PCLK_DI();

		 	 }
		 else if(pGPIOx==GPIOE)
		 	 {
		 		 GPIOE_PCLK_DI();

		 	 }
		 else
		 	 {
		 		 GPIOH_PCLK_DI();

		 	 }
 }
}



/*
 * @fn                           -GPIO_Init
 *
 * @brief                        -this function will enable the GPIO PIN/PORT
 *
 * GPIO_HANDLE_t                -It is like a folder that has multiple folders stored and each folder may point to the same structure or different it is just used to reduce clutter
 *
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle){//API to initialize the gpio pin/port
	uint32_t temp=0;
	//1 configure the mode of the gpio pin

	if(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_MODE<=GPIO_MODE_ANALOG){
		//non interrupt mode
		temp=(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_MODE<<(2*pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
		pGPIOHandle->pGPIOx->MODER&=~(0X3<<(2*pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
		pGPIOHandle->pGPIOx->MODER|=temp;
		temp=0;

	}
	else{
		//this part is for later(interrut mode)
	}

	//2 configure the speed
	temp=(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_SPEED<<(2*pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OSPEEDR&=~(0X3<<(2*pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OSPEEDR|=temp;
	temp=0;

	//3 configure the pull up pull dwn settings
	temp=(pGPIOHandle->GPIO_PINCONFIG.GPIO_PULLUP_PULLDOWN<<(2*pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->PUPDR&=~(0X3<<(2*pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->PUPDR|=temp;
	temp=0;

	//4 configure the output types
	temp=(pGPIOHandle->GPIO_PINCONFIG.GPIO_OUTPUT_TYPE<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OTYPER&=~(1<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
	pGPIOHandle->pGPIOx->OTYPER|=temp;

	//5 configure the alternate functionality if required
	if(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_ALT_FUNC_MODE==GPIO_MODE_ALTFN){
		//watch lecture number 97 for doubt clarification
		uint8_t temp1,temp2;
		temp1=pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER/8;
		temp2=pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER%8;
		pGPIOHandle->pGPIOx->AFR[temp1]&=~(0XF<<(4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1]|=(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_ALT_FUNC_MODE<<(4*temp2));

	}
}


/*
 * @fn                           -GPIO_Init
 *
 * @brief                        -this function will disable the GPIO PIN/PORT
 *
 * *pGPIOx                       -base address of the GPIO peripheral
 *
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_DeInit(GPIO_Register_Definition_t *pGPIOx)//API to deinitialize the gpio pin/port
{
	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}
	else{
		GPIOH_REG_RESET() ;
	}
}






/*
 * @fn                           -GPIO_ReadFromInputPin
 *
 * @brief                        -this function will read input from the pin
 *
 * *pGPIOx                       -base address of the GPIO peripheral
 *PinNumber                      -Pin number from where we will read the input
 *
 * @return                      -uint8_t 0 or 1
 *
 * @Note                        -none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Register_Definition_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR>>PinNumber)&0X00000001);
	return value;
}

/*
 * @fn                           -GPIO_ReadFromInputPort
 *
 * @brief                        -this function will read input from the port that means from all the pins of the port
 *
 * *pGPIOx                       -base address of the GPIO peripheral
 *
 *
 * @return                      -uint16_t
 *
 * @Note                        -none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Register_Definition_t *pGPIOx){
	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;

	return value;
}


/*
 * @fn                           -GPIO_WriteToOutputPin
 *
 * @brief                        -this function will send the output to the pin of the  port
 *
 * *pGPIOx                       -base address of the GPIO peripheral
 *Value                      -it is a macro that will either send 1 or 0 from the ENABLE or DISABLE Macro
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_WriteToOutputPin(GPIO_Register_Definition_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
	if(Value==GPIO_PIN_SET)
	{
		//writing 1 to the output data register  at the bit field corresponding to the pin umber
		pGPIOx->ODR|=(1<<PinNumber);
	}
	else
	{
		//writing 0 to the output data register at the bit field corressponding to the pin number
		pGPIOx->ODR&=~(1<<PinNumber);
	}

}


/*
 * @fn                           -GPIO_WriteToOutputPort
 *
 * @brief                        -this function will send the output to the port
 *
 * *pGPIOx                       -base address of the GPIO peripheral
 *Value                      -it is a macro that will either send 1 or 0 from the ENABLE or DISABLE Macro
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_WriteToOutputPort(GPIO_Register_Definition_t *pGPIOx,uint16_t Value){
	pGPIOx->ODR=Value;
}

/*
 * @fn                           -GPIO_ToggleOutputPin
 *
 * @brief                        -this function will toggle the pin changing from 0 to 1 continuously
 *
 * *pGPIOx                       -base address of the GPIO peripheral
 *PinNumber                      -The pin to choose where we will toggle
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_ToggleOutputPin(GPIO_Register_Definition_t *pGPIOx,uint8_t PinNumber)//toggle the output
{
	pGPIOx->ODR ^= (1<<PinNumber);

}
/*
 * Interrupt Isr configuration and ISR handling
 */

/*
 * @fn                           -GPIO_IRQConfig
 *
 * @brief                        -this function will determine how the interrupt will behave before it is actually used
 *
 * IRQNumber                      -interrupt request number this will decide which intterupt vector we are using from the NVIC table
 *IRQPriority                      -what will be priority of the present interrupt
 *EN_OR_DI                        -Whether to enable r disable the interrupt using the ENABLE or DISABLE macros
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EN_OR_DI);//used to configure the irq number to like enabling interrupt setting the priority


/*
 * @fn                           -GPIO_IRQHandling
 *
 * @brief                        -the main program will stop what it was executing and come to this function and will execute the function and go back
 *
 * PinNumber                     -the pin where we will execute the Interruot
 *
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_IRQHandling(uint8_t PinNumber); //whenever we get an interrupt the user application program can call this function to handle it
