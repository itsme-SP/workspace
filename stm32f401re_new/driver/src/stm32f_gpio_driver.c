/*
 * stm32f_gpio_driver.c
 *
 *  Created on: Aug 6, 2025
 *      Author: Shivaprasad
 */

#include "stm32f_gpio_driver.h"

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
	 else if (pGPIOx==GPIOH)
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


//API to initialize the gpio pin/port
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
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle){
	uint32_t temp=0;

	/*
	 * enabling the peripheral clock
	 * we are doing this here is beacuse many people will forget to turn on the clock by doing this the clock for the particular port will be on always
	 *
	 */

	GPIO_PeriClKCtrl(pGPIOHandle->pGPIOx, ENABLE);

	//1 configure the mode of the gpio pin

	if(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_MODE<=GPIO_MODE_ANALOG){
		//non interrupt mode
		temp=(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_MODE<<(2 *pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
		pGPIOHandle->pGPIOx->MODER&=~(0X3<<(2*pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
		pGPIOHandle->pGPIOx->MODER|=temp;

	}
	else{
		//this part is for interrupt mode
		if(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_MODE==GPIO_MODE_IT_FT)
		{
			//1 configure the falling trigger register selection (FTSR)
			EXTI->FTSR|=(1<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
			// clear the corresponding rtsr register
			EXTI->RTSR&=~(1<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
		}
		else if(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_MODE==GPIO_MODE_IT_RT){
			//1 configure the rising trigger selection register(RTSR)
			EXTI->RTSR|=(1<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
			//clear the corresponding ftsr
			EXTI->FTSR&=~(1<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
		}
		else if(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_MODE==GPIO_MODE_IT_RFT){
			//1 configure the rising and falling trigger selection register (FTSR and RTSR)
			EXTI->FTSR|=(1<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
			EXTI->RTSR|=(1<<(pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER));
		}

		//2 configure the GPIO port selection in SYSCFG_EXTICR (system configuration controller external interrupt configuration register)
		//lecture number 111
		uint8_t temp1=pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER/4;
		uint8_t temp2=pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER%4;
		uint8_t portcode=GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);//GPIO_BASE_ADDR_TO_CODE will check if the port is A...E and the corresponding value will be stored in portcode
		SYSCONFIG_PCLK_EN();
		SYSCONFIG->EXTICR[temp1]=portcode<<((temp2*4));
		//3 configure IMR(Interrupt mask register)
		EXTI->IMR|=(1<<pGPIOHandle->GPIO_PINCONFIG.GPIO_PIN_NUMBER);

	}
	temp=0;


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
 * @fn                           -GPIO_IRQ_Interrupt_Config
 *
 * @brief                        -this function will disable or enable the interrupt
 *
 * IRQNumber                      -interrupt request number this will decide which intterupt vector we are using from the NVIC table
 *EN_OR_DI                        -Whether to enable r disable the interrupt using the ENABLE or DISABLE macros
 * @return                      -none
 *
 * @Note                        -there are 60 Interrupt priority registers in ARM cortex m4 from IPR0 to IPR59 each register is 32 bits wide and divided into 4 sections each 8 bit wide-
 *                               each section is for one irqnumber for setting the irq priority
 */
void GPIO_IRQ_Interrupt_Config(uint8_t IRQNumber,uint8_t EN_OR_DI)//used to configure the irq number to like enabling interrupt setting the priority
{
	//this is to enable the irq register only the register we need
	if(EN_OR_DI==ENABLE){
		//refer the arm m4 user guide pg no219 lecture number 112
		if(IRQNumber<=31){
			// program the ISER0(interrupt set enable register 0)
			*NVIC_ISER0_BASE_ADDRESS|=(1<<IRQNumber);

		}
		else if(IRQNumber>31&&IRQNumber<=64){
			// program the ISER1(interrupt set enable register 1)
			*NVIC_ISER1_BASE_ADDRESS|=(1<<IRQNumber%32);
		}
		else if(IRQNumber>64&&IRQNumber<=95){
			// program the ISER2(interrupt set enable register 2)
			*NVIC_ISER2_BASE_ADDRESS|=(1<<IRQNumber%64);

		}
	}
	  //this is to disable the interrupt registers
	else{
		if(IRQNumber<=31){
			// program the ICER0(interrupt clear enable register 0)
			*NVIC_ICER0_BASE_ADDRESS|=(1<<IRQNumber);
		}
		else if(IRQNumber>31&&IRQNumber<=64){
			// program the ICER1(interrupt clear enable register 1)
			*NVIC_ICER1_BASE_ADDRESS|=(1<<IRQNumber%32);
		}
		else if(IRQNumber>64&&IRQNumber<=95){
			// program the ICER2(interrupt clear enable register 2)
			*NVIC_ICER2_BASE_ADDRESS|=(1<<IRQNumber%64);
		}
	}
}

/*
 * @fn                           -IRQ_Priority_Config
 *
 * @brief                        -setting the priority of the interrupt
 *IRQNumber                     -Interrupt request number which is taken from the NVIC vector table where each intterupt is numbered and for our microcontroller we have 89
 * IRQPriority                  -what will be priority of the present interrupt
 * @return                      -none
 *
 * @Note                        -there are 60 Interrupt priority registers in ARM cortex m4 from IPR0 to IPR59 each register is 32 bits wide and divided into 4 sections each 8 bit wide-
 *                               each section is for one irqnumber for setting the irq priority
 */

void GPIO_IRQ_Priority_Config(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//to find the ipr register lecture number113
	uint8_t iprx=IRQNumber/4;
	//to find which section to use in the ipr register
	uint8_t iprx_section=IRQNumber%4;
	//in priority in arm the the least four bits are Not applicable so we cannot use that so we add four to each iprx_section*8 so that the bits are shifted to top 4 bits instead of leat 4 lecture no 113
	uint8_t shift_amount=(iprx_section*8)+4;
    *(NVIC_IPR_BASE_ADDRESS+(iprx))|=(IRQPriority<<shift_amount);
}

/*
 * @fn                           -GPIO_IRQHandling
 *
 * @brief                        -In stm32 the irs is always defined in the startup code so to call it we will use the code from there but to terminate the interrupt we must set the pending register as 1 and that is what
 *                                we do in here
 *
 * PinNumber                     -the pin where we will execute the Interruot
 *
 *
 * @return                      -none
 *
 * @Note                        -none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the opin number
	if(EXTI->PR&(1<<PinNumber)){ //here we will check if the and operation between EXTI->PR and 1<<pin number is 1 then it will move inside the if block
		//clear
		EXTI->PR|=(1<<PinNumber);
	}
}
