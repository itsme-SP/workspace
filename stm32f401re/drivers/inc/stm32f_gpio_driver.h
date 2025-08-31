/*
 * stm32f_gpio_driver.h
 *
 *  Created on: Aug 3, 2025
 *      Author: Shivaprasad
 */

#ifndef INC_STM32F_GPIO_DRIVER_H_
#define INC_STM32F_GPIO_DRIVER_H_

#include <stdint.h>
#include <stdio.h>

//configurable items for user applications
typedef struct{
	uint32_t GPIO_PIN_NUMBER; //possible values from @GPIO_PIN_NUMBER
	uint32_t GPIO_PIN_MODE;    //possible values form @GPIO_PIN_MODEs
	uint32_t GPIO_PIN_SPEED;   //possible values from @GPIO_PIN_SPEED
	uint32_t GPIO_OUTPUT_TYPE; //PO
	uint32_t GPIO_PULLUP_PULLDOWN;//GPIO pin pull up or pull down mode @GPIO_PIN_PUPD
	uint32_t GPIO_PIN_ALT_FUNC_MODE;//GPIO pin alternate function mode

}GPIO_PINCONFIG_t;


/*
 * this is a handle structure for a gpio pins
 */
typedef struct{
	GPIO_Register_Definition_t *pGPIOx;//this holds the base address of the GPIO port to which the pin belongs
	GPIO_PINCONFIG_t GPIO_PINCONFIG;
}GPIO_HANDLE_t;


//@GPIO_PIN_NUMBER
#define GPIO_PIN_NO_0  0
#define GPIO_PIN_NO_1  1
#define GPIO_PIN_NO_2  2
#define GPIO_PIN_NO_3  3
#define GPIO_PIN_NO_4  4
#define GPIO_PIN_NO_5  0X5
#define GPIO_PIN_NO_6  6
#define GPIO_PIN_NO_7  7
#define GPIO_PIN_NO_8  8
#define GPIO_PIN_NO_9  9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15


//GPIO pin possible modes @GPIO_PIN_MODEs
//158 pg no rm GPIO register modes
#define GPIO_MODE_IN  0X0//input mode
#define GPIO_MODE_OUT 0X1//output mode
#define GPIO_MODE_ALTFN     0X10 //ALternate functionality mode
#define GPIO_MODE_ANALOG 0X11
//the modes we are giving not presetn in rm
#define GPIO_MODE_IN_FT 4//GPIO MODE INPUT FALLING EDGETRIGGER
#define GPIO_MODE_IN_RT 5////GPIO MODE INPUT RISING EDGE TRIGGER
#define GPIO_MODE_IN_RFT 5////GPIO MODE INPUT RISING EDGE FALLING EDGE TRIGGER

//GPIO output type register from rm pg no 158 this will state what kind of register the output port will act as
#define GPIO_OP_TYPE_PP 0 //Output push pull reset state
#define GPIO_OP_TYPE_OD 1 //Output open drain


//@GPIO_PIN_SPEED
//Gpio output speed register rm pg no 159
#define GPIO_OP_LSP 0X00 //GPIO output low speed
#define GPIO_OP_MSP 0X01 //GPIO output Medium speed
#define GPIO_OP_HSP 0X02 //GPIO output High speed
#define GPIO_OP_VHSP 0X03 //GPIO output Very high speed

//@GPIO_PIN_PUPD
//GPIO port pullup/pulldown registers rm pg no 159
#define GPIO_PIN_NO_PUPD 0X00 //GPIO Pull up pull down register : no pull up no pull down mode
#define GPIO_PIN_PU 0X01 //GPIO Pull Up pull down register : pull up mode
#define GPIO_PIN_PD 0X10 //GPIO Pull Up pull down register : pull down mode

/*
 * API's which will be supported by the drivers
 */

/*
 * Peripheral clock control
 */
void GPIO_PeriClKCtrl(GPIO_Register_Definition_t *pGPIOx,uint8_t EN_OR_DI);

/*
 * Init and deinit
 */
void GPIO_Init(GPIO_HANDLE_t *pGPIOHandle);//API to initialize the gpio pin/port
void GPIO_DeInit(GPIO_Register_Definition_t *pGPIOx);//API to deinitialize the gpio pin/port
/*
 * Read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Register_Definition_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Register_Definition_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_Register_Definition_t *pGPIOx,uint8_t PinNumber,uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Register_Definition_t *pGPIOx,uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Register_Definition_t *pGPIOx,uint8_t PinNumber);//toggle the output
/*
 * Interrupt Isr configuration and ISR handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EN_OR_DI);//used to configure the irq number to like enabling interrupt setting the priority
void GPIO_IRQHandling(uint8_t PinNumber); //whenever we get an interrupt the user application program can call this function to handle it

#endif /* INC_STM32F_GPIO_DRIVER_H_ */
