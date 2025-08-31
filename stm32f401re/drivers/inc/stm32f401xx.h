/*
 * stm32f401xx.h
 *
 *  Created on: Aug 3, 2025
 *      Author: Shivaprasad
 */

#ifndef INC_STM32f401XX_H_
#define INC_STM32F401XX_H_


#include <stdint.h>

#define __vo volatile
//base address of SRAM and FLASH
#define SRAM1_BASE_ADDR  0x20000000U//S ram base address found in
#define FLASH_BASE_ADDR  0x08000000U //flash base address found in rm p45 embedded flash memory interface
#define SRAM             SRAM1_BASE_ADDR //SRAM1 is the only s ram so SRAM1_BASE_ADDR = SRAM
#define ROM              0x1FFF0000U //Flash address for system memory we cannot change anything in here found in pg45 rmf



//base address of peripherals
#define PERIPH_BASE            0X40000000U //found in memory map in pg number 38 rm memory map
#define APB1_PERIPHERAL_BASE   PERIPH_BASE//found in memory map in pg number 38 rm memory map same for all the other below
#define APB2_PERIPHERAL_BASE   0X40010000U
#define AHB1_PERIPHERAL_BASE   0x40020000U
#define AHB2_PERIPHERAL_BASE   0x50000000U


// Base address of the peripherals which are hanging on the AHB1 bus
#define GPIOA_BASE_ADDRESS           (AHB1_PERIPHERAL_BASE)
#define GPIOB_BASE_ADDRESS           ((AHB1_PERIPHERAL_BASE)+(0X00000400))         //these are found in  the memory map of the rm pg no 38
#define GPIOC_BASE_ADDRESS           ((AHB1_PERIPHERAL_BASE)+(0X00000800))
#define GPIOD_BASE_ADDRESS           ((AHB1_PERIPHERAL_BASE)+(0X00000C00))
#define GPIOE_BASE_ADDRESS           ((AHB1_PERIPHERAL_BASE)+(0X00001000))
#define GPIOH_BASE_ADDRESS           ((AHB1_PERIPHERAL_BASE)+(0X00001C00))
#define CRC_BASE_ADDRESS             ((AHB1_PERIPHERAL_BASE)+(0X00003000))
#define RCC_BASE_ADDRESS             ((AHB1_PERIPHERAL_BASE)+(0X00003800))
#define FLASH_INTERFACE_BASE_ADDRESS ((AHB1_PERIPHERAL_BASE)+(0X00003800))
#define DMA1_BASE_ADDRESS             ((AHB1_PERIPHERAL_BASE)+(0X00006000))
#define DMA2_BASE_ADDRESS             ((AHB1_PERIPHERAL_BASE)+(0X00006400))





//base address of the peripherals hanging on the AHB2 Bus

#define USB_OTG_BASE_ADDRESS                    AHB2_PERIPHERAL_BASE



//base address of the peripherals hanging on the APB1 Bus
#define TIM2_BASE_ADDRESS             APB1_PERIPHERAL_BASE
#define TIM3_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00000400))
#define TIM4_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00000800))
#define TIM5_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00000C00))
#define RTC_AND_BKP_BASE_ADDRESS        ((APB1_PERIPHERAL_BASE)+(0x00002800))
#define WWDG_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00002C00))
#define IWDG_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00003000))
#define I2S2EXT_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00003400))
#define SPI2_AND_I2S2_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00003800))
#define SPI3_AND_I2S3_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00003C00))
#define I2S3EXT_BASE_ADDRESS             ((APB1_PERIPHERAL_BASE)+(0X00004000))
#define USART2_BASE_ADDRESS              ((APB1_PERIPHERAL_BASE)+(0X00004400))
#define I2C1_BASE_ADDRESS              ((APB1_PERIPHERAL_BASE)+(0X00005400))
#define I2C2_BASE_ADDRESS              ((APB1_PERIPHERAL_BASE)+(0X00005800))
#define I2C3_BASE_ADDRESS              ((APB1_PERIPHERAL_BASE)+(0X00005C00))
#define PWR_BASE_ADDRESS              ((APB1_PERIPHERAL_BASE)+(0X00007000))



//base address of the peripherals hanging on the APB2 Bus
#define TIM1_BASE_ADDRESS             APB2_PERIPHERAL_BASE
#define USART1_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00001000))
#define USART6_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00001400))
#define ADC1_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00002000))
#define SDIO_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00002C00))
#define SPI1_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00003000))
#define SPI4_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00003400))
#define SYSCONFIG_BASE_ADDRESS       ((APB2_PERIPHERAL_BASE)+(0X00003800))
#define EXTI_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00003C00))
#define TIM9_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00004000))
#define TIM1O_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00004400))
#define TIM11_BASE_ADDRESS           ((APB2_PERIPHERAL_BASE)+(0X00004800))






/*
 * registers of a peripheral are specific to MCU
 * eg no of registers of SPI peripheral of stm32F401RE may be different than the SPI peripheral of STM32M01
 *
 */

/*
 * this tsructure is for gpio mode registers
 */

typedef struct
{
	/*took this from the register map under gpio at pg no 164 to according to the address and then used
	that to set he registers lecture number 84
	then using the rcc registers named them so these registers tell the Gpio port how they should behave
	*/
	__vo uint32_t MODER;   //GPIO port mode register
	__vo uint32_t OTYPER;  //GPIO port output type register
	__vo uint32_t OSPEEDR; //GPIO port output speed register
	__vo uint32_t PUPDR;   //GPIO port pull-up/pull-down register
	__vo uint32_t IDR;     //GPIO port input data register
	__vo uint32_t ODR ;    //GPIO port output data register
	__vo uint32_t BSRR;    //GPIO port bit set/reset register
	__vo uint32_t LCKR;    //GPIO port configuration lock register

	//we have 2 alternate function registers in here so instead of creating two variables we created an array,
	__vo uint32_t AFR[2];	//AFR[0]=alternate function low register,AFR[1]=alternate function high register
}GPIO_Register_Definition_t;


/*
 * to Initialize the clock we will define a structure and then create variables inside the structure to hold the values of the clock to be  initialized
 */


typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t RESERVED1[2];
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t DCKCFGR;

}RCC_register_definition_t;

/*
 * peripheral base address typecasted to the structure( GPIO_Register_Definition_t)
 */

#define GPIOA ((GPIO_Register_Definition_t*)GPIOA_BASE_ADDRESS)
#define GPIOB ((GPIO_Register_Definition_t*)GPIOB_BASE_ADDRESS)
#define GPIOC ((GPIO_Register_Definition_t*)GPIOC_BASE_ADDRESS)
#define GPIOD ((GPIO_Register_Definition_t*)GPIOD_BASE_ADDRESS)
#define GPIOE ((GPIO_Register_Definition_t*)GPIOE_BASE_ADDRESS)
#define GPIOH ((GPIO_Register_Definition_t*)GPIOH_BASE_ADDRESS)

/*
 * RCC AHB1 peripheral reset register (RCC_AHB1RSTR)
 * register reset macros from rm pg no 112
 * this is used to clear the whole port and peripherals by a single value
 * 1 to reset the whole register and 0 to not change anything
 */




//RCC Base address typecasted to the structure(RCC_register_definition_t)
#define RCC ((RCC_register_definition_t*)RCC_BASE_ADDRESS)


//clock enable macros for GPIO peripherals
#define GPIOA_PCLK_EN() (RCC->AHB1ENR|=(1<<0)) //to enable GPIOA peripheral clock
#define GPIOB_PCLK_EN() (RCC->AHB1ENR|=(1<<1)) //to enable GPIOA peripheral clock
#define GPIOC_PCLK_EN() (RCC->AHB1ENR|=(1<<2)) //to enable GPIOA peripheral clock
#define GPIOD_PCLK_EN() (RCC->AHB1ENR|=(1<<3)) //to enable GPIOA peripheral clock
#define GPIOE_PCLK_EN() (RCC->AHB1ENR|=(1<<4)) //to enable GPIOA peripheral clock
#define GPIOH_PCLK_EN() (RCC->AHB1ENR|=(1<<5)) //to enable GPIOA peripheral clock

//clock enable macros for the I2cx peripherals
#define I2C1_PCLK_EN() (RCC->APB1ENR|=(1<<21)) //to enable I2C1 peripheral clock
#define I2C2_PCLK_EN() (RCC->APB1ENR|=(1<<22)) //to enable I2C2 peripheral clock
#define I2C3_PCLK_EN() (RCC->APB1ENR|=(1<<23)) //to enable I2C2 peripheral clock

//clock enable macros for the SPI peripherals
#define SPI1_PCLK_EN() (RCC->APB2ENR|=(1<<12)) //to enable SPI1 peripheral clock
#define SPI2_PCLK_EN() (RCC->APB1ENR|=(1<<14)) //to enable SPI2 peripheral clock
#define SPI3_PCLK_EN() (RCC->APB1ENR|=(1<<15)) //to enable SPI3 peripheral clock
#define SPI4_PCLK_EN() (RCC->APB2ENR|=(1<<13)) //to enable SPI4 peripheral clock

//clock enable macros for USART Peripherals
#define USART1_PCLK_EN() (RCC->APB2ENR|=(1<<4)) //to enable USART1 peripheral clock
#define USART6_PCLK_EN() (RCC->APB2ENR|=(1<<5)) //to enable USART6 peripheral clock
#define USART2_PCLK_EN() (RCC->APB1ENR|=(1<<17)) //to enable USART2 peripheral clock

//clock enable macros for Sysconfig registers
#define SYSCONFIG_PCLK_EN() (RCC->APB2ENR|=(1<<14)) //to enable SYSCONFIG peripheral clock


//clock disable registers for GPIO peripherals
#define GPIOA_PCLK_DI() (RCC->AHB1ENR&=~(1<<0)) //to diable GPIOA peripheral clock
#define GPIOB_PCLK_DI() (RCC->AHB1ENR&=~(1<<1)) //to disable GPIOB peripheral clock
#define GPIOC_PCLK_DI() (RCC->AHB1ENR&=~(1<<2)) //to disable GPIOC peripheral clock
#define GPIOD_PCLK_DI() (RCC->AHB1ENR&=~(1<<3)) //to disable GPIOD peripheral clock
#define GPIOE_PCLK_DI() (RCC->AHB1ENR&=~(1<<4)) //to disable GPIOE peripheral clock
#define GPIOH_PCLK_DI() (RCC->AHB1ENR&=~(1<<5)) //to disable GPIOH peripheral clock

//clock disable registers for I2Cx peripherals
#define I2C1_PCLK_DI() (RCC->APB1ENR&=~(1<<21)) //to enable I2C1 peripheral clock
#define I2C2_PCLK_DI() (RCC->APB1ENR&=~(1<<22)) //to enable I2C2 peripheral clock
#define I2C3_PCLK_DI() (RCC->APB1ENR&=~(1<<23)) //to enable I2C2 peripheral clock

//clock disable macros for the SPI peripherals
#define SPI1_PCLK_DI() (RCC->APB2ENR&=~(1<<12)) //to disable SPI1 peripheral clock
#define SPI2_PCLK_DI() (RCC->APB1ENR&=~(1<<14)) //to disable SPI2 peripheral clock
#define SPI3_PCLK_DI() (RCC->APB1ENR&=~(1<<15)) //to disable SPI3 peripheral clock
#define SPI4_PCLK_DI() (RCC->APB2ENR&=~(1<<13)) //to disable SPI4 peripheral clock

//clock disable macros for USART Peripherals
#define USART1_PCLK_DI() (RCC->APB2ENR&=~(1<<4))  //to disable USART1 peripheral clock
#define USART6_PCLK_DI() (RCC->APB2ENR&=~(1<<5))  //to disable USART6 peripheral clock
#define USART2_PCLK_DI() (RCC->APB1ENR&=~(1<<17)) //to disable USART2 peripheral clock

//clock disable macros for Sysconfig registers
#define SYSCONFIG_PCLK_DI() (RCC->APB2ENR&=~(1<<14)) //to disable SYSCONFIG peripheral clock

//Macros to reset GPIOx peripherals
#define GPIOA_REG_RESET()  do{(RCC->AHB1RSTR|=(1<<0));  ((RCC->AHB1RSTR)&=~(1<<0));}while(0)
#define GPIOB_REG_RESET()  do{(RCC->AHB1RSTR|=(1<<1)); (RCC->AHB1RSTR&=~(1<<1));}while(0)
#define GPIOC_REG_RESET()  do{(RCC->AHB1RSTR|=(1<<2)); (RCC->AHB1RSTR&=~(1<<2));}while(0)
#define GPIOD_REG_RESET()  do{(RCC->AHB1RSTR|=(1<<3)); (RCC->AHB1RSTR&=~(1<<3));}while(0)
#define GPIOE_REG_RESET()  do{(RCC->AHB1RSTR|=(1<<4)); (RCC->AHB1RSTR&=~(1<<4));}while(0)
#define GPIOH_REG_RESET()  do{(RCC->AHB1RSTR|=(1<<7)); (RCC->AHB1RSTR&=~(1<<7));}while(0)

//enable or disable
#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET


#include "stm32f_gpio_driver.h"
#endif


