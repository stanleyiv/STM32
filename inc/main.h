#ifndef __MAIN_H
#define __MAIN_H

#include <stm32f7xx.h>
#include <stm32f7xx_hal_gpio.h>
#include <stm32f746xx.h>

typedef struct {
	uint32_t Mode;
	uint32_t Pull;
	uint32_t Speed;
	uint32_t Alternate;
	uint32_t Pin;
}GPIO_INIT;

void dumb_delay(int ms);
void GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_INIT *gpio);

typedef struct {
	uint32_t PLLSOURCE;
	uint32_t PLLM;
	uint32_t PLLN;
	uint32_t PLLP;
	uint32_t PLLQ;
}pll_config;

typedef struct {
	uint32_t PLLSAIN;
	uint32_t PLLSAIP;
	uint32_t PLLSAIQ;
	uint32_t PLLSAIR;
}pll_sai_config;

#define GPIO_MODE             ((uint32_t)0x00000003U)
#define EXTI_MODE             ((uint32_t)0x10000000U)
#define GPIO_MODE_IT          ((uint32_t)0x00010000U)
#define GPIO_MODE_EVT         ((uint32_t)0x00020000U)
#define RISING_EDGE           ((uint32_t)0x00100000U)
#define FALLING_EDGE          ((uint32_t)0x00200000U)
#define GPIO_OUTPUT_TYPE      ((uint32_t)0x00000010U)

#define GPIO_NUMBER           ((uint32_t)16U)

#endif
