#ifndef __MAIN_H
#define __MAIN_H

#include <stm32f7xx.h>


void dumb_delay(int ms);


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


#endif
