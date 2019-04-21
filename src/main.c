#include "main.h"
#include "stm32f7xx.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "../../../Utilities/Fonts/fonts.h"
//#include "../../../Utilities/Fonts/font24.c"
#include "stm32746g_discovery_ts.h"
void SystemInit(void);
void SystemClock_Config(void);
void led_init(void);
void button_init(void);
void error_handler(void);
void external_SDRAM_init(void);
void SDRAM_MspInit(void);
void QSPI_MspInit(void);
void QSPI(void);
void mode_select(void);
void bootloader(void);
void main_image(void);
static void LTCD(void);
void DMA2D_setup(void);
void drawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code);
void drawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c);
void displayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii);
void displayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode);
void clearStringLine(uint32_t Line);
void string2array(char* input, uint8_t* output);
void USART_MspInit(void);

void touchScreen_setup(uint16_t ts_SizeX, uint16_t ts_SizeY);
void touchScreen_it_setup(void);
void touchScreen_getState(TS_StateTypeDef *TS_State);
LCD_DrawPropTypeDef DrawProp[2];
//TOUCHSCREEN
static TS_DrvTypeDef *tsDriver;
static uint16_t tsXBoundary, tsYBoundary;
static uint8_t  tsOrientation;
static uint8_t  I2cAddress;

int main(void)
{
	DrawProp[0].pFont = &Font24;
	RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
	TS_StateTypeDef tsState;
	uint16_t x, y;
	uint8_t text[30];
	uint8_t idx;
	uint8_t cleared = 0;
	uint8_t prev_nb_touches = 0;
	SCB_InvalidateICache();
	SCB_InvalidateDCache();
	SCB_EnableICache();
	SCB_EnableDCache();
	SystemClock_Config();
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
	DelayInit();
	led_init();
	button_init();
	SDRAM_MspInit();
	external_SDRAM_init();
	for(uint32_t i = 0xC0000000; i < (0xC0000000 + (480 * 272 * 4)); i += 4) *((uint32_t *)i) = 0xFFFFFFFF;
	for(uint32_t i = 0xC0000000 + (480 * 272 * 4); i < (0xC0800000); i += 4) *((uint32_t *)i) = 0xFFFFFFFF;
//	for(uint32_t i = 0; i < sizeof(shipImage); i++) {
//		*((uint8_t *)0xC0000000+i) = shipImage[i];
//	}
	LTCD();
	char string_array[] = "stanley_project -v 1  ";
	uint8_t len = sizeof(string_array);
	uint8_t arr[len];
	string2array(string_array, arr);
	displayStringAt(0, 100, (uint8_t *)&arr, CENTER_MODE);

	touchScreen_setup(480, 272);
	while(1);
	while(1)
	{
		touchScreen_getState(&tsState);
		if(tsState.touchDetected)
		{
			if(tsState.touchDetected < prev_nb_touches)
			{
				for(idx = (tsState.touchDetected + 1); idx <= 5; idx++)
				{
					clearStringLine(idx);
				}
			}
			prev_nb_touches = tsState.touchDetected;

			cleared = 0;

//			update number of presses
//			w tsState.touchDetected

			for(idx = 0; idx < tsState.touchDetected; idx++){
				x = tsState.touchX[idx];
				y = tsState.touchY[idx];

				//displayStringAt(0,0,'Number of touches = %s',LEFT_MODE);
			}
			//drawPixel(tsState.touchX[0], tsState.touchY[0]);
		}
		else{
			if(!cleared){
				//displayStringAt(0,0,'Error',LEFT_MODE);
			}
		}
	}



//	char string_array2[] = "-- debug  ";
//	len = sizeof(string_array2);
//	uint8_t arr2[len];
//	string2array(string_array2, arr2);
//	displayStringAt(100, 200, (uint8_t *)&arr2, CENTER_MODE);
	//DMA2D_setup();
	while(1) {
		GPIOI->BSRR |= GPIO_BSRR_BS_1;
		DelayMs(1000);
		GPIOI->BSRR |= GPIO_BSRR_BR_1;
		DelayMs(1000);
	}
}
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 200 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}
/*
 * --------------SDCR------------------
 * 	RPIPE -> Read pipe delay
 * 	RBURST -> Read burst
 * 	SDCLK ->
 *	WP -> Write protection
 *	CAS -> CAS latency
 *	NB -> Number of internal banks
 *	MWID -> Memory data bus width
 *	NR -> Number of row address bits
 *	NC -> Number of column address bits
 *	--------------SDTR------------------
 *	TRCD -> Row to column delay
 * 	TRP -> Row precharge delay
 * 	TWR -> Recovery delay
 * 	TRC -> Row cycle delay
 * 	TRAS -> Self refresh time
 * 	TXSR -> Exit Self-refresh delay
 * 	TMRD -> Load Mode Register to Active
 */
void external_SDRAM_init(void)
{
	volatile uint32_t tmp = 0;
	/* SDCR */
	FMC_Bank5_6->SDCR[0] &= ~(FMC_SDCR1_RPIPE_Msk |
							  FMC_SDCR1_RBURST_Msk |
							  FMC_SDCR1_SDCLK_Msk |
							  FMC_SDCR1_WP_Msk |
							  FMC_SDCR1_CAS_Msk |
							  FMC_SDCR1_NB_Msk |
							  FMC_SDCR1_MWID_Msk |
							  FMC_SDCR1_NR_Msk |
							  FMC_SDCR1_NC_Msk);
	//RPIPE, RBURST, SDCLK MUST BE BANK 0
	FMC_Bank5_6->SDCR[0] |= (0U << FMC_SDCR1_RPIPE_Pos |
							 1U << FMC_SDCR1_RBURST_Pos |
							 2U << FMC_SDCR1_SDCLK_Pos |
							 0U << FMC_SDCR1_WP_Pos |
							 2U << FMC_SDCR1_CAS_Pos |
							 1U << FMC_SDCR1_NB_Pos |
							 1U << FMC_SDCR1_MWID_Pos |
							 1U << FMC_SDCR1_NR_Pos |
							 0U << FMC_SDCR1_NC_Pos);
	/* SDTR ( ALL VALUES ARE x + 1) */
	FMC_Bank5_6->SDTR[0] &= ~(FMC_SDTR1_TRP |
							  FMC_SDTR1_TRC |
							  FMC_SDTR1_TRCD |
							  FMC_SDTR1_TWR |
							  FMC_SDTR1_TRAS |
							  FMC_SDTR1_TXSR |
							  FMC_SDTR1_TMRD);
	//TRP, TRC MUST BE BANK 0
	FMC_Bank5_6->SDTR[0] |= (1U << FMC_SDTR1_TRP_Pos |
							 6U << FMC_SDTR1_TRC_Pos |
							 1U << FMC_SDTR1_TRCD_Pos |
							 1U << FMC_SDTR1_TWR_Pos |
							 3U << FMC_SDTR1_TRAS_Pos |
							 6U << FMC_SDTR1_TXSR_Pos |
							 1U << FMC_SDTR1_TMRD_Pos);

	while(FMC_Bank5_6->SDSR & FMC_SDSR_BUSY);

	FMC_Bank5_6->SDCMR = (uint32_t)((1U << FMC_SDCMR_MODE_Pos) |
						  	  	    (1U << FMC_SDCMR_CTB1_Pos) |
									(0U << FMC_SDCMR_NRFS_Pos) |
									(0U << FMC_SDCMR_MRD_Pos)); //Clock configuration

	DelayMs(1);

	FMC_Bank5_6->SDCMR = (uint32_t)((2U << FMC_SDCMR_MODE_Pos) |
						  	  	    (1U << FMC_SDCMR_CTB1_Pos) |
									(0U << FMC_SDCMR_NRFS_Pos) |
									(0U << FMC_SDCMR_MRD_Pos)); //Clock configuration

	FMC_Bank5_6->SDCMR = (uint32_t)((3U << FMC_SDCMR_MODE_Pos) |
						  	  	    (1U << FMC_SDCMR_CTB1_Pos) |
									(7U << FMC_SDCMR_NRFS_Pos) |
									(0U << FMC_SDCMR_MRD_Pos)); //Clock configuration
	tmp = (uint32_t)(0x20);
	FMC_Bank5_6->SDCMR = (uint32_t)((4U << FMC_SDCMR_MODE_Pos) |
						  	  	    (1U << FMC_SDCMR_CTB1_Pos) |
									(0U << FMC_SDCMR_NRFS_Pos) |
									(tmp)); //Load Mode Register

	FMC_Bank5_6->SDRTR &= ~(FMC_SDRTR_COUNT_Msk);
	FMC_Bank5_6->SDRTR |= (uint32_t)(0x0603 << FMC_SDRTR_COUNT_Pos);
}

void SDRAM_MspInit(void)
{
  GPIO_InitTypeDef  GPIO_Init_Structure;

  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO clocks */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Enable FMC clock */
  __HAL_RCC_FMC_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull      = GPIO_PULLUP;
  GPIO_Init_Structure.Speed     = GPIO_SPEED_FAST;
  GPIO_Init_Structure.Alternate = GPIO_AF12_FMC;


  /* GPIOC configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);

  /* GPIOD configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
                              GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);

  /* GPIOE configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9       |\
                              GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                              GPIO_PIN_15;
  HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);

  /* GPIOF configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4      |\
                              GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
                              GPIO_PIN_15;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);

  /* GPIOG configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4| GPIO_PIN_5 | GPIO_PIN_8 |\
                              GPIO_PIN_15;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);

  /* GPIOH configuration */
  GPIO_Init_Structure.Pin   = GPIO_PIN_3 | GPIO_PIN_5;
  HAL_GPIO_Init(GPIOH, &GPIO_Init_Structure);


}

void QSPI_MspInit(void)
{
	RCC->AHB3ENR |= 0x00000002;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOB->AFR[0] = 0x0A000900;
	GPIOE->AFR[0] = 0x00000900;
	GPIOD->AFR[1] = 0x00999000;
	GPIOB->MODER = 0x000022A0;
	GPIOE->MODER = 0x00000020;
	GPIOD->MODER = 0x0A800000;
	GPIOB->OSPEEDR = 0x000030F0;
	GPIOE->OSPEEDR = 0x00000030;
	GPIOD->OSPEEDR = 0x0FC00000;
	GPIOB->PUPDR   = 0x00001100;
}
void QSPI(void)
{
	register uint32_t datareg = 0;
    QUADSPI->DCR = (QUADSPI_DCR_CSHT_0 | 23 << QUADSPI_DCR_FSIZE_Pos);

    QUADSPI->CR = (1 << QUADSPI_CR_PRESCALER_Pos |
    					QUADSPI_CR_SSHIFT |
						QUADSPI_CR_EN);

    QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE | QUADSPI_CCR_IMODE | QUADSPI_CCR_INSTRUCTION);
    QUADSPI->CCR |= (QUADSPI_CCR_FMODE_0 | QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | 0x85);
    while((QUADSPI->SR & QUADSPI_SR_TCF) == 0); //wait for complete
    datareg = QUADSPI->DR;
    QUADSPI->FCR = QUADSPI_FCR_CTCF; //clear flag

    QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE | QUADSPI_CCR_INSTRUCTION);
    QUADSPI->CCR |= 0x06;
    while((QUADSPI->SR & QUADSPI_SR_TCF) == 0); //wait for complete
    QUADSPI->FCR = QUADSPI_FCR_CTCF; //clear flag

    QUADSPI->PSMKR = 0x2;
    QUADSPI->PSMAR = 0x2;
    QUADSPI->PIR   = 0x10;
    QUADSPI->CR &= ~QUADSPI_CR_APMS;
    QUADSPI->CR |= QUADSPI_CR_APMS;

    QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE | QUADSPI_CCR_INSTRUCTION);
    QUADSPI->CCR |= (QUADSPI_CCR_FMODE_1 | QUADSPI_CCR_DMODE_0 | 0x05);
    while((QUADSPI->SR & QUADSPI_SR_SMF) == 0); //wait for complete
    QUADSPI->FCR = QUADSPI_FCR_CSMF; //clear flag

    datareg = (datareg&0xF)| 10<<4;

    QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_INSTRUCTION);
    QUADSPI->CCR |= 0x81;
    while((QUADSPI->SR & QUADSPI_SR_SMF) == 0); //wait for complete
    QUADSPI->FCR = QUADSPI_FCR_CTCF; //clear flag

    QUADSPI->DR = datareg;

    QUADSPI->CR &= ~QUADSPI_CR_ABORT;
    QUADSPI->CR |= QUADSPI_CR_ABORT;
	while((QUADSPI->SR & QUADSPI_SR_SMF) == 0);
    QUADSPI->FCR = QUADSPI_FCR_CTCF;

	QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE |
					  QUADSPI_CCR_DMODE |
					  QUADSPI_CCR_DCYC |
					  QUADSPI_CCR_ADSIZE |
					  QUADSPI_CCR_ADMODE |
					  QUADSPI_CCR_INSTRUCTION);

	QUADSPI->CCR |= (QUADSPI_CCR_FMODE |
					 QUADSPI_CCR_DMODE |
					 10 << POSITION_VAL(QUADSPI_CCR_DCYC) |
					 QUADSPI_CCR_ADSIZE_1 |
					 QUADSPI_CCR_ADMODE |
					 0xEB);
}
void led_init(void)
{
	//PI1
	__HAL_RCC_GPIOI_CLK_ENABLE();
	GPIO_InitTypeDef gpio;
	gpio.Pin = GPIO_PIN_1;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOI, &gpio);
}
void button_init(void)
{
	//PI11
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Speed = GPIO_SPEED_MEDIUM;
	gpio.Pin = GPIO_PIN_11;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOI, &gpio);
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI11_PI;
	EXTI->RTSR |= EXTI_RTSR_TR11;
	EXTI->IMR |= EXTI_IMR_MR11;
	NVIC_SetPriority(EXTI15_10_IRQn, 1);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}
void SystemInit(void)
{
  RCC->CR |= (uint32_t)0x00000001;
  RCC->CFGR = 0x00000000;
  RCC->CR &= (uint32_t)0xFEF6FFFF;
  RCC->PLLCFGR = 0x24003010;
  RCC->CR &= (uint32_t)0xFFFBFFFF;
  RCC->CIR = 0x00000000;
  //QSPI_MspInit();
  //QSPI();
}
void error_handler(void)
{
	int i = 0;
	while(1)
	{
		if (i % 2)
			GPIOI->BSRR = (uint32_t)GPIO_PIN_1 << 16;
		else
			GPIOI->BSRR = (uint32_t)GPIO_PIN_1;

		for (int j = 0; j < 30000000; j++)
		  __asm volatile("nop");
		++i;
	}
}
void EXTI15_10_IRQHandler(void)
{
	if( (EXTI->IMR & EXTI_IMR_IM11) && (EXTI->PR & EXTI_PR_PR11))
	{
		mode_select();
		while(1);
	}
}
void mode_select(void)
{
	uint32_t buttonCounter = 0;
	while((EXTI->PR & EXTI_PR_PR11) && buttonCounter < 100){
		buttonCounter++;
		HAL_Delay(100);
	}
	EXTI->PR |= EXTI_PR_PR11;
	if(buttonCounter < 100)
	{
		if(buttonCounter > 30)
		{
			bootloader();
		}
		else
		{
			main_image();
		}
	}
	main_image();
}
void bootloader(void)
{
	GPIOI->BSRR |= GPIO_BSRR_BS_1;
	while(1);
}
void main_image(void)
{
	while(1);
}
static void LTCD(void)
{
	GPIO_InitTypeDef GPIO_Init_Structure;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable the LTDC Clock */
	__HAL_RCC_LTDC_CLK_ENABLE();

	/*##-2- Configure peripheral GPIO ##########################################*/
	/******************** LTDC Pins configuration *************************/
	/* Enable GPIOs clock */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();

	/*** LTDC Pins configuration ***/
	/* GPIOE configuration */
	GPIO_Init_Structure.Pin       = GPIO_PIN_4;
	GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Pull      = GPIO_NOPULL;
	GPIO_Init_Structure.Speed     = GPIO_SPEED_FAST;
	GPIO_Init_Structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOE, &GPIO_Init_Structure);

	/* GPIOG configuration */
	GPIO_Init_Structure.Pin       = GPIO_PIN_12;
	GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Alternate = GPIO_AF9_LTDC;
	HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);

	/* GPIOI LTDC alternate configuration */
	GPIO_Init_Structure.Pin       = GPIO_PIN_9 | GPIO_PIN_10 | \
								  GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOI, &GPIO_Init_Structure);

	/* GPIOJ configuration */
	GPIO_Init_Structure.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | \
								  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | \
								  GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | \
								  GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOJ, &GPIO_Init_Structure);

	/* GPIOK configuration */
	GPIO_Init_Structure.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | \
								  GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_Init_Structure.Mode      = GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Alternate = GPIO_AF14_LTDC;
	HAL_GPIO_Init(GPIOK, &GPIO_Init_Structure);

	/* LCD_DISP GPIO configuration */
	GPIO_Init_Structure.Pin       = GPIO_PIN_12;     /* LCD_DISP pin has to be manually controlled */
	GPIO_Init_Structure.Mode      = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOI, &GPIO_Init_Structure);

	/* LCD_BL_CTRL GPIO configuration */
	GPIO_Init_Structure.Pin       = GPIO_PIN_3;  /* LCD_BL_CTRL pin has to be manually controlled */
	GPIO_Init_Structure.Mode      = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOK, &GPIO_Init_Structure);

	/* Assert display enable LCD_DISP pin */
	HAL_GPIO_WritePin(GPIOI, GPIO_PIN_12, GPIO_PIN_SET);

	/* Assert backlight LCD_BL_CTRL pin */
	HAL_GPIO_WritePin(GPIOK, GPIO_PIN_3, GPIO_PIN_SET);


	/////////////////////////////

	LTDC->GCR &= ~(LTDC_GCR_HSPOL | LTDC_GCR_VSPOL | LTDC_GCR_DEPOL | LTDC_GCR_PCPOL);
	LTDC->SSCR = 40 << LTDC_SSCR_HSW_Pos | 9 << LTDC_SSCR_VSH_Pos;
	LTDC->BPCR = (41 + 13 - 1) << LTDC_BPCR_AHBP_Pos | (10 + 2 - 1) << LTDC_BPCR_AVBP_Pos;
	LTDC->AWCR = (480 + 41 + 13 - 1) << LTDC_AWCR_AAW_Pos | (272 + 10 + 2 - 1) << LTDC_AWCR_AAH_Pos;
	LTDC->TWCR = (((480 + 41 + 13 + 32 - 1) << LTDC_TWCR_TOTALW_Pos) | (272 + 10 + 2 + 2 - 1) << LTDC_TWCR_TOTALH_Pos);
	LTDC->BCCR = (255 << LTDC_BCCR_BCRED_Pos |
				  255 << LTDC_BCCR_BCGREEN_Pos |
				  255 << LTDC_BCCR_BCBLUE_Pos); //RED,GREEN,BLUE
	LTDC->IER |= LTDC_IT_TE;
	LTDC->IER |= LTDC_IT_FU;
	LTDC->GCR |= LTDC_GCR_LTDCEN;


	/*       Layer 1      */
//	LTDC_Layer1->WHPCR = 320 << LTDC_LxWHPCR_WHSPPOS_Pos | 0 << LTDC_LxWHPCR_WHSTPOS_Pos;
	LTDC_Layer1->WHPCR &= ~(LTDC_LxWHPCR_WHSTPOS | LTDC_LxWHPCR_WHSPPOS);
	LTDC_Layer1->WHPCR = ((0 + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16) + 1) |
					 ((480 + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16)) << 16)); //X0,X1

	LTDC_Layer1->WVPCR &= ~(LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS);
	LTDC_Layer1->WVPCR = ((0 + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1) |
					  ((272 + (LTDC->BPCR & LTDC_BPCR_AVBP)) << 16)); //Y0,Y1
	LTDC_Layer1->PFCR = LTDC_PIXEL_FORMAT_ARGB8888;
	LTDC_Layer1->DCCR = 0 << LTDC_LxDCCR_DCRED_Pos |
					    0 << LTDC_LxDCCR_DCGREEN_Pos |
					  255 << LTDC_LxDCCR_DCBLUE_Pos|
					    0 << LTDC_LxDCCR_DCALPHA_Pos;
	LTDC_Layer1->CACR = 255; //ALPHA
	LTDC_Layer1->BFCR = LTDC_BLENDING_FACTOR1_PAxCA | LTDC_BLENDING_FACTOR2_PAxCA;
	LTDC_Layer1->CFBAR = (uint32_t)0xC0000000; // frame buffer start address
	LTDC_Layer1->CFBLR = (480 * 4) << LTDC_LxCFBLR_CFBP_Pos | (((480 - 0) * 4)  + 3); //image width
	LTDC_Layer1->CFBLNR = 272 << LTDC_LxCFBLNR_CFBLNBR_Pos; //image height
	LTDC_Layer1->CR |= LTDC_LxCR_LEN;
	LTDC->SRCR = LTDC_SRCR_IMR;

	/*       Layer 2      */
	LTDC_Layer2->WHPCR = ((160 + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16) + 1) |
					 ((480 + ((LTDC->BPCR & LTDC_BPCR_AHBP) >> 16)) << 16)); //X0,X1

	LTDC_Layer2->WVPCR &= ~(LTDC_LxWVPCR_WVSTPOS | LTDC_LxWVPCR_WVSPPOS);
	LTDC_Layer2->WVPCR  = ((32 + (LTDC->BPCR & LTDC_BPCR_AVBP) + 1) |
		  ((272 + (LTDC->BPCR & LTDC_BPCR_AVBP)) << 16)); //Y0,Y1

	/* Specifies the pixel format */
	LTDC_Layer2->PFCR = LTDC_PIXEL_FORMAT_RGB565;
	LTDC_Layer2->DCCR = (0 << LTDC_LxDCCR_DCRED_Pos |
					   255 << LTDC_LxDCCR_DCGREEN_Pos |
					     0 << LTDC_LxDCCR_DCBLUE_Pos |
					     0 << LTDC_LxDCCR_DCALPHA_Pos); //RED,GREEN,BLUE,ALPHA0

	LTDC_Layer2->CACR &= ~(LTDC_LxCACR_CONSTA);
	LTDC_Layer2->CACR = 200; //ALPHA

	/* Specifies the blending factors */
	LTDC_Layer2->BFCR &= ~(LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1);
	LTDC_Layer2->BFCR = (LTDC_BLENDING_FACTOR1_PAxCA | LTDC_BLENDING_FACTOR2_PAxCA);

	/* Configures the color frame buffer start address */
	LTDC_Layer2->CFBAR &= ~(LTDC_LxCFBAR_CFBADD);
	LTDC_Layer2->CFBAR = (uint32_t)0xC0000000 + (480 * 272 * 3); //BPP = 2 BUFFER HERE

	/* Configures the color frame buffer pitch in byte */
	LTDC_Layer2->CFBLR  &= ~(LTDC_LxCFBLR_CFBLL | LTDC_LxCFBLR_CFBP); //TMP = 1, corresponds to pixel format
	LTDC_Layer2->CFBLR = (320 * 2) << LTDC_LxCFBLR_CFBP_Pos  | (((320 - 0) * 2)  + 3);

	LTDC_Layer2->CFBLNR  &= ~(LTDC_LxCFBLNR_CFBLNBR);
	LTDC_Layer2->CFBLNR = 240;

	//LTDC_Layer2->CR |= (uint32_t)LTDC_LxCR_LEN;

	LTDC->SRCR = LTDC_SRCR_IMR;
}
void DMA2D_setup(void)
{
	#define LAYER_SIZE_X 			320
	#define LAYER_SIZE_Y			240
	#define LAYER_BYTE_PER_PIXEL	2
	DMA2D->CR &= ~DMA2D_CR_MODE_Msk;
	DMA2D->CR |= DMA2D_M2M_BLEND;
	DMA2D->OPFCCR = DMA2D_OUTPUT_RGB565;
	DMA2D->OOR = 0x0;
	/*      Layer1           */

	DMA2D->BGPFCCR &= ~(DMA2D_BGPFCCR_CM_Msk | DMA2D_BGPFCCR_AM_Msk | DMA2D_BGPFCCR_ALPHA_Msk);
	DMA2D->BGPFCCR |= DMA2D_INPUT_RGB565 << DMA2D_BGPFCCR_CM_Pos |
					  DMA2D_REPLACE_ALPHA << DMA2D_BGPFCCR_AM_Pos |
					  0x7F << DMA2D_BGPFCCR_ALPHA_Pos;
    DMA2D->BGOR = 0x0;


    /*      Layer2          */
	DMA2D->FGPFCCR &= ~(DMA2D_BGPFCCR_CM_Msk | DMA2D_BGPFCCR_AM_Msk | DMA2D_BGPFCCR_ALPHA_Msk);
	DMA2D->FGPFCCR |= DMA2D_INPUT_RGB565 << DMA2D_BGPFCCR_CM_Pos |
					  DMA2D_REPLACE_ALPHA << DMA2D_BGPFCCR_AM_Pos |
					  0x7F << DMA2D_BGPFCCR_ALPHA_Pos;
    DMA2D->FGOR = 0x0;

    uint32_t aBlendedImage[(320 * 240 * 2) / 4];

	DMA2D->BGMAR = (uint32_t)0xC0000000;
	DMA2D->NLR &= ~(DMA2D_NLR_NL_Msk | DMA2D_NLR_PL_Msk);
	DMA2D->NLR = (0 << DMA2D_NLR_NL_Pos | 0 << DMA2D_NLR_PL_Pos);
	DMA2D->OMAR = (uint32_t)aBlendedImage;
	DMA2D->FGMAR = (uint32_t)0xC0000000;
	DMA2D->CR |= DMA2D_IT_TC | DMA2D_IT_TE | DMA2D_IT_CE | DMA2D_CR_START;
	DMA2D->CR |= DMA2D_CR_START;
}
void drawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code)
{
  /* Write data value to all SDRAM memory */
  /* RGB565 format */
    *(__IO uint32_t*) (0xC0000000 + (4*(Ypos*480 + Xpos))) = RGB_Code;
}
void drawChar(uint16_t Xpos, uint16_t Ypos, const uint8_t *c)
{
  uint32_t i = 0, j = 0;
  uint16_t height, width;
  uint8_t  offset;
  uint8_t  *pchar;
  uint32_t line;

  height = 24;
  width  = 17;

  offset =  8 *((width + 7)/8) -  width ;

  for(i = 0; i < height; i++)
  {
    pchar = ((uint8_t *)c + (width + 7)/8 * i);

    switch(((width + 7)/8))
    {

    case 1:
      line =  pchar[0];
      break;

    case 2:
      line =  (pchar[0]<< 8) | pchar[1];
      break;

    case 3:
    default:
      line =  (pchar[0]<< 16) | (pchar[1]<< 8) | pchar[2];
      break;
    }

    for (j = 0; j < width; j++)
    {
      if(line & (1 << (width- j + offset- 1)))
      {
    	  drawPixel((Xpos + j), Ypos, 0xFF000000);
      }
      else
      {
    	  drawPixel((Xpos + j), Ypos, 0xFFFFFFFF);
      }
    }
    Ypos++;
  }
}
void displayChar(uint16_t Xpos, uint16_t Ypos, uint8_t Ascii){

	  drawChar(Xpos, Ypos, &DrawProp[0].pFont->table[(Ascii-' ') *\
	    DrawProp[0].pFont->Height * ((DrawProp[0].pFont->Width + 7) / 8)]);
}
void displayStringAt(uint16_t Xpos, uint16_t Ypos, uint8_t *Text, Text_AlignModeTypdef Mode)
{
  uint16_t ref_column = 1, i = 0;
  uint32_t size = 0, xsize = 0;
  uint8_t  *ptr = Text;

  /* Get the text size */
  while (*ptr++) size ++ ;

  /* Characters number per line */
  xsize = (480/DrawProp[0].pFont->Width);

  switch (Mode)
  {
  case CENTER_MODE:
    {
      ref_column = Xpos + ((xsize - size)* DrawProp[0].pFont->Width) / 2;
      break;
    }
  case LEFT_MODE:
    {
      ref_column = Xpos;
      break;
    }
  case RIGHT_MODE:
    {
      ref_column = - Xpos + ((xsize - size)*DrawProp[0].pFont->Width);
      break;
    }
  default:
    {
      ref_column = Xpos;
      break;
    }
  }

  /* Check that the Start column is located in the screen */
  if ((ref_column < 1) || (ref_column >= 0x8000))
  {
    ref_column = 1;
  }

  /* Send the string character by character on LCD */
  while ((*Text != 0) & (((480 - (i*DrawProp[0].pFont->Width)) & 0xFFFF) >= DrawProp[0].pFont->Width))
  {
    /* Display one character on LCD */
    displayChar(ref_column, Ypos, *Text);
    /* Decrement the column position by 16 */
    ref_column += DrawProp[0].pFont->Width;
    /* Point on the next character */
    Text++;
    i++;
  }
}
void clearStringLine(uint32_t Line)
{
  uint32_t color_backup = DrawProp[0].TextColor;
  DrawProp[0].TextColor = DrawProp[0].BackColor;

  /* Draw rectangle with background color */
  BSP_LCD_FillRect(0, (Line * DrawProp[0].pFont->Height), 480, DrawProp[0].pFont->Height);

  DrawProp[0].TextColor = color_backup;
  BSP_LCD_SetTextColor(DrawProp[0].TextColor);
}
void string2array(char* input, uint8_t* output){
	int loop = 0;
	int i = 0;
	while(input[loop] != '\0')
	{
		output[i++] = input[loop++];
	}

}
/*	PA9 USART1_TX
 * 	PB7 USART1_RX
 *
 *
 */
void USART_MspInit(void) {
	GPIO_InitTypeDef  GPIO_InitStruct;
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_APB2ENR_USART1EN);

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = GPIO_PIN_9;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*##-3- Configure the NVIC for UART ########################################*/
	/* NVIC for USART */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}
void touchScreen_setup(uint16_t ts_SizeX, uint16_t ts_SizeY)
{
	  tsXBoundary = ts_SizeX;
	  tsYBoundary = ts_SizeY;

	  /* Read ID and verify if the touch screen driver is ready */
	  ft5336_ts_drv.Init(TS_I2C_ADDRESS);
	  if(ft5336_ts_drv.ReadID(TS_I2C_ADDRESS) == FT5336_ID_VALUE)
	  {
	    /* Initialize the TS driver structure */
	    tsDriver = &ft5336_ts_drv;
	    I2cAddress = TS_I2C_ADDRESS;
	    tsOrientation = TS_SWAP_XY;

	    /* Initialize the TS driver */
	    tsDriver->Start(I2cAddress);
	  }
	  else
	  {
	    while(1);
	  }
}
void touchScreen_it_setup(void)
{
  GPIO_InitTypeDef gpio_init_structure;

  /* Configure Interrupt mode for SD detection pin */
  gpio_init_structure.Pin = TS_INT_PIN;
  gpio_init_structure.Pull = GPIO_NOPULL;
  gpio_init_structure.Speed = GPIO_SPEED_FAST;
  gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
  HAL_GPIO_Init(TS_INT_GPIO_PORT, &gpio_init_structure);

  /* Enable and set Touch screen EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type)(TS_INT_EXTI_IRQn), 0x0F, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type)(TS_INT_EXTI_IRQn));

  /* Enable the TS ITs */
  tsDriver->EnableIT(I2cAddress);

}
void touchScreen_getState(TS_StateTypeDef *TS_State)
{
  static uint32_t _x[TS_MAX_NB_TOUCH] = {0, 0};
  static uint32_t _y[TS_MAX_NB_TOUCH] = {0, 0};
  uint16_t x[TS_MAX_NB_TOUCH];
  uint16_t y[TS_MAX_NB_TOUCH];
  uint16_t brute_x[TS_MAX_NB_TOUCH];
  uint16_t brute_y[TS_MAX_NB_TOUCH];
  uint16_t x_diff;
  uint16_t y_diff;
  uint32_t index;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  uint32_t weight = 0;
  uint32_t area = 0;
  uint32_t event = 0;
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  /* Check and update the number of touches active detected */
  TS_State->touchDetected = tsDriver->DetectTouch(I2cAddress);

  if(TS_State->touchDetected)
  {
    for(index=0; index < TS_State->touchDetected; index++)
    {
      /* Get each touch coordinates */
      tsDriver->GetXY(I2cAddress, &(brute_x[index]), &(brute_y[index]));

      if(tsOrientation == TS_SWAP_NONE)
      {
        x[index] = brute_x[index];
        y[index] = brute_y[index];
      }

      if(tsOrientation & TS_SWAP_X)
      {
        x[index] = 4096 - brute_x[index];
      }

      if(tsOrientation & TS_SWAP_Y)
      {
        y[index] = 4096 - brute_y[index];
      }

      if(tsOrientation & TS_SWAP_XY)
      {
        y[index] = brute_x[index];
        x[index] = brute_y[index];
      }

      x_diff = x[index] > _x[index]? (x[index] - _x[index]): (_x[index] - x[index]);
      y_diff = y[index] > _y[index]? (y[index] - _y[index]): (_y[index] - y[index]);

      if ((x_diff + y_diff) > 5)
      {
        _x[index] = x[index];
        _y[index] = y[index];
      }

      if(I2cAddress == FT5336_I2C_SLAVE_ADDRESS)
      {
        TS_State->touchX[index] = x[index];
        TS_State->touchY[index] = y[index];
      }
      else
      {
        /* 2^12 = 4096 : indexes are expressed on a dynamic of 4096 */
        TS_State->touchX[index] = (tsXBoundary * _x[index]) >> 12;
        TS_State->touchY[index] = (tsYBoundary * _y[index]) >> 12;
      }

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

      /* Get touch info related to the current touch */
      ft5336_TS_GetTouchInfo(I2cAddress, index, &weight, &area, &event);

      /* Update TS_State structure */
      TS_State->touchWeight[index] = weight;
      TS_State->touchArea[index]   = area;

      /* Remap touch event */
      switch(event)
      {
        case FT5336_TOUCH_EVT_FLAG_PRESS_DOWN	:
          TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT5336_TOUCH_EVT_FLAG_LIFT_UP :
          TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT5336_TOUCH_EVT_FLAG_CONTACT :
          TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT5336_TOUCH_EVT_FLAG_NO_EVENT :
          TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
        	while(1);
          break;
      } /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

    } /* of for(index=0; index < TS_State->touchDetected; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    /* Get gesture Id */
    BSP_TS_Get_GestureId(TS_State);
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  } /* end of if(TS_State->touchDetected != 0) */

}

void printStringTo (uint16_t Xpos, uint16_t Ypos, char* string, uint16_t nameOfBuffer)
{
	uint8_t nameOfBuffer[0];
}
