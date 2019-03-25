//#include <stm32f7xx.h>
#include "main.h"
#include <dma.h>
#include <flash.h>
#include <qspi.h>
#include <sdram.h>

void clock_reset(void);
void clock_setup(void);
void clock_LTDC(void);
void led_init(void);
void led_toggle(int ms);
static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLength, uint32_t uwOffset);

#define BUFFER_SIZE         ((uint32_t)0x1000)
#define WRITE_READ_ADDR     ((uint32_t)0x0800)
#define DUMMY_CLOCK_CYCLES_READ_QUAD         10
/* Size of the flash */
#define QSPI_FLASH_SIZE                      23
#define QSPI_PAGE_SIZE                       256
/* End address of the QSPI memory */
#define QSPI_END_ADDR              (1 << QSPI_FLASH_SIZE)
/* Read/Write Buffers */
uint32_t aTxBuffer[BUFFER_SIZE];
uint32_t aRxBuffer[BUFFER_SIZE];

pll_config pll;
pll_sai_config pllsai;
qspi_command_typedef qspi_cmd;
qspi_init_typedef qspiInit;
fmc_sdram_timing_typedef sdram_timing;
fmc_sdram_init_typedef sdramInit;
fmc_sdram_command_typedef sdram_command;

volatile uint8_t CmdCplt, RxCplt, TxCplt, StatusMatch, TimeOut;
volatile uint8_t step = 0;
uint32_t INDEX = 0;
uint32_t sdram_status = 0;
uint32_t address = 0;

#define BUFFERSIZE                 (COUNTOF(aTxBuffer) - 1)
#define COUNTOF(__BUFFER__)        (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

int main(void) {
	SCB_InvalidateICache();
	SCB_InvalidateDCache();
	SCB_EnableICache();
	SCB_EnableDCache();
	FLASH->ACR |= FLASH_ACR_ARTEN;
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	clock_reset();
	clock_setup();
	led_init();
	sdram_timing.LoadToActiveDelay    = 2;
	sdram_timing.ExitSelfRefreshDelay = 6;
	sdram_timing.SelfRefreshTime      = 4;
	sdram_timing.RowCycleDelay        = 6;
	sdram_timing.WriteRecoveryTime    = 2;
	sdram_timing.RPDelay              = 2;
	sdram_timing.RCDDelay             = 2;
	sdramInit.SDBank             = FMC_SDRAM_BANK1;
	sdramInit.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
	sdramInit.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;
	sdramInit.MemoryDataWidth    = SDRAM_MEMORY_WIDTH;
	sdramInit.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	sdramInit.CASLatency         = FMC_SDRAM_CAS_LATENCY_2;
	sdramInit.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	sdramInit.SDClockPeriod      = SDCLOCK_PERIOD;
	sdramInit.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
	sdramInit.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;
	/* Setup clocks, gpios, and dma */
	sdram_init();
	/* Initialize SDRAM control Interface */
	fmc_sdram_bankConfig(&sdramInit);
	/* Initialize SDRAM timing Interface */
	fmc_sdram_timing_init(&sdram_timing, sdramInit.SDBank);
	/* Program the SDRAM external device */
	sdram_external_device(&sdramInit, &sdram_command);
	/*##-2- SDRAM memory read/write access #####################################*/
	/* Fill the buffer to write */
	Fill_Buffer(aTxBuffer, BUFFER_SIZE, 0xA244250F);
	/* Write data to the SDRAM memory */
	for (INDEX = 0; INDEX < BUFFER_SIZE; INDEX++) {
		*(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*INDEX) = aTxBuffer[INDEX];
	}
	/* Read back data from the SDRAM memory */
	for (INDEX = 0; INDEX < BUFFER_SIZE; INDEX++) {
		aRxBuffer[INDEX] = *(__IO uint32_t*) (SDRAM_BANK_ADDR + WRITE_READ_ADDR + 4*INDEX);
	}
	  /*##-3- Checking data integrity ############################################*/
	for (INDEX = 0; (INDEX < BUFFER_SIZE) && (sdram_status == 0); INDEX++)
	{
		if (aRxBuffer[INDEX] != aTxBuffer[INDEX]) {
			sdram_status++;
		}

		if (sdram_status > 0) {
			while(1) {
				led_toggle(100);
			}
		}
		else {
			GPIOI->BSRR |= GPIO_BSRR_BS_1;
		}
	}
//	qspi_deinit();
//	qspiInit.ClockPrescaler     = 2;
//	qspiInit.FifoThreshold      = 4;
//	qspiInit.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
//	qspiInit.FlashSize          = POSITION_VAL(0x1000000) - 1;
//	qspiInit.ChipSelectHighTime = QSPI_CS_HIGH_TIME_2_CYCLE;
//	qspiInit.ClockMode          = QSPI_CLOCK_MODE_0;
//	qspiInit.FlashID            = QSPI_FLASH_ID_1;
//	qspiInit.DualFlash          = QSPI_DUALFLASH_DISABLE;
//	qspi_init();
//	qspi_cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
//	qspi_cmd.AddressSize       = QSPI_ADDRESS_24_BITS;
//	qspi_cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	qspi_cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
//	qspi_cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
//	qspi_cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
//
//	switch(step) {
//	  case 0:
//		CmdCplt = 0;
//		/* Initialize Reception buffer --------------------------------------- */
//		for (INDEX = 0; INDEX < BUFFERSIZE; INDEX++) {
//		  aRxBuffer[INDEX] = 0;
//		}
//		/* Enable write operations ------------------------------------------- */
//		QSPI_WriteEnable(&QSPIHandle);
//		/* Erasing Sequence -------------------------------------------------- */
//		qspi_cmd.Instruction = SECTOR_ERASE_CMD;
//		qspi_cmd.AddressMode = QSPI_ADDRESS_1_LINE;
//		qspi_cmd.Address     = address;
//		qspi_cmd.DataMode    = QSPI_DATA_NONE;
//		qspi_cmd.DummyCycles = 0;
//		qspi_command(&qspi_cmd);
//		step++;
//		break;
//
//	  case 1:
//		if(CmdCplt != 0)
//		{
//		  CmdCplt = 0;
//		  StatusMatch = 0;
//		  /* Configure automatic polling mode to wait for end of erase ------- */
//		  QSPI_AutoPollingMemReady(&QSPIHandle);
//		  step++;
//		}
//		break;
//	  case 2:
//		if(StatusMatch != 0)
//		{
//		  StatusMatch = 0;
//		  TxCplt = 0;
//		  /* Enable write operations ----------------------------------------- */
//		  QSPI_WriteEnable(&QSPIHandle);
//		  /* Writing Sequence ------------------------------------------------ */
//		  qspi_cmd.Instruction = QUAD_IN_FAST_PROG_CMD;
//		  qspi_cmd.AddressMode = QSPI_ADDRESS_1_LINE;
//		  qspi_cmd.DataMode    = QSPI_DATA_4_LINES;
//		  qspi_cmd.NbData      = BUFFERSIZE;
//		  qspi_command(&qspi_cmd);
//		  HAL_QSPI_Transmit_IT(&QSPIHandle, aTxBuffer);
//		  step++;
//		}
//		break;
//
//	  case 3:
//		if(TxCplt != 0)
//		{
//		  TxCplt = 0;
//		  StatusMatch = 0;
//		  /* Configure automatic polling mode to wait for end of program ----- */
//		  QSPI_AutoPollingMemReady(&QSPIHandle);
//		  step++;
//		}
//		break;
//	  case 4:
//		if(StatusMatch != 0)
//		{
//		  StatusMatch = 0;
//		  RxCplt = 0;
//		  /* Configure Volatile Configuration register (with new dummy cycles) */
//		  QSPI_DummyCyclesCfg(&QSPIHandle);
//		  /* Reading Sequence ------------------------------------------------ */
//		  qspi_cmd.Instruction = QUAD_OUT_FAST_READ_CMD;
//		  qspi_cmd.DummyCycles = DUMMY_CLOCK_CYCLES_READ_QUAD;
//		  qspi_command(&qspi_cmd);
//		  HAL_QSPI_Receive_IT(&QSPIHandle, aRxBuffer);
//		  step++;
//		}
//		break;
//	  case 5:
//		if (RxCplt != 0)
//		{
//		  RxCplt = 0;
//		  /* Result comparison ----------------------------------------------- */
//		  for (INDEX = 0; INDEX < BUFFERSIZE; INDEX++)
//		  {
//			if (aRxBuffer[INDEX] != aTxBuffer[INDEX])
//			{
//				led_toggle(100);;
//			}
//		  }
//		  address += QSPI_PAGE_SIZE;
//		  if(address >= QSPI_END_ADDR)
//		  {
//			address = 0;
//		  }
//		  step = 0;
//		}
//		break;
//
//	  default :
//		  led_toggle(100);
//	}
	clock_LTDC();
	while(1) {

	}
}




void clock_reset(void) {
	RCC->CR |= RCC_CR_HSION;
	while(!(RCC->CR & RCC_CR_HSIRDY)) {}
	RCC->CR |= RCC_CR_HSITRIM_4;
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_CSSON);
	while(RCC->CR & RCC_CR_HSERDY) {}
	RCC->CR &= RCC_CR_PLLON;
	while(RCC->CR & RCC_CR_PLLON) {}
	RCC->CR &= ~(RCC_CR_PLLI2SON);
	while(RCC->CR & RCC_CR_PLLI2SRDY) {}
	RCC->CR &= ~(RCC_CR_PLLSAION);
	while(RCC->CR & RCC_CR_PLLSAIRDY) {}
	RCC->PLLCFGR = RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2 | 0x20000000U;
	RCC->PLLI2SCFGR = RCC_PLLI2SCFGR_PLLI2SN_6 | RCC_PLLI2SCFGR_PLLI2SN_7 | RCC_PLLI2SCFGR_PLLI2SQ_2 | RCC_PLLI2SCFGR_PLLI2SR_1;
	RCC->PLLSAICFGR = RCC_PLLSAICFGR_PLLSAIN_6 | RCC_PLLSAICFGR_PLLSAIN_7 | RCC_PLLSAICFGR_PLLSAIQ_2 | 0x20000000U;
	RCC->CIR &= ~(RCC_CIR_LSIRDYIE | RCC_CIR_LSERDYIE | RCC_CIR_HSIRDYIE | RCC_CIR_HSERDYIE | RCC_CIR_PLLRDYIE | RCC_CIR_PLLI2SRDYIE | RCC_CIR_PLLSAIRDYIE);
	RCC->CIR |= (RCC_CIR_LSIRDYC | RCC_CIR_LSERDYC | RCC_CIR_HSIRDYC | RCC_CIR_HSERDYC | RCC_CIR_PLLRDYC | RCC_CIR_PLLI2SRDYC | RCC_CIR_PLLSAIRDYC | RCC_CIR_CSSC);
	RCC->CSR &= RCC_CSR_LSION;
	RCC->CSR |= RCC_CSR_RMVF;
}
void clock_setup(void) {
	pll.PLLSOURCE = 1; //1 for hse, 0 for hsi
	pll.PLLM = 25;
	pll.PLLN = 400;
	pll.PLLP = 0;
	pll.PLLQ = 0;
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY)) {}
	RCC->CR &= ~(RCC_CR_PLLON);
	while(!(RCC->CR & RCC_CR_PLLON)) {}
	RCC->PLLCFGR = (0x20000000 | (pll.PLLSOURCE << RCC_PLLCFGR_PLLSRC_Pos) | (pll.PLLM) |
			(pll.PLLN << RCC_PLLCFGR_PLLN_Pos) | (pll.PLLP << RCC_PLLCFGR_PLLP_Pos) | (pll.PLLQ << RCC_PLLCFGR_PLLQ_Pos));
	RCC->CR |= RCC_CR_PLLON;
	PWR->CR1 |= PWR_CR1_ODEN;
	while(!(PWR->CSR1 & PWR_CSR1_ODRDY)) {}
	PWR->CR1 |= PWR_CR1_ODSWEN;
	while(!(PWR->CSR1 & PWR_CSR1_ODSWRDY)) {}
	RCC->CFGR &= (~(0xF) << RCC_CFGR_HPRE_Pos);
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	//RCC->CFGR |= RCC_CFGR_HPRE;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	RCC->CR |= RCC_CR_CSSON;
}
void clock_LTDC(void) {
	pllsai.PLLSAIN = 0;
	pllsai.PLLSAIP = 0;
	pllsai.PLLSAIQ = 0;
	pllsai.PLLSAIR = 0;
	RCC->CR &= ~(RCC_CR_PLLSAION);
	while(RCC->CR & RCC_CR_PLLSAION) {}
	RCC->PLLSAICFGR = ((pllsai.PLLSAIN << RCC_PLLSAICFGR_PLLSAIN_Pos) | (pllsai.PLLSAIP << RCC_PLLSAICFGR_PLLSAIP_Pos) | (pllsai.PLLSAIQ << RCC_PLLSAICFGR_PLLSAIQ_Pos) |
		(pllsai.PLLSAIR << RCC_PLLSAICFGR_PLLSAIR_Pos));
	RCC->DCKCFGR1 |= RCC_DCKCFGR1_PLLSAIDIVR; //DIVR
	RCC->CR |= RCC_CR_PLLSAION;
	while(!(RCC->CR & RCC_CR_PLLSAIRDY)) {}
}
void led_init(void) {
	//PI1
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
	GPIOI->MODER &= ~(GPIO_MODER_MODER1_Msk);
	GPIOI->MODER |= (1U << GPIO_MODER_MODER1_Pos);
	GPIOI->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR1_Msk);
	GPIOI->OSPEEDR |= (2U << GPIO_OSPEEDER_OSPEEDR1_Pos);
	GPIOI->PUPDR &= ~(GPIO_MODER_MODER1_Msk);
	GPIOI->PUPDR |= (0U << GPIO_MODER_MODER1_Pos);
}
void led_toggle(int ms) {
	GPIOI->BSRR |= GPIO_BSRR_BS_1;
	int j=0;
	for(int i=0;i<ms;i++)
	{
		j++;
	}
	GPIOI->BSRR |= GPIO_BSRR_BR_1;
	for(int i=0;i<ms;i++)
	{
		j++;
	}
}
void dumb_delay(int ms) {
	int i, j = 0;
	for(i=0;i<ms;i++) {
		j++;
	}
}
/**
  * @brief  Fills buffer with user predefined data.
  * @param  pBuffer: pointer on the buffer to fill
  * @param  uwBufferLenght: size of the buffer to fill
  * @param  uwOffset: first value to fill on the buffer
  * @retval None
  */
static void Fill_Buffer(uint32_t *pBuffer, uint32_t uwBufferLength, uint32_t uwOffset)
{
  uint32_t tmpIndex = 0;

  /* Put in global buffer different values */
  for (tmpIndex = 0; tmpIndex < uwBufferLength; tmpIndex++ )
  {
    pBuffer[tmpIndex] = tmpIndex + uwOffset;
  }
}
