#include <sdram.h>
#include <dma.h>
#include <main.h>
#include <stm32f7xx_hal.h>
#include <stm32f7xx_hal_gpio.h>
void fmc_sdram_sendCommand(fmc_sdram_init_typedef *sdram, fmc_sdram_command_typedef *Command)
{
/*
	#define FMC_NORSRAM_TypeDef            FMC_Bank1_TypeDef
	#define FMC_NORSRAM_EXTENDED_TypeDef   FMC_Bank1E_TypeDef
	#define FMC_NAND_TypeDef               FMC_Bank3_TypeDef
	#define FMC_SDRAM_TypeDef              FMC_Bank5_6_TypeDef

	#define FMC_NORSRAM_DEVICE             FMC_Bank1
	#define FMC_NORSRAM_EXTENDED_DEVICE    FMC_Bank1E
	#define FMC_NAND_DEVICE                FMC_Bank3
	#define FMC_SDRAM_DEVICE               FMC_Bank5_6 */
  __IO uint32_t tmpr = 0;

  /* Set command register */
  tmpr = (uint32_t)((Command->CommandMode)                  |\
                    (Command->CommandTarget)                |\
                    (((Command->AutoRefreshNumber)-1) << 5) |\
                    ((Command->ModeRegisterDefinition) << 9)
                    );
  FMC_Bank5_6->SDCMR = tmpr;
}
void sdram_external_device(fmc_sdram_init_typedef *init, fmc_sdram_command_typedef *Command)
{
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  fmc_sdram_sendCommand(init, Command);

  /* Step 4: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  //HAL_Delay(1);
  dumb_delay(1);

  /* Step 5: Configure a PALL (precharge all) command */
  Command->CommandMode = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  fmc_sdram_sendCommand(init, Command);

  /* Step 6 : Configure a Auto-Refresh command */
  Command->CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 8;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  fmc_sdram_sendCommand(init, Command);

  /* Step 7: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_2           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  fmc_sdram_sendCommand(init, Command);
  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  FMC_Bank5_6->SDRTR |= ((uint32_t)((1292)<< 1));
}
void fmc_sdram_bankConfig(fmc_sdram_init_typedef *Init) {
  uint32_t tmpr1 = 0;
  uint32_t tmpr2 = 0;
  /* Set SDRAM bank configuration parameters */
  if (Init->SDBank != FMC_SDRAM_BANK2)
  {
    tmpr1 = FMC_Bank5_6->SDCR[FMC_SDRAM_BANK1];

    /* Clear NC, NR, MWID, NB, CAS, WP, SDCLK, RBURST, and RPIPE bits */
    tmpr1 &= ((uint32_t)~(FMC_SDCR1_NC  | FMC_SDCR1_NR | FMC_SDCR1_MWID | \
                          FMC_SDCR1_NB  | FMC_SDCR1_CAS | FMC_SDCR1_WP   | \
                          FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE));

    tmpr1 |= (uint32_t)(Init->ColumnBitsNumber   |\
                        Init->RowBitsNumber      |\
                        Init->MemoryDataWidth    |\
                        Init->InternalBankNumber |\
                        Init->CASLatency         |\
                        Init->WriteProtection    |\
                        Init->SDClockPeriod      |\
                        Init->ReadBurst          |\
                        Init->ReadPipeDelay
                        );
    FMC_Bank5_6->SDCR[FMC_SDRAM_BANK1] = tmpr1;
  }
  else /* FMC_Bank2_SDRAM */
  {
    tmpr1 = FMC_Bank5_6->SDCR[FMC_SDRAM_BANK1];

    /* Clear SDCLK, RBURST, and RPIPE bits */
    tmpr1 &= ((uint32_t)~(FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE));

    tmpr1 |= (uint32_t)(Init->SDClockPeriod      |\
                        Init->ReadBurst          |\
                        Init->ReadPipeDelay);

    tmpr2 = FMC_Bank5_6->SDCR[FMC_SDRAM_BANK2];

    /* Clear NC, NR, MWID, NB, CAS, WP, SDCLK, RBURST, and RPIPE bits */
    tmpr2 &= ((uint32_t)~(FMC_SDCR1_NC  | FMC_SDCR1_NR | FMC_SDCR1_MWID | \
                          FMC_SDCR1_NB  | FMC_SDCR1_CAS | FMC_SDCR1_WP   | \
                          FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST | FMC_SDCR1_RPIPE));

    tmpr2 |= (uint32_t)(Init->ColumnBitsNumber   |\
                       Init->RowBitsNumber       |\
                       Init->MemoryDataWidth     |\
                       Init->InternalBankNumber  |\
                       Init->CASLatency          |\
                       Init->WriteProtection);

    FMC_Bank5_6->SDCR[FMC_SDRAM_BANK1] = tmpr1;
    FMC_Bank5_6->SDCR[FMC_SDRAM_BANK2] = tmpr2;
  }

}
void fmc_sdram_timing_init(fmc_sdram_timing_typedef *Timing, uint32_t Bank)
{
  uint32_t tmpr1 = 0;
  uint32_t tmpr2 = 0;

  /* Set SDRAM device timing parameters */
  if (Bank != FMC_SDRAM_BANK2)
  {
    tmpr1 = FMC_Bank5_6->SDTR[FMC_SDRAM_BANK1];

    /* Clear TMRD, TXSR, TRAS, TRC, TWR, TRP and TRCD bits */
    tmpr1 &= ((uint32_t)~(FMC_SDTR1_TMRD  | FMC_SDTR1_TXSR | FMC_SDTR1_TRAS | \
                          FMC_SDTR1_TRC  | FMC_SDTR1_TWR | FMC_SDTR1_TRP | \
                          FMC_SDTR1_TRCD));

    tmpr1 |= (uint32_t)(((Timing->LoadToActiveDelay)-1)           |\
                       (((Timing->ExitSelfRefreshDelay)-1) << 4) |\
                       (((Timing->SelfRefreshTime)-1) << 8)      |\
                       (((Timing->RowCycleDelay)-1) << 12)       |\
                       (((Timing->WriteRecoveryTime)-1) <<16)    |\
                       (((Timing->RPDelay)-1) << 20)             |\
                       (((Timing->RCDDelay)-1) << 24));
    FMC_Bank5_6->SDTR[FMC_SDRAM_BANK1] = tmpr1;
  }
  else /* FMC_Bank2_SDRAM */
  {
    tmpr1 = FMC_Bank5_6->SDTR[FMC_SDRAM_BANK1];

    /* Clear TRC and TRP bits */
    tmpr1 &= ((uint32_t)~(FMC_SDTR1_TRC | FMC_SDTR1_TRP));

    tmpr1 |= (uint32_t)((((Timing->RowCycleDelay)-1) << 12)       |\
                        (((Timing->RPDelay)-1) << 20));

    tmpr2 = FMC_Bank5_6->SDTR[FMC_SDRAM_BANK2];

    /* Clear TMRD, TXSR, TRAS, TRC, TWR, TRP and TRCD bits */
    tmpr2 &= ((uint32_t)~(FMC_SDTR1_TMRD  | FMC_SDTR1_TXSR | FMC_SDTR1_TRAS | \
                          FMC_SDTR1_TRC  | FMC_SDTR1_TWR | FMC_SDTR1_TRP | \
                          FMC_SDTR1_TRCD));

    tmpr2 |= (uint32_t)(((Timing->LoadToActiveDelay)-1)           |\
                       (((Timing->ExitSelfRefreshDelay)-1) << 4)  |\
                       (((Timing->SelfRefreshTime)-1) << 8)       |\
                       (((Timing->WriteRecoveryTime)-1) <<16)     |\
                       (((Timing->RCDDelay)-1) << 24));

    FMC_Bank5_6->SDTR[FMC_SDRAM_BANK1] = tmpr1;
    FMC_Bank5_6->SDTR[FMC_SDRAM_BANK2] = tmpr2;
  }
}
void sdram_init(void) {
//void *Params)
	GPIO_INIT gpio;
	dma_init_typedef dmaInit;
	/* Enable FMC clock */
	RCC->AHB3ENR |= RCC_AHB3ENR_FMCEN;

	/* Enable chosen DMAx clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/* Enable GPIOs clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;

	/* Common GPIO configuration */
	gpio.Mode      = GPIO_MODE_AF_PP;
	gpio.Pull      = GPIO_PULLUP;
	gpio.Speed     = GPIO_SPEED_FAST;
	gpio.Alternate = GPIO_AF12_FMC;

	 //GPIOC configuration
	gpio.Pin   = GPIO_PIN_3;
	GPIO_Init(GPIOC, &gpio);
	/*
	 GPIOD configuration
	gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
							  GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
	HAL_GPIO_Init(GPIOD, &gpio_init_structure);

	 GPIOE configuration
	gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7| GPIO_PIN_8 | GPIO_PIN_9 |\
							  GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
							  GPIO_PIN_15;
	HAL_GPIO_Init(GPIOE, &gpio_init_structure);

	 GPIOF configuration
	gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3 | GPIO_PIN_4 |\
							  GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |\
							  GPIO_PIN_15;
	HAL_GPIO_Init(GPIOF, &gpio_init_structure);

	 GPIOG configuration
	gpio_init_structure.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4| GPIO_PIN_5 | GPIO_PIN_8 |\
							  GPIO_PIN_15;
	HAL_GPIO_Init(GPIOG, &gpio_init_structure);

	 GPIOH configuration
	gpio_init_structure.Pin   = GPIO_PIN_3 | GPIO_PIN_5;
	HAL_GPIO_Init(GPIOH, &gpio_init_structure); */

	/* Configure common DMA parameters */
	dmaInit.Channel             = SDRAM_DMAx_CHANNEL;
	dmaInit.Direction           = DMA_MEMORY_TO_MEMORY;
	dmaInit.PeriphInc           = DMA_PINC_ENABLE;
	dmaInit.MemInc              = DMA_MINC_ENABLE;
	dmaInit.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	dmaInit.MemDataAlignment    = DMA_MDATAALIGN_WORD;
	dmaInit.Mode                = DMA_NORMAL;
	dmaInit.Priority            = DMA_PRIORITY_HIGH;
	dmaInit.FIFOMode            = DMA_FIFOMODE_DISABLE;
	dmaInit.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	dmaInit.MemBurst            = DMA_MBURST_SINGLE;
	dmaInit.PeriphBurst         = DMA_PBURST_SINGLE;
//	/* Associate the DMA handle */
//	__HAL_LINKDMA(init, hdma, dma_init);

	/* Deinitialize the stream for new transfer */
	dma_deinit(&dmaInit);

	/* Configure the DMA stream */
	dma_init(&dmaInit);
	/* NVIC configuration for DMA transfer complete interrupt */
	NVIC_SetPriority(SDRAM_DMAx_IRQn, 0);
	NVIC_EnableIRQ(SDRAM_DMAx_IRQn);
}
