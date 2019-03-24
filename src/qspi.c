//#include <qspi.h>
////#include <stm32f746xx.h>
//
//
//void qspi_setup(void) {
//	register uint32_t datareg = 0;
//	while(QUADSPI->SR & QUADSPI_SR_BUSY) {}
//	QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE | QUADSPI_CCR_IMODE | QUADSPI_CCR_INSTRUCTION);
//	QUADSPI->CCR |= (QUADSPI_CCR_FMODE_0 | QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | 0x85);
//	while(!(QUADSPI->SR & QUADSPI_SR_TCF)) {}
//	datareg = QUADSPI->DR;
//	QUADSPI->FCR = QUADSPI_FCR_CTCF;
//	while(!(QUADSPI->SR & QUADSPI_SR_TCF)) {}
//	QUADSPI->FCR = QUADSPI_FCR_CTCF;
//	QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE | QUADSPI_CCR_INSTRUCTION);
//	QUADSPI->CCR |= 0x06;
//	while(!(QUADSPI->SR & QUADSPI_SR_TCF)) {}
//	QUADSPI->FCR = QUADSPI_FCR_CTCF;
//
//	QUADSPI->PSMAR = 0x2;
//	QUADSPI->PIR   = 0x10;
//    QUADSPI->CR = (QUADSPI->CR&(~QUADSPI_CR_APMS));
//    QUADSPI->CR |= QUADSPI_CR_APMS;
//    QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE | QUADSPI_CCR_INSTRUCTION);
//    QUADSPI->CCR |= (QUADSPI_CCR_FMODE_1 | QUADSPI_CCR_DMODE_0 | 0x05);
//    while(!(QUADSPI->SR & QUADSPI_SR_SMF)) {}
//    QUADSPI->FCR = QUADSPI_FCR_CSMF;
//    datareg = (datareg&0xF)| 10<<4;
//    QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_INSTRUCTION);
//    QUADSPI->CCR |= 0x81;
//    QUADSPI->DR = datareg;
//    while(!(QUADSPI->SR & QUADSPI_SR_TCF)) {}
//    QUADSPI->FCR = QUADSPI_FCR_CTCF;
//    QUADSPI->CR &= ~(QUADSPI_CR_ABORT);
//    QUADSPI->CR |= QUADSPI_CR_ABORT;
//    while(!(QUADSPI->SR & QUADSPI_SR_TCF)) {}
//    QUADSPI->FCR = QUADSPI_FCR_CTCF;
//    QUADSPI->CCR &= ~(QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE | QUADSPI_CCR_DCYC | QUADSPI_CCR_ADSIZE | QUADSPI_CCR_ADMODE | QUADSPI_CCR_INSTRUCTION);
//    QUADSPI->CCR |= (QUADSPI_CCR_FMODE | QUADSPI_CCR_DMODE |  (10 << POSITION_VAL(QUADSPI_CCR_DCYC)) | QUADSPI_CCR_ADSIZE_1 | QUADSPI_CCR_ADMODE | 0xEB);
//}
//void qspi_init(void) {
//	//gpio and clocks
//	RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
//	GPIOB->AFR[0] = 0x0A000900;
//	GPIOE->AFR[0] = 0x00000900;
//	GPIOD->AFR[1] = 0x00999000;
//	GPIOB->MODER = 0x000022A0;
//	GPIOE->MODER = 0x00000020;
//	GPIOD->MODER = 0x0A800000;
//	GPIOB->OSPEEDR = 0x000030F0;
//	GPIOE->OSPEEDR = 0x00000030;
//	GPIOD->OSPEEDR = 0x0FC00000;
//	GPIOB->PUPDR   = 0x00001100;
//	//nvic
//	NVIC_SetPriority(QUADSPI_IRQn, 0);
//	NVIC_EnableIRQ(QUADSPI_IRQn);
//	/* Configure QSPI FIFO Threshold */
//	QUADSPI->CR &= ~(QUADSPI_CR_FTHRES_Msk);
//	QUADSPI->CR |= 0x3 << QUADSPI_CR_FTHRES_Pos;
//	while(QUADSPI->SR & QUADSPI_SR_BUSY) {}
//	/* Configure QSPI Clock Prescaler and Sample Shift */
//	QUADSPI->CR &= ~(QUADSPI_CR_PRESCALER_Msk | QUADSPI_CR_SSHIFT_Msk | QUADSPI_CR_FSEL_Msk | QUADSPI_CR_DFM_Msk);
//	QUADSPI->CR |= 1U << QUADSPI_CR_PRESCALER_Pos;
//	QUADSPI->CR |= 1U << QUADSPI_CR_SSHIFT_Pos;
//	QUADSPI->CR |= 0U << QUADSPI_CR_FSEL_Pos;
//	QUADSPI->CR |= 0U << QUADSPI_CR_DFM_Pos;
//	/* Configure QSPI Flash Size, CS High Time and Clock Mode */
//	QUADSPI->DCR &= (QUADSPI_DCR_FSIZE_Msk | QUADSPI_DCR_CSHT_Msk | QUADSPI_DCR_CKMODE_Msk);
//	QUADSPI->DCR |= 23U << QUADSPI_DCR_FSIZE_Pos;
//	QUADSPI->DCR |= 5U << QUADSPI_DCR_CSHT_Pos;
//	QUADSPI->DCR |= 0U << QUADSPI_DCR_CKMODE_Pos;
//    /* Enable the QSPI peripheral */
//	QUADSPI->CR = ~(QUADSPI_CR_PRESCALER_Msk);
//	QUADSPI->CR |= QUADSPI_CR_SSHIFT | QUADSPI_CR_EN;
//}
//void qspi_deinit(void) {
//	/* Disable the NVIC for QSPI */
//	NVIC_DisableIRQ(QUADSPI_IRQn);
//	/* De-Configure QSPI pins */
//	GPIOB->MODER &= ~(GPIO_MODER_MODER6_Msk);
//	GPIOB->AFR[0] &= ~((uint32_t)0xF << GPIO_AFRL_AFRL6_Pos);
//	GPIOB->OSPEEDR &= ~(3U << GPIO_OSPEEDER_OSPEEDR6_Pos);
//	GPIOB->OTYPER  &= ~(1U << 6U) ;
//	GPIOB->PUPDR &= ~(3U << GPIO_PUPDR_PUPDR6_Pos);
//	GPIOB->MODER &= ~(GPIO_MODER_MODER2_Msk);
//	GPIOB->AFR[0] &= ~((uint32_t)0xF << GPIO_AFRL_AFRL2_Pos);
//	GPIOB->OSPEEDR &= ~(3U << GPIO_OSPEEDER_OSPEEDR2_Pos);
//	GPIOB->OTYPER  &= ~(1U << 2U) ;
//	GPIOB->PUPDR &= ~(3U << GPIO_PUPDR_PUPDR2_Pos);
//	GPIOD->MODER &= ~(GPIO_MODER_MODER11_Msk);
//	GPIOD->AFR[1] &= ~((uint32_t)0xF << GPIO_AFRH_AFRH3_Pos);
//	GPIOD->OSPEEDR &= ~(3U << GPIO_OSPEEDER_OSPEEDR11_Pos);
//	GPIOD->OTYPER  &= ~(1U << 11U) ;
//	GPIOD->PUPDR &= ~(3U << GPIO_PUPDR_PUPDR11_Pos);
//	GPIOD->MODER &= ~(GPIO_MODER_MODER12_Msk);
//	GPIOD->AFR[1] &= ~((uint32_t)0xF << GPIO_AFRH_AFRH4_Pos) ;
//	GPIOD->OSPEEDR &= ~(3U << GPIO_OSPEEDER_OSPEEDR12_Pos);
//	GPIOD->OTYPER  &= ~(1U << 12U);
//	GPIOD->PUPDR &= ~(3U << GPIO_PUPDR_PUPDR12_Pos);
//	GPIOD->MODER &= ~(GPIO_MODER_MODER13_Msk);
//	GPIOD->AFR[1] &= ~((uint32_t)0xF << GPIO_AFRH_AFRH4_Pos);
//	GPIOD->OSPEEDR &= ~(3U << GPIO_OSPEEDER_OSPEEDR13_Pos);
//	GPIOD->OTYPER  &= ~(1U << 13U) ;
//	GPIOD->PUPDR &= ~(3U << GPIO_PUPDR_PUPDR13_Pos);
//	GPIOE->MODER &= ~(GPIO_MODER_MODER2_Msk);
//	GPIOE->AFR[0] &= ~((uint32_t)0xF << GPIO_AFRL_AFRL2_Pos);
//	GPIOE->OSPEEDR &= ~(3U << GPIO_OSPEEDER_OSPEEDR2_Pos);
//	GPIOE->OTYPER  &= ~(1U << 2U) ;
//	GPIOE->PUPDR &= ~(3U << GPIO_PUPDR_PUPDR2_Pos);
//	/* Reset the QuadSPI memory interface */
//	RCC->AHB3RSTR |= (RCC_AHB3RSTR_QSPIRST);
//	RCC->AHB3RSTR &= ~(RCC_AHB3RSTR_QSPIRST);
//	/* Disable the QuadSPI memory interface clock */
//	RCC->AHB3ENR &= ~(RCC_AHB3ENR_QSPIEN);
//}
///*	parameters(mode):
// * 		QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE
// *		QSPI_FUNCTIONAL_MODE_INDIRECT_READ
// *		QSPI_FUNCTIONAL_MODE_AUTO_POLLING
// *		QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED
// */
//static void qspi_config(qspi_command_typedef *cmd, uint32_t mode) {
//  if ((cmd->DataMode != QSPI_DATA_NONE) && (mode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED))
//  {
//    /* Configure QSPI: DLR register with the number of data to read or write */
//    QUADSPI->DLR |= (cmd->NbData - 1);
//  }
//
//  if (cmd->InstructionMode != QSPI_INSTRUCTION_NONE)
//  {
//    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
//    {
//      /* Configure QSPI: ABR register with alternate bytes value */
//      QUADSPI->ABR = cmd->AlternateBytes;
//
//      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
//      {
//        /*---- Command with instruction, address and alternate bytes ----*/
//        /* Configure QSPI: CCR register with all communications parameters */
//        QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
//                                         cmd->AlternateByteMode | cmd->AddressSize | cmd->AddressMode |
//                                         cmd->InstructionMode | cmd->Instruction | mode);
//
//        if (mode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
//        {
//          /* Configure QSPI: AR register with address value */
//          QUADSPI->AR = cmd->Address;
//        }
//      }
//      else
//      {
//        /*---- Command with instruction and alternate bytes ----*/
//        /* Configure QSPI: CCR register with all communications parameters */
//    	  QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
//                                         cmd->AlternateByteMode | cmd->AddressMode | cmd->InstructionMode |
//                                         cmd->Instruction | mode);
//      }
//    }
//    else
//    {
//      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
//      {
//        /*---- Command with instruction and address ----*/
//        /* Configure QSPI: CCR register with all communications parameters */
//    	  QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode |
//                                         cmd->AddressSize | cmd->AddressMode | cmd->InstructionMode |
//                                         cmd->Instruction | mode);
//
//        if (mode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
//        {
//          /* Configure QSPI: AR register with address value */
//        	QUADSPI->AR = cmd->Address;
//        }
//      }
//      else
//      {
//        /*---- Command with only instruction ----*/
//        /* Configure QSPI: CCR register with all communications parameters */
//    	  QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode |
//                                         cmd->AddressMode | cmd->InstructionMode | cmd->Instruction  |
//                                         mode);
//      }
//    }
//  }
//  else
//  {
//    if (cmd->AlternateByteMode != QSPI_ALTERNATE_BYTES_NONE)
//    {
//      /* Configure QSPI: ABR register with alternate bytes value */
//    	QUADSPI->ABR = cmd->AlternateBytes;
//
//      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
//      {
//        /*---- Command with address and alternate bytes ----*/
//        /* Configure QSPI: CCR register with all communications parameters */
//    	  QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
//                                         cmd->AlternateByteMode | cmd->AddressSize | cmd->AddressMode |
//                                         cmd->InstructionMode | mode);
//
//        if (mode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
//        {
//          /* Configure QSPI: AR register with address value */
//        	QUADSPI->AR = cmd->Address;
//        }
//      }
//      else
//      {
//        /*---- Command with only alternate bytes ----*/
//        /* Configure QSPI: CCR register with all communications parameters */
//    	  QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateBytesSize |
//                                         cmd->AlternateByteMode | cmd->AddressMode | cmd->InstructionMode |
//                                         mode);
//      }
//    }
//    else
//    {
//      if (cmd->AddressMode != QSPI_ADDRESS_NONE)
//      {
//        /*---- Command with only address ----*/
//        /* Configure QSPI: CCR register with all communications parameters */
//    	  QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                         cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode |
//                                         cmd->AddressSize | cmd->AddressMode | cmd->InstructionMode |
//                                         mode);
//
//        if (mode != QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED)
//        {
//          /* Configure QSPI: AR register with address value */
//        	QUADSPI->AR = cmd->Address;
//        }
//      }
//      else
//      {
//        /*---- Command with only data phase ----*/
//        if (cmd->DataMode != QSPI_DATA_NONE)
//        {
//          /* Configure QSPI: CCR register with all communications parameters */
//        	QUADSPI->CCR = (cmd->DdrMode | cmd->DdrHoldHalfCycle | cmd->SIOOMode |
//                                           cmd->DataMode | (cmd->DummyCycles << 18) | cmd->AlternateByteMode |
//                                           cmd->AddressMode | cmd->InstructionMode | mode);
//        }
//      }
//    }
//  }
//}
//void qspi_write_enable(qspi_init_typedef *init) {
//	qspi_command_typedef     cmd;
//	qspi_autoPolling_typedef autoP;
//
//	/* Enable write operations */
//	cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
//	cmd.Instruction       = WRITE_ENABLE_CMD;
//	cmd.AddressMode       = QSPI_ADDRESS_NONE;
//	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	cmd.DataMode          = QSPI_DATA_NONE;
//	cmd.DummyCycles       = 0;
//	cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
//	cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
//	cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
//	qspi_command(&cmd);
//	/* Configure automatic polling mode to wait for write enabling */
//	autoP.Match           = N25Q128A_SR_WREN;
//	autoP.Mask            = N25Q128A_SR_WREN;
//	autoP.MatchMode       = QSPI_MATCH_MODE_AND;
//	autoP.StatusBytesSize = 1;
//	autoP.Interval        = 0x10;
//	autoP.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
//
//	cmd.Instruction    = READ_STATUS_REG_CMD;
//	cmd.DataMode       = QSPI_DATA_1_LINE;
//
//	qspi_autoPolling(&init, &cmd, &autoP);
//}
//void qspi_command(qspi_command_typedef *cmd) {
//    /* Wait till BUSY flag reset */
//	while(QUADSPI->SR & QUADSPI_SR_BUSY) {}
//	if (cmd->DataMode == QSPI_DATA_NONE) {
//		/* Clear interrupt */
//    	QUADSPI->SR |= QUADSPI_SR_TEF | QUADSPI_SR_TCF;
//    }
//    /* Call the configuration function */
//	qspi_config(cmd, QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE);
//	if (cmd->DataMode == QSPI_DATA_NONE) {
//		/* Enable the QSPI Transfer Error Interrupt */
//		QUADSPI->CR |= QUADSPI_CR_TEIE |  QUADSPI_CR_TCIE;
//	}
//}
//void qspi_autoPolling(qspi_init_typedef *init, qspi_command_typedef *cmd, qspi_autoPolling_typedef *autoP) {
//    /* Wait till BUSY flag reset */
//	while(QUADSPI->SR & QUADSPI_SR_BUSY) {}
//	/* Configure QSPI: PSMAR register with the status match value */
//	QUADSPI->PSMAR, autoP->Match;
//	/* Configure QSPI: PSMKR register with the status mask value */
//	QUADSPI->PSMKR, autoP->Mask;
//	/* Configure QSPI: PIR register with the interval value */
//	QUADSPI->PIR, autoP->Interval;
//	/* Configure QSPI: CR register with Match mode and Automatic stop mode */
//	QUADSPI->CR &= ~(QUADSPI_CR_PMM | QUADSPI_CR_APMS);
//	QUADSPI->CR |= (autoP->MatchMode | autoP->AutomaticStop);
//    /* Clear interrupt */
//	QUADSPI->SR |= QUADSPI_SR_TEF | QUADSPI_SR_SMF;
//    /* Call the configuration function */
//    cmd->NbData = autoP->StatusBytesSize;
//    qspi_config(cmd, QSPI_FUNCTIONAL_MODE_AUTO_POLLING);
//    /* Enable the QSPI Transfer Error and status match Interrupt */
//    QUADSPI->CR |= QUADSPI_CR_SMIE | QUADSPI_CR_TEIE;
//}
//static void qspi_autoPolling _memReady(qspi_init_typedef *init, qspi_command_typedef *cmd, ) {
//  /* Configure automatic polling mode to wait for memory ready */
//  cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
//  cmd.Instruction       = READ_STATUS_REG_CMD;
//  cmd.AddressMode       = QSPI_ADDRESS_NONE;
//  cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//  cmd.DataMode          = QSPI_DATA_1_LINE;
//  cmd.DummyCycles       = 0;
//  cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
//  cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
//  cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
//  config.Match           = 0;
//  config.Mask            = N25Q128A_SR_WIP;
//  config.MatchMode       = QSPI_MATCH_MODE_AND;
//  config.StatusBytesSize = 1;
//  config.Interval        = 0x10;
//  config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;
//  qspi_autoPolling(&init, &cmd, &config);
//}
//void qspi_reset_memory(void) {
//	qspi_command_typedef cmd;
//
//	/* Initialize the reset enable command */
//	cmd.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
//	cmd.Instruction       = RESET_ENABLE_CMD;
//	cmd.AddressMode       = QSPI_ADDRESS_NONE;
//	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	cmd.DataMode          = QSPI_DATA_NONE;
//	cmd.DummyCycles       = 0;
//	cmd.DdrMode           = QSPI_DDR_MODE_DISABLE;
//	cmd.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
//	cmd.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
//
//	/* Send the command */
//	qspi_command(&cmd);
//	/* Send the reset memory command */
//	cmd.Instruction = RESET_MEMORY_CMD;
//	qspi_command(&cmd);
//	/* Configure automatic polling mode to wait the memory is ready */
//	qspi_autoPolling_memReady(&init);
//}
//void qspi_erase_chip(qspi_init_typedef *init, qspi_command_typedef *cmd) {
//
//	/* Initialize the erase command */
//	cmd->InstructionMode   = QSPI_INSTRUCTION_1_LINE;
//	cmd->Instruction       = BULK_ERASE_CMD;
//	cmd->AddressMode       = QSPI_ADDRESS_NONE;
//	cmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
//	cmd->DataMode          = QSPI_DATA_NONE;
//	cmd->DummyCycles       = 0;
//	cmd->DdrMode           = QSPI_DDR_MODE_DISABLE;
//	cmd->DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
//	cmd->SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
//
//	/* Enable write operations */
//	qspi_write_enable(&init);
//	/* Send the command */
//	qspi_command(&cmd);
//	/* Configure automatic polling mode to wait for end of erase */
//	qspi_autoPolling _memReady(&QSPIHandle, N25Q128A_BULK_ERASE_MAX_TIME);
//}
