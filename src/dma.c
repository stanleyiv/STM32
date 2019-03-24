#include <dma.h>
void dma_init(dma_init_typedef *dmaInit)
{
	uint32_t tmp = 0U;
	//DMA_Base_Registers *regs;
	/* Disable the peripheral */
	DMA1_Stream0->CR &=  ~DMA_SxCR_EN;
	/* Check if the DMA Stream is effectively disabled */
	while(DMA1_Stream0->CR & DMA_SxCR_EN) {}
	/* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits */
	DMA1_Stream0->CR &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
					  DMA_SxCR_PL    | DMA_SxCR_MSIZE  | DMA_SxCR_PSIZE  | \
					  DMA_SxCR_MINC  | DMA_SxCR_PINC   | DMA_SxCR_CIRC   | \
					  DMA_SxCR_DIR   | DMA_SxCR_CT     | DMA_SxCR_DBM));

  /* Prepare the DMA Stream configuration */
  tmp |=  dmaInit->Channel             | dmaInit->Direction        |
		  dmaInit->PeriphInc           | dmaInit->MemInc           |
		  dmaInit->PeriphDataAlignment | dmaInit->MemDataAlignment |
		  dmaInit->Mode                | dmaInit->Priority;

  /* the Memory burst and peripheral burst are not used when the FIFO is disabled */
  if(dmaInit->FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get memory burst and peripheral burst */
    tmp |=  dmaInit->MemBurst | dmaInit->PeriphBurst;
  }

  /* Write to DMA Stream CR register */
  DMA1_Stream0->CR = tmp;

  /* Get the FCR register value */
  tmp = DMA1_Stream0->FCR;

  /* Clear Direct mode and FIFO threshold bits */
  tmp &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);

  /* Prepare the DMA Stream FIFO configuration */
  tmp |= dmaInit->FIFOMode;

  /* The FIFO threshold is not used when the FIFO mode is disabled */
  if(dmaInit->FIFOMode == DMA_FIFOMODE_ENABLE)
  {
    /* Get the FIFO threshold */
    tmp |= dmaInit->FIFOThreshold;


  }

  /* Write to DMA Stream FCR */
  DMA1_Stream0->FCR = tmp;

//  /* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
//     DMA steam Base Address needed by HAL_DMA_IRQHandler() and HAL_DMA_PollForTransfer() */
//  regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(hdma);

  /* Clear all interrupt flags */
  DMA1->LIFCR |= 0x3FU;
}






void dma_deinit(dma_init_typedef *dmaInit) {
	//DMA_Base_Registers *regs;
	//DMA_TypeDef dma;
	/* Disable the selected DMA Streamx */
	DMA1_Stream0->CR &=  ~DMA_SxCR_EN;
	/* Reset DMA Streamx control register */
	DMA1_Stream0->CR   = 0U;
	/* Reset DMA Streamx number of data to transfer register */
	DMA1_Stream0->NDTR = 0U;
	/* Reset DMA Streamx peripheral address register */
	DMA1_Stream0->PAR  = 0U;
	/* Reset DMA Streamx memory 0 address register */
	DMA1_Stream0->M0AR = 0U;
	/* Reset DMA Streamx memory 1 address register */
	DMA1_Stream0->M1AR = 0U;
	/* Reset DMA Streamx FIFO control register */
	DMA1_Stream0->FCR  = (uint32_t)0x00000021U;
	/* Get DMA steam Base Address */
	//regs = (DMA_Base_Registers *)DMA_CalcBaseAndBitshift(hdma);

	/* Clear all interrupt flags at correct offset within the register */
	//regs->IFCR = 0x3FU << hdma->StreamIndex;
	//DMA1->LISR = 0x0000003FU;
	DMA1->LIFCR |= 0x0000003FU;
//
//  /* Clean all callbacks */
//  hdma->XferCpltCallback = NULL;
//  hdma->XferHalfCpltCallback = NULL;
//  hdma->XferM1CpltCallback = NULL;
//  hdma->XferM1HalfCpltCallback = NULL;
//  hdma->XferErrorCallback = NULL;
//  hdma->XferAbortCallback = NULL;
}
