#ifndef __QSPI_H
#define __QSPI_H

#include <n25q128a.h>
#include <stm32f7xx_hal.h>
#include <stm32f746xx.h>

#define QSPI_FUNCTIONAL_MODE_INDIRECT_WRITE ((uint32_t)0x00000000U)          /*!<Indirect write mode*/
#define QSPI_FUNCTIONAL_MODE_INDIRECT_READ  ((uint32_t)QUADSPI_CCR_FMODE_0) /*!<Indirect read mode*/
#define QSPI_FUNCTIONAL_MODE_AUTO_POLLING   ((uint32_t)QUADSPI_CCR_FMODE_1) /*!<Automatic polling mode*/
#define QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED  ((uint32_t)QUADSPI_CCR_FMODE)   /*!<Memory-mapped mode*/

typedef struct
{
	uint32_t Instruction;        /* Specifies the Instruction to be sent
								  This parameter can be a value (8-bit) between 0x00 and 0xFF */
	uint32_t Address;            /* Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize)
								  This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
	uint32_t AlternateBytes;     /* Specifies the Alternate Bytes to be sent (Size from 1 to 4 bytes according AlternateBytesSize)
								  This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF */
	uint32_t AddressSize;        /* Specifies the Address Size
								  This parameter can be a value of @ref QSPI_AddressSize */
	uint32_t AlternateBytesSize; /* Specifies the Alternate Bytes Size
								  This parameter can be a value of @ref QSPI_AlternateBytesSize */
	uint32_t DummyCycles;        /* Specifies the Number of Dummy Cycles.
								  This parameter can be a number between 0 and 31 */
	uint32_t InstructionMode;    /* Specifies the Instruction Mode
								  This parameter can be a value of @ref QSPI_InstructionMode */
	uint32_t AddressMode;        /* Specifies the Address Mode
								  This parameter can be a value of @ref QSPI_AddressMode */
	uint32_t AlternateByteMode;  /* Specifies the Alternate Bytes Mode
								  This parameter can be a value of @ref QSPI_AlternateBytesMode */
	uint32_t DataMode;           /* Specifies the Data Mode (used for dummy cycles and data phases)
								  This parameter can be a value of @ref QSPI_DataMode */
	uint32_t NbData;             /* Specifies the number of data to transfer.
								  This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length
								  until end of memory)*/
	uint32_t DdrMode;            /* Specifies the double data rate mode for address, alternate byte and data phase
								  This parameter can be a value of @ref QSPI_DdrMode */
	uint32_t DdrHoldHalfCycle;   /* Specifies the DDR hold half cycle. It delays the data output by one half of
								  system clock in DDR mode.
								  This parameter can be a value of @ref QSPI_DdrHoldHalfCycle */
	uint32_t SIOOMode;          /* Specifies the send instruction only once mode
								  This parameter can be a value of @ref QSPI_SIOOMode */
} qspi_command_typedef;

typedef struct
{
  uint32_t Match;              /* Specifies the value to be compared with the masked status register to get a match.
                                  This parameter can be any value between 0 and 0xFFFFFFFF */
  uint32_t Mask;               /* Specifies the mask to be applied to the status bytes received.
                                  This parameter can be any value between 0 and 0xFFFFFFFF */
  uint32_t Interval;           /* Specifies the number of clock cycles between two read during automatic polling phases.
                                  This parameter can be any value between 0 and 0xFFFF */
  uint32_t StatusBytesSize;    /* Specifies the size of the status bytes received.
                                  This parameter can be any value between 1 and 4 */
  uint32_t MatchMode;          /* Specifies the method used for determining a match.
                                  This parameter can be a value of @ref QSPI_MatchMode */
  uint32_t AutomaticStop;      /* Specifies if automatic polling is stopped after a match.
                                  This parameter can be a value of @ref QSPI_AutomaticStop */
} qspi_autoPolling_typedef;

typedef struct
{
  uint32_t ClockPrescaler;     /* Specifies the prescaler factor for generating clock based on the AHB clock.
                                  This parameter can be a number between 0 and 255 */

  uint32_t FifoThreshold;      /* Specifies the threshold number of bytes in the FIFO (used only in indirect mode)
                                  This parameter can be a value between 1 and 32 */

  uint32_t SampleShifting;     /* Specifies the Sample Shift. The data is sampled 1/2 clock cycle delay later to
                                  take in account external signal delays. (It should be QSPI_SAMPLE_SHIFTING_NONE in DDR mode)
                                  This parameter can be a value of @ref QSPI_SampleShifting */

  uint32_t FlashSize;          /* Specifies the Flash Size. FlashSize+1 is effectively the number of address bits
                                  required to address the flash memory. The flash capacity can be up to 4GB
                                  (addressed using 32 bits) in indirect mode, but the addressable space in
                                  memory-mapped mode is limited to 256MB
                                  This parameter can be a number between 0 and 31 */

  uint32_t ChipSelectHighTime; /* Specifies the Chip Select High Time. ChipSelectHighTime+1 defines the minimum number
                                  of clock cycles which the chip select must remain high between commands.
                                  This parameter can be a value of @ref QSPI_ChipSelectHighTime */

  uint32_t ClockMode;          /* Specifies the Clock Mode. It indicates the level that clock takes between commands.
                                  This parameter can be a value of @ref QSPI_ClockMode */

  uint32_t FlashID;            /* Specifies the Flash which will be used,
                                  This parameter can be a value of @ref QSPI_Flash_Select */

  uint32_t DualFlash;          /* Specifies the Dual Flash Mode State
                                  This parameter can be a value of @ref QSPI_DualFlash_Mode */
} qspi_init_typedef;


void qspi_deinit(void);
void qspi_init(void);
void qspi_command(qspi_command_typedef *cmd);

#endif
