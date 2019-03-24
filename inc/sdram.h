#ifndef __SDRAM_H
#define __SDRAM_H

#include <stm32f7xx_ll_fmc.h>
#include <stm32f7xx_hal.h>
#include <stm32f746xx.h>


#define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
#define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
#define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
#define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
#define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
#define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000)
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200)

#define SDRAM_MEMORY_WIDTH               FMC_SDRAM_MEM_BUS_WIDTH_16
#define SDCLOCK_PERIOD                   FMC_SDRAM_CLOCK_PERIOD_2

/* DMA definitions for SDRAM DMA transfer */
#define __DMAx_CLK_ENABLE                 __HAL_RCC_DMA2_CLK_ENABLE
#define __DMAx_CLK_DISABLE                __HAL_RCC_DMA2_CLK_DISABLE
#define SDRAM_DMAx_CHANNEL                DMA_CHANNEL_0
#define SDRAM_DMAx_STREAM                 DMA2_Stream0
#define SDRAM_DMAx_IRQn                   DMA2_Stream0_IRQn
#define BSP_SDRAM_DMA_IRQHandler          DMA2_Stream0_IRQHandler

#define SDRAM_BANK_ADDR                 ((uint32_t)0xC0000000)

typedef struct
{
  uint32_t CommandMode;                  /*!< Defines the command issued to the SDRAM device.
                                              This parameter can be a value of @ref FMC_SDRAM_Command_Mode.          */

  uint32_t CommandTarget;                /*!< Defines which device (1 or 2) the command will be issued to.
                                              This parameter can be a value of @ref FMC_SDRAM_Command_Target.        */

  uint32_t AutoRefreshNumber;            /*!< Defines the number of consecutive auto refresh command issued
                                              in auto refresh mode.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16   */
  uint32_t ModeRegisterDefinition;       /*!< Defines the SDRAM Mode register content                                */
} fmc_sdram_command_typedef;

typedef struct
{
  uint32_t SDBank;                      /*!< Specifies the SDRAM memory device that will be used.
                                             This parameter can be a value of @ref FMC_SDRAM_Bank                */

  uint32_t ColumnBitsNumber;            /*!< Defines the number of bits of column address.
                                             This parameter can be a value of @ref FMC_SDRAM_Column_Bits_number. */

  uint32_t RowBitsNumber;               /*!< Defines the number of bits of column address.
                                             This parameter can be a value of @ref FMC_SDRAM_Row_Bits_number.    */

  uint32_t MemoryDataWidth;             /*!< Defines the memory device width.
                                             This parameter can be a value of @ref FMC_SDRAM_Memory_Bus_Width.   */

  uint32_t InternalBankNumber;          /*!< Defines the number of the device's internal banks.
                                             This parameter can be of @ref FMC_SDRAM_Internal_Banks_Number.      */

  uint32_t CASLatency;                  /*!< Defines the SDRAM CAS latency in number of memory clock cycles.
                                             This parameter can be a value of @ref FMC_SDRAM_CAS_Latency.        */

  uint32_t WriteProtection;             /*!< Enables the SDRAM device to be accessed in write mode.
                                             This parameter can be a value of @ref FMC_SDRAM_Write_Protection.   */

  uint32_t SDClockPeriod;               /*!< Define the SDRAM Clock Period for both SDRAM devices and they allow
                                             to disable the clock before changing frequency.
                                             This parameter can be a value of @ref FMC_SDRAM_Clock_Period.       */

  uint32_t ReadBurst;                   /*!< This bit enable the SDRAM controller to anticipate the next read
                                             commands during the CAS latency and stores data in the Read FIFO.
                                             This parameter can be a value of @ref FMC_SDRAM_Read_Burst.         */

  uint32_t ReadPipeDelay;               /*!< Define the delay in system clock cycles on read data path.
                                             This parameter can be a value of @ref FMC_SDRAM_Read_Pipe_Delay.    */
} fmc_sdram_init_typedef;

typedef struct
{
  uint32_t LoadToActiveDelay;            /*!< Defines the delay between a Load Mode Register command and
                                              an active or Refresh command in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t ExitSelfRefreshDelay;         /*!< Defines the delay from releasing the self refresh command to
                                              issuing the Activate command in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t SelfRefreshTime;              /*!< Defines the minimum Self Refresh period in number of memory clock
                                              cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t RowCycleDelay;                /*!< Defines the delay between the Refresh command and the Activate command
                                              and the delay between two consecutive Refresh commands in number of
                                              memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t WriteRecoveryTime;            /*!< Defines the Write recovery Time in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t RPDelay;                      /*!< Defines the delay between a Precharge Command and an other command
                                              in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */

  uint32_t RCDDelay;                     /*!< Defines the delay between the Activate Command and a Read/Write
                                              command in number of memory clock cycles.
                                              This parameter can be a value between Min_Data = 1 and Max_Data = 16  */
} fmc_sdram_timing_typedef;


void sdram_init(void);
void fmc_sdram_bankConfig(fmc_sdram_init_typedef *Init);
void fmc_sdram_timing_init(fmc_sdram_timing_typedef *Timing, uint32_t Bank);
void sdram_external_device(fmc_sdram_init_typedef *init, fmc_sdram_command_typedef *Command);

#endif
