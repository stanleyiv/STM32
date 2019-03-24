#ifndef __FLASH_H
#define __FLASH_H

#include <stm32f7xx_hal_flash.h>
#include <stm32f7xx_hal_flash_ex.h>

void flash_lock(void);
void flash_unlock(void);
void flash_option_lock(void);
void flash_option_unlock(void);
void flash_option_enable_WRP(uint32_t sector);
void flash_option_disable_WRP(uint32_t sector);
void flash_option_RDP_config(uint8_t level);
void flash_option_boot_address(uint8_t BOOT_ADD, uint32_t bootAddress);
void flash_mass_erase(void);
void flash_sector_erase(uint32_t sector);
void flash_program_byte(uint32_t address, uint8_t data);
void flash_program_halfWord(uint32_t address, uint16_t data);
void flash_program_word(uint32_t address, uint32_t data);
void flash_program_doubleWord(uint32_t address, uint64_t data);

#endif
