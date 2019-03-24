#include <flash.h>

void flash_lock(void) {
	FLASH->CR |= FLASH_CR_LOCK;
}
void flash_unlock(void) {
	if(FLASH->CR & FLASH_CR_LOCK) {
		FLASH->KEYR = (uint32_t)0x45670123U;
		FLASH->KEYR = (uint32_t)0xCDEF89ABU;
		if(FLASH->CR & FLASH_CR_LOCK) {}
	}
}
void flash_option_lock(void) {
	FLASH->OPTCR |= FLASH_OPTCR_OPTLOCK;
}
void flash_option_unlock(void) {
	if(FLASH->OPTCR & FLASH_OPTCR_OPTLOCK){
		FLASH->OPTKEYR = FLASH_OPT_KEY1;
		FLASH->OPTKEYR = FLASH_OPT_KEY2;
	}
}
void flash_option_enable_WRP(uint32_t sector) {
	while(FLASH->SR & FLASH_SR_BSY) {}
	FLASH->OPTCR &= (~sector);
}
void flash_option_disable_WRP(uint32_t sector) {
	while(FLASH->SR & FLASH_SR_BSY) {}
	FLASH->OPTCR |= sector;
}
void flash_option_RDP_config(uint8_t level) {
	*(__IO uint8_t*)OPTCR_BYTE1_ADDRESS = level;
														/*	@arg OB_RDP_LEVEL_0: No protection
															@arg OB_RDP_LEVEL_1: Read protection of the memory
															@arg OB_RDP_LEVEL_2: Full chip protection */
}
void flash_option_boot_address(uint8_t BOOT_ADD, uint32_t bootAddress) {
	while(FLASH->SR & FLASH_SR_BSY) {}
	if(BOOT_ADD == 0) {
		FLASH->OPTCR1 = bootAddress; //BOOT_ADD0
	}
	else {
		FLASH->OPTCR1 = (bootAddress << 16U); //BOOT_ADD1
	}
/*	               OB_BOOTADDR_ITCM_RAM : Boot from ITCM RAM (0x00000000)
	               OB_BOOTADDR_SYSTEM : Boot from System memory bootloader (0x00100000)
	               OB_BOOTADDR_ITCM_FLASH : Boot from Flash on ITCM interface (0x00200000)
	               OB_BOOTADDR_AXIM_FLASH : Boot from Flash on AXIM interface (0x08000000)
	               OB_BOOTADDR_DTCM_RAM : Boot from DTCM RAM (0x20000000)
	               OB_BOOTADDR_SRAM1 : Boot from SRAM1 (0x20010000)
	               OB_BOOTADDR_SRAM2 : Boot from SRAM2 (0x2004C000)     */
}
void flash_mass_erase(void) {
	FLASH->CR |= FLASH_CR_EOPIE;
	FLASH->CR |= ((uint32_t)0x02000000U);
	FLASH->SR = (FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_ERSERR);
	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_CR_MER;
	if(FLASH->SR & FLASH_SR_BSY) {}
	FLASH->CR |= FLASH_CR_STRT | ((uint32_t)FLASH_VOLTAGE_RANGE_3 << 8U);
	__DSB();
}
void flash_sector_erase(uint32_t sector) { //FLASH_SECTOR_0 -> FLASH_SECTOR_8
	FLASH->CR |= FLASH_CR_EOPIE;
	FLASH->CR |= ((uint32_t)0x02000000U);
	FLASH->SR = (FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_ERSERR);
	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_PSIZE_WORD; //voltage range is 3, 32 bit
	FLASH->CR &= 0xFFFFFF07U;
	FLASH->CR |= FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
	if(FLASH->SR & FLASH_SR_BSY) {}
	FLASH->CR |= FLASH_CR_STRT;
	__DSB();
}
void flash_program(uint32_t type, uint32_t address, uint64_t data) {
	FLASH->CR |= FLASH_CR_EOPIE;
	FLASH->CR |= ((uint32_t)0x02000000U);
	FLASH->SR = (FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_ERSERR);
	while(FLASH->SR & FLASH_SR_BSY) {}
	switch(type) {
		case FLASH_TYPEPROGRAM_BYTE : {
			/*Program byte (8-bit) at a specified address.*/
			flash_program_byte(address, (uint8_t) data);
			break;
		}
		case FLASH_TYPEPROGRAM_HALFWORD : {
			/*Program halfword (16-bit) at a specified address.*/
			flash_program_halfWord(address, (uint16_t) data);
			break;
		}
		case FLASH_TYPEPROGRAM_WORD : {
			/*Program word (32-bit) at a specified address.*/
			flash_program_word(address, (uint32_t) data);
			break;
		}
		case FLASH_TYPEPROGRAM_DOUBLEWORD : {
			/*Program double word (64-bit) at a specified address.*/
			flash_program_doubleWord(address, data);
			break;
		}
		default :
			break;
	}
	while(FLASH->SR & FLASH_SR_BSY) {}
	FLASH->CR &= (~FLASH_CR_PG);
}
void flash_program_byte(uint32_t address, uint8_t data) {
	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_PSIZE_BYTE;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint8_t*)address = data;
	__DSB();
}
void flash_program_halfWord(uint32_t address, uint16_t data) {
	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_PSIZE_HALF_WORD;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint16_t*)address = data;
	__DSB();
}
void flash_program_word(uint32_t address, uint32_t data) {
	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_PSIZE_WORD;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint32_t*)address = data;
	__DSB();
}
void flash_program_doubleWord(uint32_t address, uint64_t data) {
	FLASH->CR &= CR_PSIZE_MASK;
	FLASH->CR |= FLASH_PSIZE_DOUBLE_WORD;
	FLASH->CR |= FLASH_CR_PG;
	*(__IO uint32_t*)address = data;
	*(__IO uint32_t*)(address+4) = (uint32_t)(data >> 32);
	__DSB();
}
