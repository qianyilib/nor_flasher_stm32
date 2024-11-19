/*
 * norflash.h
 *
 *  Created on: Nov 11, 2024
 *      Author: Administrator
 */

#ifndef INC_NORFLASH_H_
#define INC_NORFLASH_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"

#define FLASH_SECTOR_SIZE  (64 * 1024)   // S29GL064N 每个扇区大小 64KB
#define FSMC_Bank1_NORSRAM1_BASE  ((uint32_t)0x60000000)

#define Bank1_NOR2_ADDR       ((uint32_t)0x64000000)
/* Delay definition */
#define BlockErase_Timeout    ((uint32_t)0x00A00000)
#define ChipErase_Timeout     ((uint32_t)0x30000000)
#define Program_Timeout       ((uint32_t)0x00001400)
/**
  * @}
  */

/** @defgroup STM3210E_EVAL_FSMC_NOR_Private_Macros
  * @{
  */
#define NOR_FLASH_START_ADDR FSMC_Bank1_NORSRAM1_BASE
#define ADDR_SHIFT(A) (NOR_FLASH_START_ADDR + (2 * (A)))
//#define NOR_WRITE(Address, Data)  (*(__IO uint16_t *)(Address) = (Data))


typedef volatile uint16_t vu16;
typedef HAL_NOR_StatusTypeDef NOR_Status;

/* Function prototypes */
uint32_t GetSector(uint32_t address);
int Flash_write(uint32_t flash_address, uint32_t *data_buffer, uint32_t data_size);
int Flash_read(uint32_t flash_address, uint32_t *data_buffer, uint32_t data_size);
void Read_NOR_Flash_ID(uint16_t *manufacturer_id, uint16_t *device_id);
void ReadICInfo(NOR_HandleTypeDef *hnor);
void WriteTest(void);
void ReadTest(void);

void FSMC_NOR_ReadID(NOR_IDTypeDef *NOR_ID);
NOR_Status FSMC_NOR_EraseBlock(uint32_t BlockAddr);
NOR_Status FSMC_NOR_EraseChip(void);
NOR_Status FSMC_NOR_WriteHalfWord(uint32_t WriteAddr, uint16_t Data);
NOR_Status FSMC_NOR_WriteBuffer(uint16_t *pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite);
NOR_Status FSMC_NOR_ProgramBuffer(uint16_t *pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite);
NOR_Status FSMC_NOR_ProgramBuffer_Extended(NOR_HandleTypeDef *hnor, uint16_t *pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite);
uint16_t FSMC_NOR_ReadHalfWord(uint32_t ReadAddr);
void FSMC_NOR_ReadBuffer(uint16_t *pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead);
NOR_Status FSMC_NOR_ReturnToReadMode(void);
NOR_Status FSMC_NOR_Reset(void);
NOR_Status FSMC_NOR_GetStatus(uint32_t Timeout);

#ifdef __cplusplus
}
#endif

#endif /* INC_NORFLASH_H_ */
