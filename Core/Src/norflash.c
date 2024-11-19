#include "norflash.h"
#include <stdio.h>

uint32_t GetSector(uint32_t address)
{
    // 根据地址计算扇区
    return (address - FSMC_Bank1_NORSRAM1_BASE) / FLASH_SECTOR_SIZE;
}

int Flash_write(uint32_t flash_address, uint32_t *data_buffer, uint32_t data_size)
{
    static FLASH_EraseInitTypeDef erase_init_struct;
    uint32_t sector_error = 0;
    uint32_t address = flash_address; // 记录写入的地

    /* 解锁 */
    HAL_FLASH_Unlock();

    /* 擦除 */
    erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS; // 使用扇区擦除
    erase_init_struct.Sector = GetSector(flash_address); // 获取要擦除的扇区
    erase_init_struct.NbSectors = (data_size + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE; // 计算要擦除的扇区

    if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK)
    {
        HAL_FLASH_Lock(); // 确保在出错时锁定
        return -1; // 擦除失败
    }

    /* 向外FLASH 写入数据 */
    uint32_t i = 0;
    while (i < data_size / 4) // 确保不会超出 data_buffer 的大
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data_buffer[i]) == HAL_OK)
        {
            address += 4; // 地址递增
            i++;
        }
        else
        {
            HAL_FLASH_Lock(); // 写入出错，确保锁
            return -1; // 写入失败
        }
    }

    /* 上锁 */
    HAL_FLASH_Lock();

    return 0; // 成功
}

int Flash_read(uint32_t flash_address, uint32_t *data_buffer, uint32_t data_size)
{
    // 查地是否在有效范围内
    if (flash_address < FSMC_Bank1_NORSRAM1_BASE || flash_address + data_size > (FSMC_Bank1_NORSRAM1_BASE + (8 * 1024 * 1024))) // 假设 S29GL064N 大小�????????????????? 64Mb
    {
        return -1; // 地址超出范围
    }

    // 读取数据
    uint32_t address = flash_address;
    uint32_t i = 0;

    while (i < data_size / 4)
    {
        data_buffer[i] = *(__IO uint32_t*)address;
        address += 4;
        i++;
    }

    return 0; // 成功
}

void Read_NOR_Flash_ID(uint16_t *manufacturer_id, uint16_t *device_id) {

    *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x555 << 1))) = 0xAA;
    *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x2AA << 1))) = 0x55;
    *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x555 << 1))) = 0x90;

    *manufacturer_id = *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x000 << 1)));

    *device_id = *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x001 << 1)));

    *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x555 << 1))) = 0xAA;
    *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x2AA << 1))) = 0x55;
    *((volatile uint16_t *)(FSMC_Bank1_NORSRAM1_BASE + (0x555 << 1))) = 0xF0;
}

void ReadICInfo(NOR_HandleTypeDef *hnor)
{
	uint16_t manufacturer_id = 0;
	uint16_t device_id = 0;

	NOR_IDTypeDef id;
	HAL_StatusTypeDef sts = HAL_NOR_Read_ID(hnor, &id);
	printf("sts1 = %d\n", sts);

	printf("HAL_NOR_Read_ID = %04x %04x %04x %04x \n",
	id.Manufacturer_Code,
	id.Device_Code1,
	id.Device_Code2,
	id.Device_Code3);

	Read_NOR_Flash_ID(&manufacturer_id, &device_id);
	printf("Read_NOR_Flash_ID = %04x %04x \n",manufacturer_id,device_id);


}
void WriteTest(){
	uint32_t flash_address = FSMC_Bank1_NORSRAM1_BASE;
	uint32_t data_buffer[256] = {0};
	uint32_t data_size = 256;
	for(uint32_t i=0;i<data_size;i++){
		data_buffer[i] = i;
	}
	Flash_write(flash_address, data_buffer, data_size);
}
void ReadTest()
{
	uint32_t flash_address = FSMC_Bank1_NORSRAM1_BASE;
	uint32_t data_buffer[256] = {0};
	uint32_t data_size = 256;

	Flash_read( flash_address,  data_buffer, data_size);
	printf("%lu", data_buffer[1]);
}

/******************************************************************************
 * Function Name : FSMC_NOR_ReadID
 * Description : Reads NOR memory's Manufacturer and Device Code.
 * Input : - NOR_ID: pointer to a NOR_IDTypeDef structure which will hold
 * the Manufacturer and Device Code.
 * Output : None
 * Return : None
 * Attention         : None
 *******************************************************************************/

void FSMC_NOR_ReadID(NOR_IDTypeDef *NOR_ID)
{
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x0090);

    NOR_ID->Manufacturer_Code = *(vu16 *)ADDR_SHIFT(0x0000);

    NOR_ID->Device_Code1 = *(vu16 *)ADDR_SHIFT(0x0001);
    NOR_ID->Device_Code2 = *(vu16 *)ADDR_SHIFT(0x000E);

    NOR_ID->Device_Code3 = *(vu16 *)ADDR_SHIFT(0x000F);
}

/*******************************************************************************
 * Function Name : FSMC_NOR_EraseBlock
 * Description : Erases the specified Nor memory block.
 * Input : - BlockAddr: address of the block to erase.
 * Output : None
 * Return : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
 * or NOR_TIMEOUT
 * Attention         : None
 *******************************************************************************/
NOR_Status FSMC_NOR_EraseBlock(uint32_t BlockAddr)
{
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x0080);
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
    NOR_WRITE((NOR_FLASH_START_ADDR + BlockAddr), 0x30);

    return (FSMC_NOR_GetStatus(BlockErase_Timeout));
}

/*******************************************************************************
 * Function Name : FSMC_NOR_EraseChip
 * Description : Erases the entire chip.
 * Input : None
 * Output : None
 * Return : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
 * or NOR_TIMEOUT
 * Attention         : None
 *******************************************************************************/
NOR_Status FSMC_NOR_EraseChip(void)
{
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x0080);
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x0010);

    return (FSMC_NOR_GetStatus(ChipErase_Timeout));
}

/******************************************************************************

* Function Name : FSMC_NOR_WriteHalfWord

* Description : Writes a half-word to the NOR memory.

* Input : - WriteAddr : NOR memory internal address to write to.

* - Data : Data to write.

* Output : None

* Return : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
* or NOR_TIMEOUT

* Attention         : None

*******************************************************************************/
NOR_Status FSMC_NOR_WriteHalfWord(uint32_t WriteAddr, uint16_t Data)
{
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00A0);
    NOR_WRITE((NOR_FLASH_START_ADDR + WriteAddr), Data);

    return (FSMC_NOR_GetStatus(Program_Timeout));
}

/*******************************************************************************

* Function Name : FSMC_NOR_WriteBuffer

* Description : Writes a half-word buffer to the FSMC NOR memory.

* Input : - pBuffer : pointer to buffer.

* - WriteAddr : NOR memory internal address from which the data

* will be written.

* - NumHalfwordToWrite : number of Half words to write.

* Output : None

* Return : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
* or NOR_TIMEOUT

* Attention         : None

*******************************************************************************/
NOR_Status FSMC_NOR_WriteBuffer(uint16_t *pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
    NOR_Status status = NOR_ONGOING;

    do
    {
        /* Transfer data to the memory */
        status = FSMC_NOR_WriteHalfWord(WriteAddr, *pBuffer++);
        WriteAddr = WriteAddr + 2;
        NumHalfwordToWrite--;
    } while ((status == NOR_SUCCESS) && (NumHalfwordToWrite != 0));

    return (status);
}

/*******************************************************************************
 * Function Name : FSMC_NOR_ProgramBuffer
 * Description : Writes a half-word buffer to the FSMC NOR memory. This function nor a0-a15 stm32 a1-a16
 * must be used only with S29GL128P NOR memory.
 * Input : - pBuffer : pointer to buffer.
 * - WriteAddr: NOR memory internal address from which the data
 * will be written.
 * - NumHalfwordToWrite: number of Half words to write.
 * The maximum allowed value is 32 Half words (64 bytes).
 * Output : None
 * Return : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR
 * or NOR_TIMEOUT
 * Attention         : None
 *******************************************************************************/
#define MAX_WRITE_BLOCK_SIZE 16
NOR_Status FSMC_NOR_ProgramBuffer(uint16_t *pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
     // 确保写入的半字数不超过最大限制
    if (NumHalfwordToWrite > MAX_WRITE_BLOCK_SIZE) {
        return NOR_ERROR; // 超过最大写入块大小
    }

    // 初始化变量
    uint32_t currentaddress = WriteAddr;

    // 解锁命令序列
    NOR_WRITE(ADDR_SHIFT(0x00555), 0x00AA); // 第1步：写入AA到地址555
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055); // 第2步：写入55到地址2AA

    // 写入缓冲区命令
    NOR_WRITE(ADDR_SHIFT(0x00555), 0x00AA); // 第3步：再次写入AA到地址555
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055); // 第4步：再次写入55到地址2AA
    NOR_WRITE(ADDR_SHIFT(WriteAddr), 0x0025); // 第5步：写入25到编程地址，表示写入缓冲区
    NOR_WRITE(ADDR_SHIFT(WriteAddr), (NumHalfwordToWrite - 1)); // 第6步：写入WC，表示要写入的半字数减1

    // 加载数据到NOR缓冲区
    for (uint32_t i = 0; i < NumHalfwordToWrite; i++)
    {
        NOR_WRITE(ADDR_SHIFT(currentaddress), *pBuffer++); // 写入数据到缓冲区
        currentaddress += 2; // 每个半字为2字节，更新地址
    }

    // 编程缓冲区到闪存命令
    NOR_WRITE(ADDR_SHIFT(currentaddress - 2), 0x29); // 第7步：写入29到最后一个地址，表示将缓冲区编程到闪存

    // 返回操作状态
    return FSMC_NOR_GetStatus(Program_Timeout); // 检查状态
}

#define NOR_CMD_ADDRESS_FIRST                 (uint16_t)0x0555
#define NOR_CMD_DATA_FIRST                    (uint16_t)0x00AA
#define NOR_CMD_ADDRESS_SECOND                (uint16_t)0x02AA
#define NOR_CMD_DATA_SECOND                   (uint16_t)0x0055
#define NOR_CMD_DATA_BUFFER_AND_PROG          (uint8_t)0x25
#define NOR_CMD_DATA_BUFFER_AND_PROG_CONFIRM  (uint8_t)0x29

HAL_StatusTypeDef NOR_ProgramBuffer(NOR_HandleTypeDef *hnor, uint32_t uwAddress, uint16_t *pData,
                                        uint32_t uwBufferSize)
{
  uint16_t *p_currentaddress;
  const uint16_t *p_endaddress;
  uint16_t *data = pData;
  uint32_t deviceaddress;
  HAL_StatusTypeDef status = HAL_OK;

  //printf("Program Addr:0x%08x size:0x%08x\r\n", uwAddress, uwBufferSize);

  /* Check the NOR controller state */
  if (hnor->State == HAL_NOR_STATE_BUSY)
  {
    return HAL_BUSY;
  }
  else if (hnor->State == HAL_NOR_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hnor);

    /* Update the NOR controller state */
    hnor->State = HAL_NOR_STATE_BUSY;

    deviceaddress = NOR_MEMORY_ADRESS1;

    /* Initialize variables */
    p_currentaddress  = (uint16_t *)(deviceaddress + uwAddress);
    p_endaddress      = (uint16_t *)(deviceaddress + uwAddress + (2U * (uwBufferSize - 1U)));

    //uint32_t addr1 = NOR_ADDR_SHIFT(deviceaddress, NOR_MEMORY_16B, NOR_CMD_ADDRESS_FIRST);
    //uint32_t addr2 = ADDR_SHIFT(NOR_CMD_ADDRESS_FIRST);
    //printf("addr1:%08x addr2:%08x\r\n", addr1, addr2);

	/* Issue unlock command sequence */
	NOR_WRITE(NOR_ADDR_SHIFT(deviceaddress, NOR_MEMORY_16B, NOR_CMD_ADDRESS_FIRST), NOR_CMD_DATA_FIRST);
	NOR_WRITE(NOR_ADDR_SHIFT(deviceaddress, NOR_MEMORY_16B, NOR_CMD_ADDRESS_SECOND), NOR_CMD_DATA_SECOND);

	/* Write Buffer Load Command */
	NOR_WRITE((deviceaddress + uwAddress), NOR_CMD_DATA_BUFFER_AND_PROG);
	NOR_WRITE((deviceaddress + uwAddress), (uint16_t)(uwBufferSize - 1U));

	/* Load Data into NOR Buffer */
	while (p_currentaddress <= p_endaddress)
	{
		NOR_WRITE(p_currentaddress, *data);
		data++;
		p_currentaddress ++;
	}
	NOR_WRITE((deviceaddress + uwAddress), NOR_CMD_DATA_BUFFER_AND_PROG_CONFIRM);

	status = FSMC_NOR_GetStatus(Program_Timeout);

    /* Check the NOR controller state */
    hnor->State = HAL_NOR_STATE_READY;

    /* Process unlocked */
    __HAL_UNLOCK(hnor);
  }
  else
  {
    return HAL_ERROR;
  }

  return status;

}

// 封装函数：支持更长数据的写入
NOR_Status FSMC_NOR_ProgramBuffer_Extended(NOR_HandleTypeDef *hnor, uint16_t *pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
	NOR_Status status = NOR_SUCCESS;
    uint32_t remainingData = NumHalfwordToWrite;  // 剩余需要写入的数据量
    uint32_t currentWriteAddr = WriteAddr;        // 当前写入的地址

    // 分批次写入数据，每次最多32个半字
    while (remainingData > 0)
    {
        // 计算本次写入的数据量，最多写32个半字
        uint32_t writeSize = (remainingData > MAX_WRITE_BLOCK_SIZE) ? MAX_WRITE_BLOCK_SIZE : remainingData;

        // 调用原始的 FSMC_NOR_ProgramBuffer 进行写入
        //status = FSMC_NOR_ProgramBuffer(pBuffer, currentWriteAddr, writeSize);
        status = NOR_ProgramBuffer(hnor, currentWriteAddr, pBuffer, writeSize);
        if (status != NOR_SUCCESS)
        {
            // 如果写入失败，返回错误
            return status;
        }

        // 更新剩余数据量和地址
        remainingData -= writeSize;
        pBuffer += writeSize;            // 移动数据指针
        currentWriteAddr += writeSize * 2;  // 每个半字为2字节

    }

    return status;
}
/******************************************************************************
 * Function Name : FSMC_NOR_ReadHalfWord
 * Description : Reads a half-word from the NOR memory.
 * Input : - ReadAddr : NOR memory internal address to read from.
 * Output : None
 * Return : Half-word read from the NOR memory
 * Attention         : None
 *******************************************************************************/
uint16_t FSMC_NOR_ReadHalfWord(uint32_t ReadAddr)
{
    NOR_WRITE(ADDR_SHIFT(0x00555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x002AA), 0x0055);
    NOR_WRITE((NOR_FLASH_START_ADDR + ReadAddr), 0x00F0);

    return (*(vu16 *)((NOR_FLASH_START_ADDR + ReadAddr)));
}

/*******************************************************************************

* Function Name : FSMC_NOR_ReadBuffer

* Description : Reads a block of data from the FSMC NOR memory.

* Input : - pBuffer : pointer to the buffer that receives the data read

* from the NOR memory.

* - ReadAddr : NOR memory internal address to read from.

* - NumHalfwordToRead : number of Half word to read.

* Output : None

* Return : None

* Attention         : None

*******************************************************************************/
void FSMC_NOR_ReadBuffer(uint16_t *pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead)
{
    NOR_WRITE(ADDR_SHIFT(0x0555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x02AA), 0x0055);
    NOR_WRITE((NOR_FLASH_START_ADDR + ReadAddr), 0x00F0);

    for (; NumHalfwordToRead != 0x00; NumHalfwordToRead--) /* while there is data to read */
    {
        /* Read a Halfword from the NOR */
        *pBuffer++ = *(vu16 *)((NOR_FLASH_START_ADDR + ReadAddr));
        ReadAddr = ReadAddr + 2;
    }
}


/******************************************************************************
 * Function Name : FSMC_NOR_ReturnToReadMode
 * Description : Returns the NOR memory to Read mode.
 * Input : None
 * Output : None
 * Return : NOR_SUCCESS
 * Attention         : None
 *******************************************************************************/
NOR_Status FSMC_NOR_ReturnToReadMode(void)
{
    NOR_WRITE(NOR_FLASH_START_ADDR, 0x00F0);

    return (NOR_SUCCESS);
}

/******************************************************************************

* Function Name : FSMC_NOR_Reset

* Description : Returns the NOR memory to Read mode and resets the errors in

* the NOR memory Status Register.

* Input : None

* Output : None

* Return : NOR_SUCCESS
* Attention         : None
*******************************************************************************/
NOR_Status FSMC_NOR_Reset(void)
{
    NOR_WRITE(ADDR_SHIFT(0x00555), 0x00AA);
    NOR_WRITE(ADDR_SHIFT(0x002AA), 0x0055);
    NOR_WRITE(NOR_FLASH_START_ADDR, 0x00F0);

    return (NOR_SUCCESS);
}

/******************************************************************************
* Function Name : FSMC_NOR_GetStatus
* Description : Returns the NOR operation status.
* Input : - Timeout: NOR progamming Timeout
* Output : None
* Return : NOR_Status:The returned value can be: NOR_SUCCESS, NOR_ERROR

* or NOR_TIMEOUT
* Attention         : None
*******************************************************************************/
NOR_Status FSMC_NOR_GetStatus(uint32_t Timeout)
{
    uint16_t val1 = 0x00, val2 = 0x00;
    NOR_Status status = NOR_ONGOING;
    uint32_t timeout = Timeout;

    /* Poll on NOR memory Ready/Busy signal ------------------------------------*/


    while ((HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) != GPIO_PIN_RESET) && (timeout > 0))
    {
        timeout--;
    }

    timeout = Timeout;

    while ((HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6) == GPIO_PIN_RESET) && (timeout > 0))
    {
        timeout--;
    }

    /* Get the NOR memory operation status -------------------------------------*/
    while ((Timeout != 0x00) && (status != NOR_SUCCESS))
    {
        Timeout--;

        /* Read DQ6 and DQ5 */
        val1 = *(vu16 *)(NOR_FLASH_START_ADDR);
        val2 = *(vu16 *)(NOR_FLASH_START_ADDR);

        /* If DQ6 did not toggle between the two reads then return NOR_Success */
        if ((val1 & 0x0040) == (val2 & 0x0040))
        {
            return NOR_SUCCESS;
        }

        if ((val1 & 0x0020) != 0x0020)
        {
            status = NOR_ONGOING;
        }

        val1 = *(vu16 *)(NOR_FLASH_START_ADDR);
        val2 = *(vu16 *)(NOR_FLASH_START_ADDR);

        if ((val1 & 0x0040) == (val2 & 0x0040))
        {
            return NOR_SUCCESS;
        }
        else if ((val1 & 0x0020) == 0x0020)
        {
            return NOR_ERROR;
        }
    }

    if (Timeout == 0x00)
    {
        status = NOR_TIMEOUT;
    }

    /* Return the operation status */
    return (status);
}
