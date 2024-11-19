/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "norflash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

NOR_HandleTypeDef hnor1;

/* USER CODE BEGIN PV */

#define FIRMWARE_BUFFER_SIZE 1024 
uint8_t uartRxBuffer[FIRMWARE_BUFFER_SIZE];

#define PACKET_TYPE_WRITE_FIRMWARE    0x01
#define PACKET_TYPE_READ_FIRMWARE     0x02
#define PACKET_TYPE_WRITE_DATA        0x03
#define PACKET_TYPE_READ_DATA         0x04
#define PACKET_TYPE_RESPONSE          0x80   
#define PACKET_TYPE_MSG				  0x05
#define PACKET_TYPE_ERASE_BLOCK		  0x06
#define PACKET_TYPE_ERASE_ALL		  0x07
#define PACKET_TYPE_TEST			  0x08
#define PACKET_TYPE_RESET			  0x09

#define CMD_START                     0x01    
#define CMD_DATA                      0x02    
#define CMD_END                       0x03    
#define CMD_ERROR                     0xFF    

#define RESPONSE_OK                   0x00
#define RESPONSE_ERROR               0x01

#define MSG_BUF_SIZE 256

/* USER CODE END PD */

/* USER CODE BEGIN PTD */
#pragma pack(1)  
typedef struct {
    uint8_t  packetType;    
    uint8_t  command;        
    uint32_t length;         
    uint32_t addr;         
    uint16_t crc;           
} UartPacket;

typedef struct {
    uint8_t  packetType;     
    uint8_t  status;        
    uint16_t crc;         
} UartResponse;


#pragma pack() 

// Add DMA buffers and flags
#define DMA_RX_BUFFER_SIZE sizeof(UartPacket)
uint8_t dmaRxBuffer[DMA_RX_BUFFER_SIZE];
volatile uint8_t rxComplete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}


// CRC16
uint16_t CalculateCRC16(const uint8_t* data, uint32_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

void SendResponse(uint8_t originalType, uint8_t status) {
    UartResponse response = {
        .packetType = PACKET_TYPE_RESPONSE | originalType,
        .status = status
    };
    response.crc = CalculateCRC16((uint8_t*)&response, sizeof(UartResponse) - 2);
     // 使用阻塞式发送替代DMA
    HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(UartResponse), HAL_MAX_DELAY);
}


HAL_StatusTypeDef HandleFirmwareWrite(const UartPacket* packet) {
    uint32_t totalSize = packet->length;
    uint32_t remainingBytes = totalSize;
    uint32_t currentAddr = packet->addr;
    HAL_StatusTypeDef status;
    
    printf("HandleFirmwareWrite\r\n");
    printf("totalSize:%08x\r\n", (unsigned int)totalSize);
    printf("currentAddr:%08x\r\n", (unsigned int)currentAddr);

    HAL_UART_DMAStop(&huart2);
    HAL_UART_AbortReceive(&huart2);

    SendResponse(packet->packetType, RESPONSE_OK);
    
    while (remainingBytes > 0) {
        //printf("remainingBytes:%d\r\n", (int)remainingBytes);
        
        uint32_t blockSize = (remainingBytes>FIRMWARE_BUFFER_SIZE)?FIRMWARE_BUFFER_SIZE:remainingBytes;
        
        // 添加接收前的调试信息
        //printf("Waiting for data block, size: %d bytes\r\n", (int)blockSize);
        
        // 1. 使用HAL_UART_Receive接收数据
         // 每次接收前重置UART状�??
        HAL_UART_AbortReceive(&huart2);
        
        status = HAL_UART_Receive(&huart2, uartRxBuffer, blockSize, 5000);
        if (status != HAL_OK) {
            printf("UART receive failed, status: %d\r\n", (int)status);
            printf("HAL_UART_STATE: %d\r\n", (int)HAL_UART_GetState(&huart2));
            SendResponse(packet->packetType, RESPONSE_ERROR);
            return HAL_ERROR;
        }

        // 添加接收成功的调试信�???
        //printf("Data received successfully\r\n");

        //printf("nor flash write addr: 0x%08x, size: %d\r\n",
        //       (unsigned int)currentAddr, (int)blockSize);
               
        // 2. �???查NOR Flash写入结果
        //NOR_Status nor_status = HAL_NOR_ProgramBuffer(&hnor1, currentAddr, (uint16_t*)uartRxBuffer,
        //		blockSize/2);
        NOR_Status nor_status = FSMC_NOR_ProgramBuffer_Extended(&hnor1, (uint16_t*)uartRxBuffer,
                                                    currentAddr, blockSize/2);

        if (nor_status != NOR_SUCCESS) {
            printf("NOR write failed: %d\r\n", (int)nor_status);
            SendResponse(packet->packetType, RESPONSE_ERROR);
            return HAL_ERROR;
        }
        
        //printf("NOR write successful\r\n");
        
        remainingBytes -= blockSize;
        currentAddr += blockSize;
        
        //printf("Sending response\r\n");
        SendResponse(packet->packetType, RESPONSE_OK);
        //printf("Response sent, remaining: %d\r\n", (int)remainingBytes);
        
        // 添加短暂延时，确保上位机有足够时间处�???
        //HAL_Delay(1);
    }
    
    printf("HandleFirmwareWrite complete\r\n");
    return HAL_OK;
}


HAL_StatusTypeDef HandleFirmwareRead(const UartPacket* packet) {
    uint32_t readSize = packet->length;
    uint32_t remainingBytes = readSize;
    uint32_t currentAddr = packet->addr;
    
    SendResponse(packet->packetType, RESPONSE_OK);

    while (remainingBytes > 0) {
        uint32_t blockSize = (remainingBytes > FIRMWARE_BUFFER_SIZE) ? 
                            FIRMWARE_BUFFER_SIZE : remainingBytes;


        UartPacket dataPacket = {
            .packetType = packet->packetType,
            .command = CMD_DATA,
            .length = blockSize,
            .addr = currentAddr
        };
        dataPacket.crc = CalculateCRC16((uint8_t*)&dataPacket, sizeof(UartPacket) - 2);


        if (HAL_UART_Transmit(&huart2, (uint8_t*)&dataPacket, sizeof(UartPacket), 1000) != HAL_OK) {
            return HAL_ERROR;
        }


        FSMC_NOR_ReadBuffer((uint16_t*)uartRxBuffer, currentAddr, blockSize/2);
        

        if (HAL_UART_Transmit(&huart2, uartRxBuffer, blockSize, 10000) != HAL_OK) {
            return HAL_ERROR;
        }
        
        remainingBytes -= blockSize;
        currentAddr += blockSize;


        UartResponse response;
        if (HAL_UART_Receive(&huart2, (uint8_t*)&response, sizeof(UartResponse), 5000) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef HandleEraseAll(const UartPacket* packet)
{
	printf("HandleEraseAll\r\n");
	NOR_Status ret = FSMC_NOR_EraseChip();
	printf("HandleEraseAll ret:%d\r\n", (int)ret);
	if(ret != HAL_NOR_STATUS_SUCCESS)
		return HAL_ERROR;
	else{
		SendResponse(packet->packetType, RESPONSE_OK);
		return HAL_OK;
	}

}

HAL_StatusTypeDef HandleEraseBlock(const UartPacket* packet) {

    uint32_t currentAddr = packet->addr;

    printf("HandleEraseBlock:%08x\r\n", (unsigned int)currentAddr);
    NOR_Status ret = FSMC_NOR_EraseBlock(currentAddr);
    printf("HandleEraseBlock ret:%d\r\n", (int)ret);
    if(ret != HAL_NOR_STATUS_SUCCESS)
    	return HAL_ERROR;
    else{
    	SendResponse(packet->packetType, RESPONSE_OK);
    	return HAL_OK;
    }

}

void SoftwareReset(void) {
    NVIC_SystemReset(); // 触发系统复位
}

void ProcessPacket(const UartPacket* packet) {
	int i=0;
    
    uint16_t calculatedCRC = CalculateCRC16((uint8_t*)packet, sizeof(UartPacket) - 2);
    if (calculatedCRC != packet->crc) {
        SendResponse(packet->packetType, RESPONSE_ERROR);
        printf("\nRecv data:\n");
        printf("Packet Type: 0x%02x\n", packet->packetType);
		printf("Command: 0x%02x\n", packet->command);
		printf("Length: 0x%08x\n", (unsigned int)packet->length);
		printf("Address: 0x%08x\n", (unsigned int)packet->addr);
		printf("CRC: 0x%04x\n", packet->crc);

        for(i = 0; i<sizeof(UartPacket);i++)
        	printf("0x%02x ", ((uint8_t*)packet)[i]);
        printf("bad crc recv:0x%04x calc:0x%04x\n", packet->crc, calculatedCRC);
        return;
    }

    switch (packet->packetType) {
        case PACKET_TYPE_WRITE_FIRMWARE:
            if (HandleFirmwareWrite(packet) != HAL_OK) {
                
                return;
            }
            break;
            
        case PACKET_TYPE_READ_FIRMWARE:
            if (HandleFirmwareRead(packet) != HAL_OK) {
                SendResponse(packet->packetType, RESPONSE_ERROR);
                return;
            }
            break;
        case PACKET_TYPE_ERASE_BLOCK:
        	if (HandleEraseBlock(packet) != HAL_OK) {
				SendResponse(packet->packetType, RESPONSE_ERROR);
				return;
			}
        	break;
        case PACKET_TYPE_ERASE_ALL:
        	if (HandleEraseAll(packet) != HAL_OK) {
        	    SendResponse(packet->packetType, RESPONSE_ERROR);
        	    return;
        	}
            break;
        case PACKET_TYPE_RESET:
        	SendResponse(packet->packetType, RESPONSE_OK);
        	HAL_Delay(300);
        	SoftwareReset();
        	break;
        default:
            SendResponse(packet->packetType, RESPONSE_ERROR);
            break;
    }
}
// Add DMA callback functions
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2) {
        rxComplete = 1;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  NOR_IDTypeDef NOR_ID;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FSMC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(200);
  //SD_Driver.disk_initialize(0);
  //printf_sdcard_info();
  printf("System started!\r\n");
  ReadICInfo(&hnor1);

  FSMC_NOR_ReadID(&NOR_ID);
  
  // 启动首次DMA接收
  HAL_UART_Receive_DMA(&huart2, dmaRxBuffer, DMA_RX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (rxComplete) {
        rxComplete = 0;
        
        // 在处理数据包之前暂停DMA接收
        HAL_UART_DMAStop(&huart2);
        
        // 处理接收到的命令�???
        ProcessPacket((UartPacket*)dmaRxBuffer);
        
        // 清空DMA缓冲�???
        memset(dmaRxBuffer, 0, DMA_RX_BUFFER_SIZE);
        
        // 重新启动DMA接收下一个命令包
        HAL_UART_Receive_DMA(&huart2, dmaRxBuffer, DMA_RX_BUFFER_SIZE);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the NOR1 memory initialization sequence
  */
  hnor1.Instance = FSMC_NORSRAM_DEVICE;
  hnor1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hnor1.Init */
  hnor1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hnor1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hnor1.Init.MemoryType = FSMC_MEMORY_TYPE_NOR;
  hnor1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hnor1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hnor1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hnor1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hnor1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hnor1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hnor1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hnor1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hnor1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hnor1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hnor1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 40;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_NOR_Init(&hnor1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
