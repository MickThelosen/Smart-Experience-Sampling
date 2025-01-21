/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DW1000.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVER_CHANNEL 			(0)
#define MAX_NUM_PACKET          (100)     /* Number of packets used for the test */
#define TX_WAKEUP_TIME          (400)     /* 400 us */
#define RX_WAKEUP_TIME          (380)     /* 300 us */
#define RX_TIMEOUT_ACK          (40000)   /* 400 us */
#define RX_TIMEOUT           	(100000)  /* 100 ms */
#define MAX_BEACON_COUNT 		(40)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DW1000_t dw1000 = { .spi = &hspi2, .NSS_port = GPIOA, .NSS_pin = GPIO_PIN_4,
		.NRST_port = GPIOB, .NRST_pin = GPIO_PIN_10, .tx = 1 };

uint8_t sendData[MAX_PACKET_LENGTH];
uint8_t receivedData[MAX_PACKET_LENGTH];
uint8_t receivedDistanceData[MAX_PACKET_LENGTH];
uint8_t sendNewPacket = FALSE;
uint8_t beacon_count = 0;
uint8_t current_channel = 0;
uint64_t beacon_ID[MAX_BEACON_COUNT];
uint64_t distance_to_beacon[MAX_BEACON_COUNT];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RADIO_Init(void);
static void MX_RADIO_TIMER_Init(void);
/* USER CODE BEGIN PFP */
void split_from_uint64(uint64_t, uint8_t*);
uint64_t combine_to_uint64(uint8_t*, uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_RADIO_Init();
	MX_RADIO_TIMER_Init();
	/* USER CODE BEGIN 2 */
	DW1000_init(&dw1000);
	uint8_t pll_lock = DW1000_config(&dw1000, &dw1000_cfg);
	if (pll_lock) {
		hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
		HAL_SPI_Init(&hspi2);
	}
	uint64_t ID = *((uint64_t*) UID64_BASE);
	uint8_t i = 2;
	sendData[1] = 8;
	do {
		sendData[i++] = ID & 0xFF;
	} while (ID >>= 8);
	HAL_RADIO_SetNetworkID(0x88DF88DF);
	HAL_RADIO_SetTxPower(0x18);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		HAL_RADIO_TIMER_Tick();
		if (sendNewPacket) {
			sendNewPacket = FALSE;
			for (current_channel = 0; current_channel < MAX_BEACON_COUNT;
					current_channel++) {
				sendData[0] = 0x1D;
				HAL_RADIO_SendPacketWithAck(current_channel, TX_WAKEUP_TIME,
						sendData, receivedData, RX_TIMEOUT_ACK,
						MAX_LL_PACKET_LENGTH, HAL_RADIO_Callback);
				HAL_Delay(40);
				HAL_RADIO_TIMER_Tick();
				if (beacon_ID[current_channel] != 0) {
					DW1000_initiator(&dw1000, current_channel);
					sendData[0] = 0xDD;
					HAL_RADIO_SendPacketWithAck(current_channel,
					TX_WAKEUP_TIME, sendData, receivedData,
					RX_TIMEOUT_ACK,
					MAX_LL_PACKET_LENGTH, HAL_RADIO_Callback);
					HAL_Delay(40);
					HAL_RADIO_TIMER_Tick();

				}
			}
			if (beacon_count > 0) {
				memset(receivedDistanceData, 0, sizeof receivedDistanceData);
				receivedDistanceData[0] = 0xFF;
				uint8_t clicker_uuid[8];
				uint8_t data_size = 10;
				split_from_uint64(*((uint64_t*) UID64_BASE), clicker_uuid);
				for (uint8_t k = 2, l = 0; k < 10; k++, l++) {
					receivedDistanceData[k] = clicker_uuid[l];
				}
				for (uint8_t j = 0; j + 1 < MAX_BEACON_COUNT; j++) {
					if (distance_to_beacon[j] != 0 && beacon_ID[j] != 0) {
						uint8_t uuid[8];
						split_from_uint64(beacon_ID[j], uuid);
						for (uint8_t k = data_size, l = 0; k < data_size + 8;
								k++, l++) {
							receivedDistanceData[k] = uuid[l];
						}
						receivedDistanceData[data_size + 8] =
								(uint8_t) distance_to_beacon[j] / 100;
						receivedDistanceData[data_size + 9] =
								distance_to_beacon[j]
										- (receivedDistanceData[data_size + 8]
												* 100);
						data_size += 10;
					}
				}
				receivedDistanceData[1] = data_size;
				HAL_RADIO_SendPacketWithAck(SERVER_CHANNEL,
				TX_WAKEUP_TIME, receivedDistanceData, receivedData,
				RX_TIMEOUT_ACK,
				MAX_LL_PACKET_LENGTH, HAL_RADIO_Callback);
				HAL_Delay(40);
				HAL_RADIO_TIMER_Tick();
				memset(distance_to_beacon, 0, sizeof distance_to_beacon);
				memset(beacon_ID, 0, sizeof beacon_ID);
				beacon_count = 0;
			}
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource and SYSCLKDivider
	 */
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_RC64MPLL;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_RC64MPLL_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_WAIT_STATES_0)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
	PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLK_DIV4;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief RADIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_RADIO_Init(void) {

	/* USER CODE BEGIN RADIO_Init 0 */

	/* USER CODE END RADIO_Init 0 */

	RADIO_HandleTypeDef hradio = { 0 };

	/* USER CODE BEGIN RADIO_Init 1 */

	/* USER CODE END RADIO_Init 1 */
	hradio.Instance = RADIO;
	HAL_RADIO_Init(&hradio);
	/* USER CODE BEGIN RADIO_Init 2 */

	/* USER CODE END RADIO_Init 2 */

}

/**
 * @brief RADIO_TIMER Initialization Function
 * @param None
 * @retval None
 */
static void MX_RADIO_TIMER_Init(void) {

	/* USER CODE BEGIN RADIO_TIMER_Init 0 */

	/* USER CODE END RADIO_TIMER_Init 0 */

	RADIO_TIMER_InitTypeDef RADIO_TIMER_InitStruct = { 0 };

	/* USER CODE BEGIN RADIO_TIMER_Init 1 */

	/* USER CODE END RADIO_TIMER_Init 1 */

	if (__HAL_RCC_RADIO_IS_CLK_DISABLED()) {
		/* Radio Peripheral reset */
		__HAL_RCC_RADIO_FORCE_RESET();
		__HAL_RCC_RADIO_RELEASE_RESET();

		/* Enable Radio peripheral clock */
		__HAL_RCC_RADIO_CLK_ENABLE();
	}
	/* Wait to be sure that the Radio Timer is active */
	while (LL_RADIO_TIMER_GetAbsoluteTime(WAKEUP) < 0x10)
		;
	RADIO_TIMER_InitStruct.XTAL_StartupTime = 320;
	RADIO_TIMER_InitStruct.enableInitialCalibration = FALSE;
	RADIO_TIMER_InitStruct.periodicCalibrationInterval = 0;
	HAL_RADIO_TIMER_Init(&RADIO_TIMER_InitStruct);
	/* USER CODE BEGIN RADIO_TIMER_Init 2 */

	/* USER CODE END RADIO_TIMER_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SWDIO;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/**/
	LL_PWR_EnableGPIOPullUp(LL_PWR_GPIO_A, LL_PWR_GPIO_BIT_2);

	/**/
	LL_PWR_SetNoPullA(LL_PWR_GPIO_BIT_4);

	/**/
	LL_PWR_SetNoPullB(LL_PWR_GPIO_BIT_10);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(GPIOB_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(GPIOB_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void split_from_uint64(uint64_t id, uint8_t *array) {
	for (uint8_t i = 0; i < 8; i++) {
		array[i] = (id >> (8 * (7 - i))) & 0xFF;
	}
}
uint64_t combine_to_uint64(uint8_t *array, uint8_t start_index) {
	uint64_t result = 0;

	for (uint8_t i = 0; i < 8; i++) {
		result |= ((uint64_t) array[start_index + i]) << (8 * (7 - i));
	}

	return result;
}

void HAL_RADIO_CallbackRcvOk(RxStats_t *rxPacketStats) {
	if (receivedData[0] == 0x1D) {
		uint64_t ID = combine_to_uint64(receivedData, 2U);
		uint8_t duplicate = 0;
		for (uint8_t i = 0; i < 39; i++) {
			if (beacon_ID[i] == ID) {
				duplicate++;
			}
		}
		if (duplicate == 0) {
			beacon_ID[current_channel] = ID;
		}
		beacon_count = 0;
		for (uint8_t i = 0; i < 39; i++) {
			if (beacon_ID[i] != 0) {
				beacon_count++;
			}
		}
		if (receivedData[10] != 0 || receivedData[11] != 0) {
			uint32_t measured_distance = (receivedData[10] * 100)
					+ receivedData[11];
			uint32_t distance =
					measured_distance
							* (0.5316 + 0.0259 * measured_distance
									+ -0.0001
											* (measured_distance
													* measured_distance));
			distance_to_beacon[current_channel] = distance;
		}
	}
	memset(receivedData, 0, sizeof receivedData);
}

void HAL_GPIO_EXTI_Callback(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	sendNewPacket = TRUE;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
