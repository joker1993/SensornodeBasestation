/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#define NODEID 1 // id of sensornode
#define SHT_ADDRESS (0x44 << 1) // I2C address of SHT31-DIS-B sensor
#define SHT_MEASURE_COMMAND 0x2400 // measurement command for SHT31-DIS-B sensor
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static uint8_t shtMeasureData[5]; // measurement data (temperature and humidity) from SHT31-DIS-B sensor
static uint16_t st; // raw sensor output for temperature
static uint16_t srh; // raw sensor output for humidity
static float temperature; // value of temperature
static uint8_t humidity; // value of humidity

static CanTxMsgTypeDef txMessage;
static CanRxMsgTypeDef rxMessage;
static CAN_FilterConfTypeDef  filterConfig;

uint8_t lock1 = 1;
uint8_t lock2 = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void esp8266Init();
void canInit();
void i2cInit();
void CANFilter(CAN_FilterConfTypeDef *filter);
float readTemperature();
uint8_t readHumidity();
void readCan();
void send(char topic[], float value);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  esp8266Init();
  canInit();
  i2cInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (lock1 == 0){
		  temperature = readTemperature();
		  send("sensornode/1/temperature", temperature);

		  readCan();
	  	  lock1 = 1;
	  }
	  if (lock2 == 0){
		  humidity = readHumidity();
		  send("sensornode/1/humidity", humidity);

		  readCan();
		  lock2 = 1;
	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
}

/* USER CODE BEGIN 4 */
void esp8266Init()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, SET); // regulator for ESP8266
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET); // enable ESP8266
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET); // set ESP 8266 reset pin to HIGH
	HAL_Delay(2000);
}

void canInit()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET); // enable regulator for CAN transceiver and SHT sensor
	hcan1.pTxMsg = &txMessage;
	hcan1.pRxMsg = &rxMessage;
	CANFilter(&filterConfig);
	HAL_CAN_Receive_IT(&hcan1,CAN_FIFO0);
}

void i2cInit()
{
	HAL_I2C_Mem_Read(&hi2c1, SHT_ADDRESS, SHT_MEASURE_COMMAND, 2, shtMeasureData, 5, 1000); // read from SHT sensor
}

void CANFilter(CAN_FilterConfTypeDef *filter)
{
	filter->FilterNumber = 0;
	filter->FilterMode = CAN_FILTERMODE_IDMASK;
	filter->FilterScale = CAN_FILTERSCALE_32BIT;
	filter->FilterIdHigh = 0x0000;
	filter->FilterIdLow = 0x0000;
	filter->FilterMaskIdHigh = 0x0000;
	filter->FilterMaskIdLow = 0x0000;
	filter->FilterFIFOAssignment = 0;
	filter->FilterActivation = ENABLE;
	filter->BankNumber = 14;

    HAL_CAN_ConfigFilter(&hcan1, filter);
}

float readTemperature()
{
	uint8_t ready = -1;
	uint8_t read = -1;
	float temperature;

//	HAL_I2C_Mem_Read(&hi2c1, SHT_ADDRESS, SHT_MEASURE_COMMAND, 2, shtMeasureData, 5, 1000);
	while (ready != 0 && read != 0){
		ready = HAL_I2C_IsDeviceReady(&hi2c1, SHT_ADDRESS, 3, 1000); // check connection to SHT sensor
		read = HAL_I2C_Mem_Read(&hi2c1, SHT_ADDRESS, SHT_MEASURE_COMMAND, 2, shtMeasureData, 5, 1000); // read from SHT sensor
	}

	st = shtMeasureData[0]*256 + shtMeasureData[1]; // raw sensor output for temperature
	temperature = (-45 + (float)(175*st)/65535); // value of temperature in Celsius

	return temperature;
}

uint8_t readHumidity()
{
	uint8_t ready = -1;
	uint8_t read = -1;
	uint8_t humidity;

	while (ready != 0 && read != 0){
		ready = HAL_I2C_IsDeviceReady(&hi2c1, SHT_ADDRESS, 3, 1000); // check connection to SHT sensor
		read = HAL_I2C_Mem_Read(&hi2c1, SHT_ADDRESS, SHT_MEASURE_COMMAND, 2, shtMeasureData, 5, 1000); // read from SHT sensor
	}

	srh = shtMeasureData[3]*256 + shtMeasureData[4]; // raw sensor output for humidity
	humidity = (uint32_t)((100*srh)/65535); // value of humidity in %

	return humidity;
}

void readCan()
{
	// format of received string: <node id>, <sensor type (t-temperature, h-humidity)>, <sensor value>
	char receiveString[30]; // message from another sensor node
	char topic[30]; // MQTT topic name
	char data[5]; // sensor value
	char sensorType; // sensor type (t-temperature, h-humidity)
	char* subString; // for split message
	char nodeId[3]; // id of sender node
	float temperature; // value of temperature
	int8_t humidity; // value of humidity

	strcpy(receiveString, (char *)hcan1.pRxMsg->Data);
	subString = strtok(receiveString, ",");
	strcpy(nodeId, subString);
	subString = strtok(NULL, ",");
	sensorType = *subString;
	subString = strtok(NULL, ",");
	strcpy(data, subString);

	strcpy(topic, "sensornode/");
	strcat(topic, nodeId);

	switch (sensorType)
	{
		case 't':
			strcat(topic, "/temperature");
			sscanf(data, "%f", (float *)&temperature);
			send(topic, temperature);
			break;
		case 'h':
			strcat(topic, "/humidity");
			sscanf(data, "%d", (int *)&humidity);
			send(topic, humidity);
			break;
	}
}

void send(char topic[], float value)
{
	char data[10];
	char payload[30];

	sprintf(data, "%.1f" , value);
	strcpy(payload, topic);
	strcat(payload, ": ");
	strcat(payload, data);
	strcat(payload, "\n");

	HAL_UART_Transmit(&hlpuart1, (uint8_t *)payload, strlen(payload), 1000);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
