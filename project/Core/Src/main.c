/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "xsense.h"
#include "CAN_IMU_Bridge.h"
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

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern TIM_HandleTypeDef htim6;
CAN_FilterTypeDef sFilterConfig;
uint8_t sgt_CAN_busy = 0;
uint8_t IMU_data_ready = 0;
uint8_t set_parameters = 0;

extern CAN_HandleTypeDef hcan1;
extern MCU_IMU_angular_velocity_TypeDef MCU_IMU_angular_velocity_Data;
extern MCU_IMU_acceleration_TypeDef MCU_IMU_acceleration_Data;
extern MCU_IMU_euler_angles_TypeDef MCU_IMU_euler_angles_Data;
extern MCU_IMU_gps_position_TypeDef MCU_IMU_gps_position_Data;
extern MCU_IMU_gps_speed_TypeDef MCU_IMU_gps_speed_Data;
extern struct XDI_RateOfTurnDataType XDI_RateOfTurn;
extern struct XDI_AccelerationDataType XDI_Acceleration;
extern struct XDI_EulerAnglesDataType XDI_EulerAngles;
extern struct XDI_VelocityDataType XDI_Velocity;
extern struct XDI_GPSPositionDataType XDI_GPSPosition;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void send_sgt_CAN(void);
void USART_ReInit(void);
static void MX_NVIC_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void initCanFilter(void)
{
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
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
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  MX_NVIC_Init();

  initCanFilter();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);								// enable idle line interrupt
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); 						// disable half complete interrupt
  HAL_UART_Receive_DMA(&huart2, XSENSE_rx_buffer, XSENSE_rx_buffer_size);	// start receiving data

  HAL_TIM_Base_Start_IT(&htim6);
  //HAL_TIM_Base_MspInit(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(IMU_data_ready)
	  {
		  send_sgt_CAN();
	  }

	  if(set_parameters)
	  {
		  USART_ReInit();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/* USER CODE BEGIN 4 */

void set_MCU_IMU_data(void)
{
	MCU_IMU_angular_velocity_Data.gyrX = XDI_RateOfTurn.gyrX * 100;
	MCU_IMU_angular_velocity_Data.gyrY = XDI_RateOfTurn.gyrY * 100;
	MCU_IMU_angular_velocity_Data.gyrZ = XDI_RateOfTurn.gyrZ * 100;

	MCU_IMU_acceleration_Data.accX = XDI_Acceleration.accx * 1000;
	MCU_IMU_acceleration_Data.accY = XDI_Acceleration.accy * 1000;
	MCU_IMU_acceleration_Data.accZ = XDI_Acceleration.accz * 1000;

	MCU_IMU_euler_angles_Data.pitch = XDI_EulerAngles.pitch * 100;
	MCU_IMU_euler_angles_Data.roll = XDI_EulerAngles.roll * 100;
	MCU_IMU_euler_angles_Data.yaw = XDI_EulerAngles.yaw * 100;

	MCU_IMU_gps_position_Data.lat = XDI_GPSPosition.latitude;
	MCU_IMU_gps_position_Data.longitude = XDI_GPSPosition.longitude;
	MCU_IMU_gps_speed_Data.gps_velocity = XDI_Velocity.velSum;

	IMU_data_ready = 1;
}

void send_sgt_CAN(void)
{
	static uint8_t CAN_Tx_msg_counter;

	sgt_CAN_busy = 1;

	Tx_MCU_IMU_angular_velocity_Data(&hcan1, &MCU_IMU_angular_velocity_Data);
	Tx_MCU_IMU_acceleration_Data(&hcan1, &MCU_IMU_acceleration_Data);
	Tx_MCU_IMU_euler_angles_Data(&hcan1, &MCU_IMU_euler_angles_Data);
	HAL_Delay(0.1);
	Tx_MCU_IMU_gps_position_Data(&hcan1, &MCU_IMU_gps_position_Data);
	Tx_MCU_IMU_gps_speed_Data(&hcan1, &MCU_IMU_gps_speed_Data);

	IMU_data_ready = 0;
	sgt_CAN_busy = 0;

	/* Blink IMU LED every 10 received messages */
	if (CAN_Tx_msg_counter > 10)
	{
		CAN_Tx_msg_counter = 0;
		HAL_GPIO_TogglePin(CAN_TX_L_GPIO_Port, CAN_TX_L_Pin);
	}
	else
	{
		CAN_Tx_msg_counter++;
	}
}

uint8_t is_sgt_CAN_busy()
{
	return sgt_CAN_busy;
}

void USART_ReInit(void)
{
    HAL_UART_Abort_IT(&huart2);
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_IDLE);

    HAL_UART_DeInit(&huart2);

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

    HAL_UART_Receive_DMA(&huart2, XSENSE_rx_buffer, XSENSE_rx_buffer_size);

    HAL_Delay(1000);
    set_parameters = 0;
	return;
}

/** NVIC Configuration
 */
static void MX_NVIC_Init(void) {
	/* USART2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

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
