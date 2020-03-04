/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <FreeRTOS.h>
#include <task.h>
#include "aruna.h"
#include "aruna/log.h"
#include "aruna/comm.h"
//#include <aruna/control.h>
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
using namespace aruna;
comm::Link *uart_driver;
//control::Actuator* l293d_driver;
comm::Link *rs485_driver;
aruna::log::channel_t *example_log;

extern "C" void app_main(void);
void testUART_task(void *param);
void start_comm();
void register_drivers();

void comm_test_task(void *arg) {
    comm::transmitpackage_t d;
    comm::channel_t testapp = {
            .port = 1,
    };
    example_log->info("comm register: %s", err_to_char.at(testapp.register_err));
    example_log->info("comm send: %s", err_to_char.at(testapp.send( 0xAA, (uint8_t *) "precies 32 bytes aan data size!\n", 32, 0)));
    example_log->info("comm send: %s", err_to_char.at(testapp.send( 0xAE, (uint8_t *) "een overflow van data! ohh nee, wat gebeurt er nu?\n", 51, true)));
    example_log->info("comm send: %s", err_to_char.at(testapp.send( 0xDE, (uint8_t *) "My man!\n", 8, true)));

    while (1) {
        if (testapp.receive(&d)) {
            example_log->info("howdy, partner!");
            example_log->info("data: '%s'", d.data_received);
            example_log->info("data_length: '%d'", d.data_lenght);
            example_log->info("from: '%d'", d.from_port);
            example_log->info("me: '%d'", d.to_port);
            testapp.receive_clear();
        }
    }
    vTaskDelete(NULL);
}


extern "C" void app_main(void) {
//    change to VERBOSE enable logging.
    log::set_max_level(log::level_t::VERBOSE);
//
    example_log = new log::channel_t("aruna_example");
    log::set_level("comm", log::level_t::VERBOSE);
    log::set_level("aruna_example", log::level_t::VERBOSE);
//    esp_log_level_set("UART", ESP_LOG_VERBOSE);
    example_log->info("hello world!");
    register_drivers();
    start_comm();

//    control::start();
//	control::set_X_velocity(100, direction_t::PLUS);

//    test application

//		TODO task overflow?!
    xTaskCreate(comm_test_task, "comm test task", 2048, NULL, 0, NULL);
//    xTaskCreate(aruna::blinky::start_blinky_task, "blinky_app", 2048, NULL, 0, NULL);

}

void register_drivers() {
//    TODO if there are more drivers, then this needs his own .cpp file

//  comm
// TODO error check
//    uart_driver = new comm::UART();
//    comm::register_candidate_driver(uart_driver);
//
//    uart_config_t rs485_config = {
//            .baud_rate = 5000000,
//            .data_bits = UART_DATA_8_BITS,
//            .parity = UART_PARITY_DISABLE,
//            .stop_bits = UART_STOP_BITS_1,
//            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//            .rx_flow_ctrl_thresh = 122,
//            .use_ref_tick = false
//    };
//	rs485_driver = new aruna::comm::UART((char*)"RS485",
//							UART_NUM_1,
//							23,
//							22,
//							18,
//							UART_PIN_NO_CHANGE,
//							rs485_config,
//							UART_MODE_RS485_HALF_DUPLEX,
//							256,
//							512);
//  comm::register_candidate_driver(rs485_driver);


//  Control
// TODO error check
//    l293d_driver = new control::Pwm(control::axis_mask_t::X,(gpio_num_t) 32,(gpio_num_t) 33, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B);
//    aruna::control::register_driver(l293d_driver);
}

void start_comm() {
    example_log->debug("comm start: %s", err_to_char.at(comm::start(uart_driver)));
    example_log->debug("comm status: %d", comm::get_status());
    example_log->debug("comm driver name: %s", comm::getName());
    example_log->debug("comm speed: %i", comm::get_speed());
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
extern "C" int main(void)
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  app_main();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
