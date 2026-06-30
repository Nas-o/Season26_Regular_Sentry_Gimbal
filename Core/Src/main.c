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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "iwdg.h"
#include "memorymap.h"
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* ---- BSP层: 底层通信驱动 ---- */
#include "UART_data_txrx.hpp"    // UART DMA 收发初始化
#include "CAN_receive_send.hpp"  // FDCAN 收发与电机/IMU数据解析
#include "BoardLink.hpp"         // 双板通信协议 (云台板?底盘板)

/* ---- 硬件设备驱动 ---- */
#include "motor.hpp"             // 电机统一驱动接口 (DJI/DM)
#include "supercup.hpp"          // 超级电容管理
#include "vofa+.hpp"             // VOFA+ 上位机调试协议
/* #include "IMU_updata.h" */    // 板载BMI088 IMU (已弃用, 改用外置达妙IMU)
#include "dm_imu.hpp"            // 达妙外置陀螺仪CAN驱动
#include "HI12.hpp"              // 超核HI12 IMU (备用)
#include "LED.hpp"               // LED/WS2812 状态灯
#include "Power_switch.hpp"      // 电源开关控制
#include "referee_system.hpp"    // 裁判系统串口协议

/* ---- Software层: 控制逻辑 ---- */
#include "music.hpp"             // 蜂鸣器音乐/提示音
#include "Chassis.hpp"           // 底盘控制器 (云台板侧指令生成)
#include "Gimbal.hpp"            // 云台控制器 (Pitch/Yaw 轴)
#include "Shoot.hpp"             // 发射机构控制器 (摩擦轮+拨弹)
#include "Auto_control.hpp"      // 自瞄/视觉联控
#include "Global_status.hpp"     // 全局状态机 (模式/开火许可等)

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

/* USER CODE END PV */
                    
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */ 
HAL_StatusTypeDef ITsta;              // HAL中断状态 (调试用)
extern DMA_HandleTypeDef hdma_uart7_rx; // UART7 DMA句柄 (裁判系统接收)
uint32_t fdcan_freq;                   // FDCAN 外设时钟频率 (用于波特率校验)
/* USER CODE END 0 */

/**\
  * @brief  The application entry point  .
  * * @retval intr
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_SPI6_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  MX_FDCAN3_Init();
  MX_OCTOSPI2_Init();
  MX_IWDG1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /*==========================================================
   * 用户外设初始化序列
   * 注意: 初始化顺序有依赖关系, 请勿随意调整!
   * 看门狗在初始化过程中需定期喂狗, 防止初始化耗时过长触发复位
   *==========================================================*/
  HAL_IWDG_Refresh(&hiwdg1);          // 喂狗: 防止后续初始化超时

  /* ---- 通信总线初始化 ---- */
  Uart_Init();                         // 初始化所有UART DMA接收 (遥控器/裁判系统/导航/视觉)
  HI12_Init(&hfdcan3);                 // 初始化超核HI12 IMU (挂载在FDCAN3)
  Can_Init();                          // 初始化FDCAN1/2/3滤波器并启动接收
  BoardLink_Init();                    // 初始化双板CAN通信协议
  Refree_system_init();                // 初始化裁判系统数据解析

  /* ---- IMU初始化 (达妙外置陀螺仪) ---- */
  imu_init(0x01, 0x11, &hfdcan1, &imu_gimbal);  // 云台IMU: 主机ID=0x01, 从机ID=0x11, 挂FDCAN1
  imu_init(0x02, 0x12, &hfdcan3, &imu_chassis); // 底盘IMU: 主机ID=0x02, 从机ID=0x12, 挂FDCAN3
  /* IMU_Init(); */                     // 板载BMI088 (已弃用)

  HAL_IWDG_Refresh(&hiwdg1);          // 喂狗: IMU初始化可能耗时

  /* ---- 电源与外设使能 ---- */
  HAL_GPIO_WritePin(Power_5V_GPIO_Port, Power_5V_Pin, GPIO_PIN_SET);  // 打开5V电源轨

  /* ---- 切换IMU为主动请求模式 (降低CAN总线负载) ---- */
  imu_change_to_request(&imu_gimbal);  // 云台IMU: 由广播→请求式
  imu_change_to_request(&imu_chassis); // 底盘IMU: 由广播→请求式

  /* ---- 相机触发与定时器启动 ---- */
  HAL_GPIO_WritePin(Camera_ctrl_GPIO_Port, Camera_ctrl_Pin, GPIO_PIN_SET); // 相机供电使能
  HAL_TIM_Base_Start_IT(&htim14);      // 启动TIM14: 200Hz 视觉数据发送 + 相机曝光同步
  HAL_TIM_Base_Start_IT(&htim13);      // 启动TIM13: 1kHz IMU数据匹配定时器

  /* USER CODE END 2 */

  /*==========================================================
   * FreeRTOS 调度器启动
   * 此后控制权转交给RTOS, main()不再返回
   * 各任务在 freertos.c 中定义:
   *   - Gimbal_Task  (1kHz): 云台控制循环
   *   - Chassis_Task (1kHz): 底盘指令生成
   *   - Shoot_Task   (1kHz): 发射机构控制
   *   - Remote_control_Task: 遥控器/键鼠解析
   *   - Referee_Task: 裁判系统数据处理
   *   - Motor_control_Task: CAN电机统一发送
   *==========================================================*/

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

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
  * @brief  系统时钟配置
  * @note   HSE=16MHz → PLL → SYSCLK=480MHz (STM32H723最高主频)
  *         AHB=240MHz, APB1/2/3/4=120MHz
  *         FDCAN时钟=120MHz, USB用HSI48=48MHz
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_1);
  fdcan_freq = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* 禁用MPU以进行配置 */
  HAL_MPU_Disable();

  /*--------------------------------------------------------------
   * Region 0: Flash区域 (0x0800_0000, 1MB)
   * - 只读, 可执行, 可缓存
   * - 保护固件代码不被意外写入
   *--------------------------------------------------------------*/
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x08000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_PRIV_RO;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /*--------------------------------------------------------------
   * Region 1-4: SRAM区域 (0x2000_0000, 128MB范围)
   * - 全访问权限, 禁用Cache
   * - 禁用Cache确保DMA一致性 (FDCAN/UART DMA访问此区域)
   * - SubRegionDisable=0xEE: 仅启用部分子区域
   *--------------------------------------------------------------*/
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128MB;
  MPU_InitStruct.SubRegionDisable = 0xEE;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM23 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  static int i = 0;  // 相机曝光触发翻转标志 (0/1交替产生方波)
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM23)
  {
    HAL_IncTick();   // SysTick 递增 (HAL时基, 1ms)
  }
  /* USER CODE BEGIN Callback 1 */
  uint8_t history = 4;

  /*--------------------------------------------------------------
   * TIM13 中断: 1kHz
   * 功能: 匹配/更新IMU数据, 为云台和底盘控制提供最新姿态
   * 注意: 此中断优先级应高于FreeRTOS任务, 保证IMU数据实时性
   *--------------------------------------------------------------*/
  if (htim->Instance == TIM13)
  {
    /* IMU_Updata(); */               // 板载BMI088 (已弃用)
    IMU_MatchData(&imu_chassis);      // 达妙外置IMU: 底盘姿态数据匹配
    IMU_MatchData(&imu_gimbal);       // 达妙外置IMU: 云台姿态数据匹配
  }

  /*--------------------------------------------------------------
   * TIM14 中断: 200Hz
   * 功能: 1) 向视觉MiniPC发送当前姿态数据 (STM32_to_MINIPC)
   *       2) 产生Camera_ctrl方波信号, 用于相机硬件触发曝光同步
   *          高电平期间发送数据, 低电平翻转 → 200Hz方波 = 100fps曝光
   *--------------------------------------------------------------*/
  else if (htim->Instance == TIM14)
  {
    if (i == 0)
    {
      HAL_GPIO_WritePin(Camera_ctrl_GPIO_Port, Camera_ctrl_Pin, GPIO_PIN_SET);  // 曝光信号拉高
      STM32_to_MINIPC();              // 发送姿态+状态数据给视觉上位机
    }
    else
    {
      HAL_GPIO_WritePin(Camera_ctrl_GPIO_Port, Camera_ctrl_Pin, GPIO_PIN_RESET); // 曝光信号拉低
      i = 0;
    }
  }
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
