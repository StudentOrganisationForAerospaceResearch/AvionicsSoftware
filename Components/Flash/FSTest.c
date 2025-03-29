///**
// ********************************************************************************
// * @file    FSTest.c
// * @author  spiro
// * @date    Jan 18, 2025
// * @brief
// ********************************************************************************
// */
//
///************************************
// * INCLUDES
// ************************************/
//
///************************************
// * PRIVATE MACROS AND DEFINES
// ************************************/
//
///************************************
// * VARIABLES
// ************************************/
//
///************************************
// * FUNCTION DECLARATIONS
// ************************************/
//
///************************************
// * FUNCTION DEFINITIONS
// ************************************/
//
///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2024 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "memorymap.h"
//#include "spi.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//
///* USER CODE BEGIN PV */
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
///* USER CODE BEGIN PFP */
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//
//  /* USER CODE BEGIN 1 */
//    const char* fn_templ1 = "F%u.tst";
//	const char* fn_templ2 = "R%u.tst";
//	char fn[32], fn2[32];
//	lfs_file_t fp;
//  /* USER CODE END 1 */
//
//  /* Enable the CPU Cache */
//
//  /* Enable I-Cache---------------------------------------------------------*/
//  SCB_EnableICache();
//
//  /* Enable D-Cache---------------------------------------------------------*/
//  SCB_EnableDCache();
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART1_UART_Init();
//  MX_SPI1_Init();
//  MX_TIM1_Init();
//
//  /* USER CODE BEGIN 2 */
//
//  HAL_TIM_Base_Start(&htim1);										// Not used
//
//  printf("\n\nlittlefs version %x\n",LFS_VERSION);
//
//  W25Q_Reset();
//
//  printf("Flash Identifier = 0x%08lx\n",W25Q_ReadID());
//
//  W25Q_ReadUniqueID();
//
//  printf("StatusReg1=%02x\n",W25Q_ReadStatus(1));
//  printf("StatusReg2=%02x\n",W25Q_ReadStatus(2));
//  printf("StatusReg3=%02x\n",W25Q_ReadStatus(3));
//
//  printf("\nRead SFDP Table:\n");
//  uint8_t sfdp[256]={0};
//  W25Q_ReadSFDP(sfdp);
//  printf("%c %c %c %c ",sfdp[0],sfdp[1],sfdp[2],sfdp[3]);
//  for (int i=4;i<256;i++) printf("%02x ",sfdp[i]);
//  printf("\n\n");
//
//
//  // test file system
//  printf("\n\n ********************* Mount lfs ***********************\n\n");
//  stmlfs_mount(true);
//
//  //---------------------------------------------------------------------------------------------
//  // We'll create 32 files, verify them, rename them, reverify, and delete them.
//  //---------------------------------------------------------------------------------------------
//
//  for (int i = 0; i < 32; i++) {
//
//	  sprintf(fn, fn_templ1,i);                                  	// Create file name string
//
//      int err = stmlfs_file_open(&fp, fn, LFS_O_WRONLY | LFS_O_CREAT);// Create the fp
//      if (err < 0) {
//          printf("open failed\n");
//          fflush(stdout);
//          Error_Handler();
//      }
//
//      printf("Write to File %s\n",fn);
//      if ((strlen(fn) + 1) != (uint32_t)stmlfs_file_write(&fp, fn, strlen(fn) + 1)) {// Write the file name to the file
//          printf("write fails\n");
//          fflush(stdout);
//          Error_Handler();
//      }
//
//      if (stmlfs_file_close(&fp)<0){                          		// flush and close the file
//          printf("closed failed\n");
//          fflush(stdout);
//          Error_Handler();
//      }
//  }
//
//  dump_dir();														// Show directory
//
//  //stmlfs_unmount();                                               	// Unmount & remount
//  //stmlfs_mount(false);
//
//  struct littlfs_fsstat_t stat;                                   	// Display file system sizes
//  stmlfs_fsstat(&stat);
//  printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);
//
//  for (int i = 0; i < 32; i++) {
//  	  sprintf(fn, fn_templ1, i);
//      sprintf(fn2, fn_templ2, i);
//
//      printf("Rename from %s to %s\n",fn,fn2);
//      if (stmlfs_rename(fn, fn2) < 0) {                           	// rename
//          printf("rename failed\n");
//          fflush(stdout);
//          Error_Handler();
//      }
//  }
//  dump_dir();														// Show directory
//
//  stmlfs_fsstat(&stat);                                           	// Display file system sizes
//  printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);
//
//  char buf[32];
//  for (int i = 0; i < 32; i++) {
//
//      sprintf(fn, fn_templ1, i);
//      sprintf(fn2, fn_templ2, i);
//
//      printf("Reopen Filename=%s\n",fn2);
//      int err = stmlfs_file_open(&fp, fn2, LFS_O_RDONLY);       	// verify the file's content
//      if (err < 0) {
//          printf("lfs open failed\n");
//          fflush(stdout);
//          Error_Handler();
//      } else {
//          stmlfs_file_read(&fp, buf, sizeof(buf));
//          if (strcmp(fn, buf) != 0) {
//              printf("lfs read failed\n");
//              fflush(stdout);
//              Error_Handler();
//          }
//          stmlfs_file_close(&fp);
//
//          if (stmlfs_remove(fn2) < 0) {                             // Delete the file
//              printf("remove failed\n");
//              fflush(stdout);
//              Error_Handler();
//          } else printf("File %s removed\n",fn2);
//      }
//  }
//  dump_dir();
//
//  stmlfs_fsstat(&stat);                                         	// Display file system sizes
//  printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,(int)stat.blocks_used);
//
//  stmlfs_unmount();                                             	// Release any resources we were using
//  printf("lfs test done\n");
//  fflush(stdout);
//
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//      HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port,BLUE_LED_Pin);
//	  HAL_Delay(300);
//
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Supply configuration update enable
//  */
//  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
//
//  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 5;
//  RCC_OscInitStruct.PLL.PLLN = 48;
//  RCC_OscInitStruct.PLL.PLLP = 2;
//  RCC_OscInitStruct.PLL.PLLQ = 3;
//  RCC_OscInitStruct.PLL.PLLR = 2;
//  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
//  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
//  RCC_OscInitStruct.PLL.PLLFRACN = 0;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
//                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
//  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///* USER CODE BEGIN 4 */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART1 and Loop until the end of transmission */
//  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//  SOAR_PRINT("Character Printed");
//
//
//  return ch;
//}
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
