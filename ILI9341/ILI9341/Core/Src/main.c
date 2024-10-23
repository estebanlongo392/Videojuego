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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "bitmaps.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SPI_HandleTypeDef hspi1;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t option[1];
uint8_t tx_buffer[20];
extern uint8_t fondo[];
int incremento = 0;
int y_incre = 0;
int decremento = 1;
int wh1 = 0;
int wh2 = 0;
int win = 0;
int lo = 0;

#define IMAGE_SIZE 9968 //
uint8_t image[IMAGE_SIZE];  // Variable para almacenar los datos de la imagen

#define IMAGE_SIZE2 12716 //
uint8_t image2[IMAGE_SIZE2];  // Variable para almacenar los datos de la imagen
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int load_image_from_sd_to_play(const char* filename);
int load_image_from_sd_to_play2(const char* filename);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
int time = 0;
int time1 = -160;

int uni = 0;
int dec = 0;

int b = 0;

int y = -30;
int y1 = 40;
int yp = 110;

int z = 0;
int i = 0;

int GO = 0;
int punt = 0;

int posu[11] = {-50,-25,-10,50 ,-30,0  ,15,-50 ,10,35, -50};
int posd[11] = {20 ,50 ,10 ,-20,10,-40,-30,-10,50,-30,50};

// Montar SD

   fres = f_mount(&fs,"/" , 0);
   if(fres == FR_OK){
   //   transmit_uart("Micro SD is mounted successfully\n\n\n\n");
   } else {
    //   transmit_uart("Micro SD is mounted bad\n\n\n\n");
	   }


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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart3, option, 1);  // EMPEZAR RECEPCIÓN
  HAL_UART_Receive_IT(&huart5, option, 1);  // EMPEZAR RECEPCIÓN

	LCD_Init();

	LCD_Clear(0x00);
	FillRect(0, 0, 319, 239, 0x0000);

	LCD_Sprite(68, 20, 184, 50, titulo1, 1, 0, 0, 1);
	LCD_Sprite(68, 110, 184, 50, titulo2, 1, 0, 0, 1);
	LCD_Sprite(100, 190, 116, 20, titulo3, 1, 0, 0, 1);


	  //LCD_Print("FUCK YOU BITCH", 20, 100, 1, 0x001F, 0xCAB9);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		// Aumentar la velocidad de las tuberías

		while(wh1 == 1){

				LCD_Sprite(15, 30, 290, 25, cuan, 1, 0, 0, 1);
				LCD_Sprite(10, 110, 232, 12, p1, 1, 0, 0, 1);
				LCD_Sprite(10, 160, 285, 12, p2, 1, 0, 0, 1);

				while(wh2 == 1){
					wh1 = 0;
					time = time + 5;  // Incremento ajustado para mayor velocidad
					time1 = time1 + 5;  // Incremento ajustado para mayor velocidad

					b++;

					// TUBERIA
					FillRect(330-time, 151-y, 5, 90+y, 0x07E0);
					FillRect(330-time, 161-y, 5, 5, 0x0000);
					FillRect(365-time, 151-y, 5, 90+y, 0x0000);

					FillRect(330-time, 0, 5, 75-y, 0x07E0);
					FillRect(330-time, 60-y, 5, 5, 0x0000);
					FillRect(365-time, 0, 5, 75-y, 0x0000);


					FillRect(330-time1, 151-y1, 5, 90+y1, 0x07E0);
					FillRect(330-time1, 161-y1, 5, 5, 0x0000);
					FillRect(365-time1, 151-y1, 5, 90+y1, 0x0000);

					FillRect(330-time1, 0, 5, 75-y1, 0x07E0);
					FillRect(330-time1, 60-y1, 5, 5, 0x0000);
					FillRect(365-time1, 0, 5, 75-y1, 0x0000);
					// Reiniciar las tuberías cuando ya no se vean en la pantalla
					if(time >= 375){  // Ajuste del valor límite para evitar cortes
						time = 0;  // Reiniciar de forma continua y suave
						i++;
					y = posu[i];
					};

					if(time1 >= 375){  // Ajuste del valor límite para evitar cortes
						time1 = 0;  // Reiniciar de forma continua y suave
						z++;
						y1 = posd[z];
					};
					if(i == 10){
						i = 0;
					};
					if(z == 10){
						z = 0;
					};
									// Control de la animación del pájaro
					if(b == 2){
						b = 0;
					};


					LCD_Sprite(75, yp, 40, 30, bird40x30, 3, b, 0, 1);

					yp = yp+incremento;
										if(y_incre == 1){
											incremento++;

											if (decremento == 0){
												FillRect(75, yp+30, 40, 15, 0x0000);
											}else if (decremento == 1){
												FillRect(75, yp-30, 40, 27, 0x0000);
											}
											if(incremento == 0){
												decremento = 1;
											}
											if(yp >= 183){
												incremento = 0;
												y_incre = 0;
												yp = 183;
											}
										}

					// Detectar cuando las tuberías están en la misma posición que el pájaro
					if(time == 275){
						uni = uni + 1;
						transmit_uart3("s");
					};
					if(time1 == 275){
						uni = uni + 1;
					};
					if(uni == 10){
						uni = 0;
						dec = dec + 1;
					};
					LCD_Sprite(276, 0, 26, 39, num26x39, 10, uni, 0, 1);
					LCD_Sprite(250, 0, 26, 39, num26x39, 10, dec, 0, 1);

					if(time <= 245 && time >= 215 ){
						if(yp <= (60-y) || yp >= (170+y)){

							GO = 1;
							punt = uni + dec*10;


							wh2 = 0;
						};
					};
					if(time1 <= 245 && time1 >= 215 ){
						if(yp <= (60-y1) || yp >= (170+y1)){

							punt = uni + dec*10;
							GO = 1;
							wh2 = 0;
						};
					};
				}
		}
		if(GO == 1){
			FillRect(0, 0, 319, 239, 0x0000);
			switch (punt) {
										        case 0:
										            transmit_uart3("M: 0");
										            break;
										        case 1:
										            transmit_uart3("M: 1");
										            break;
										        case 2:
										            transmit_uart3("M: 2");
										            break;
										        case 3:
										            transmit_uart3("M: 3");
										            break;
										        case 4:
										            transmit_uart3("M: 4");
										            break;
										        case 5:
										            transmit_uart3("M: 5");
										            break;
										        case 6:
										            transmit_uart3("M: 6");
										            break;
										        case 7:
										            transmit_uart3("M: 7");
										            break;
										        case 8:
										            transmit_uart3("M: 8");
										            break;
										        case 9:
										            transmit_uart3("M: 9");
										            break;
										        case 10:
										            transmit_uart3("M: 10");
										            break;
										        case 11:
										            transmit_uart3("M: 11");
										            break;
										        case 12:
										            transmit_uart3("M: 12");
										            break;
										        case 13:
										            transmit_uart3("M: 13");
										            break;
										        case 14:
										            transmit_uart3("M: 14");
										            break;
										        case 15:
										            transmit_uart3("M: 15");
										            break;
										        case 16:
										            transmit_uart3("M: 16");
										            break;
										        case 17:
										            transmit_uart3("M: 17");
										            break;
										        case 18:
										            transmit_uart3("M: 18");
										            break;
										        case 19:
										            transmit_uart3("M: 19");
										            break;
										        case 20:
										            transmit_uart3("M: 20");
										            break;
										        case 21:
										            transmit_uart3("M: 21");
										            break;
										        case 22:
										            transmit_uart3("M: 22");
										            break;
										        case 23:
										            transmit_uart3("M: 23");
										            break;
										        case 24:
										            transmit_uart3("M: 24");
										            break;
										        case 25:
										            transmit_uart3("M: 25");
										            break;
										        case 26:
										            transmit_uart3("M: 26");
										            break;
										        case 27:
										            transmit_uart3("M: 27");
										            break;
										        case 28:
										            transmit_uart3("M: 28");
										            break;
										        case 29:
										            transmit_uart3("M: 29");
										            break;
										        case 30:
										        	transmit_uart3("M: 30");
										            break;
										        default:
										            break;
			}
			GO = 0;
		};
		if(win == 1){
			 if (load_image_from_sd_to_play("winner.bin")) {
				LCD_Sprite(71, 106, 178, 28, image, 1, 0, 0, 1);
				}
		};
		if(lo == 1){
			 if (load_image_from_sd_to_play2("loser.bin")) {
				LCD_Sprite(66, 103, 187, 34, image2, 1, 0, 0, 1);
			 	 }
		};
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int load_image_from_sd_to_play(const char* filename) {
    FIL fil;
    UINT bytes_read;
    FRESULT fres;

    // Abrir el archivo desde la SD
    fres = f_open(&fil, filename, FA_READ);
    if (fres != FR_OK) {
      //  transmit_uart("Error al abrir el archivo en la SD\n");
        return 0;  // Error al abrir el archivo
    }

    // Leer los datos del archivo y almacenarlos en la variable play
    fres = f_read(&fil, image, IMAGE_SIZE, &bytes_read);
    if (fres != FR_OK || bytes_read == 0) {
      //  transmit_uart("Error al leer la imagen desde la SD\n");
        f_close(&fil);
        return 0;  // Error al leer el archivo
    }

    // Cerrar el archivo
    f_close(&fil);
   // transmit_uart("Imagen cargada correctamente desde la SD a la variable play\n");
    return 1;
}
int load_image_from_sd_to_play2(const char* filename) {
    FIL fil;
    UINT bytes_read;
    FRESULT fres;

    // Abrir el archivo desde la SD
    fres = f_open(&fil, filename, FA_READ);
    if (fres != FR_OK) {
      //  transmit_uart("Error al abrir el archivo en la SD\n");
        return 0;  // Error al abrir el archivo
    }

    // Leer los datos del archivo y almacenarlos en la variable play
    fres = f_read(&fil, image2, IMAGE_SIZE2, &bytes_read);
    if (fres != FR_OK || bytes_read == 0) {
      //  transmit_uart("Error al leer la imagen desde la SD\n");
        f_close(&fil);
        return 0;  // Error al leer el archivo
    }

    // Cerrar el archivo
    f_close(&fil);
   // transmit_uart("Imagen cargada correctamente desde la SD a la variable play\n");
    return 1;
}

void transmit_uart3(char *message){
	HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}



  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	  if(huart->Instance == UART5){

	    		  switch (option[0]){

	    		  case 'M':
	    			  win = 1;
	    			  lo = 0;
	    			  transmit_uart3("r");
	    			  break;
	    		  case 'h':
	    			  lo = 1;
	    			  win= 0;
	    			  transmit_uart3("R");
	    			  break;
	    		  default:
	    			  break;
	    		  }

	    		  HAL_UART_Transmit(&huart5, tx_buffer, strlen(tx_buffer), 10);
	    	  	  HAL_UART_Receive_IT(&huart5, option, 1);


	    	  }

	  if(huart->Instance == USART3){

		  switch (option[0]){
		  case 'w':
			//strcpy(tx_buffer, "w");
			incremento= -8;
			y_incre = 1;
			decremento= 0;
			transmit_uart3("p");
			break;

		  case 'a':
			  wh1 = 1;
			  LCD_Clear(0x00);
			  FillRect(0, 0, 319, 239, 0x0000);
			  break;

		  case 'b':
			  wh2 = 1;
			  LCD_Clear(0x00);
			  FillRect(0, 0, 319, 239, 0x0000);
			  break;
		  case 'c':
			  wh2 = 1;
			  LCD_Clear(0x00);
			  FillRect(0, 0, 319, 239, 0x0000);
			  transmit_uart3("f");
			  break;

		  default:
			  break;
		  }

		  HAL_UART_Transmit(&huart3, tx_buffer, strlen(tx_buffer), 10);
	  	  HAL_UART_Receive_IT(&huart3, option, 1);


	  }
  }


  void transmit_uart5(char *message){
  	HAL_UART_Transmit(&huart5, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
  }
  void transmit_uart2(char *message){
  	HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
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
