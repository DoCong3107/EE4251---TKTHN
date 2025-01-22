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
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "lora.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>																// Uncomment for Reception



void SystemClock_Config(void);

lora_pins_t lora_pins;												// Structure variable for lora pins
lora_t lora;																	// Structure variable for lora																// character buffer
char msg[64], buf[80], data_receive[60];
int id_d,id_r,stt=0;
float t,h,x_r,y_r,x_d,y_d;
float t_min = 20,t_max=25, h_min=50, h_max=70;
uint32_t now_tick,tick;
uint8_t time_out =0;
void send_broadcast(void){
	sprintf(buf,"Open R");
	lora_begin_packet(&lora);
	lora_tx(&lora, (uint8_t *)buf, strlen(buf));
	lora_end_packet(&lora);
	sprintf(buf,"Da gui broadcast cho node\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}
bool check_format_register(const char *packet, int *id, float *x, float *y) {
    // Sử dụng sscanf để phân tích gói tin
    if (sscanf(packet, "%d %f %f", id, x, y) == 3) {
        // Nếu sscanf trả về 3, nghĩa là id, x, y được đọc thành công
        return true;
    }
    return false; // Nếu không đọc được đúng định dạng
}
void listen_register(void){
	uint8_t ret=0;
	now_tick=HAL_GetTick();
	while(!ret){
		if((HAL_GetTick()-now_tick)>3000){
			time_out =1;
			sprintf(buf,"time out on listen_register\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
			break;
		}
		/*sprintf(buf,"Chua nhan duoc dang ky nao\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);*/
		ret = lora_prasePacket(&lora);
		if(ret){
			uint8_t i = 0;
			while (lora_available(&lora)) {
			   buf[i] = lora_read(&lora);
			   i++;
			}
			buf[i] = '\0'; // Kết thúc chuỗi
			sprintf(data_receive,buf);
			if(check_format_register(data_receive, &id_r, &x_r, &y_r)){
				sprintf(buf,"Da nhan dang ky: %s\r\n",data_receive);
				HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
				stt++;
			}
			else{
				ret=0;
			}
		}
	}
}
void send_stt(void){
	char data_stt[50];
	sprintf(data_stt,"%d %d %.1f %.1f %.1f %.1f",id_r,stt,t_min,t_max,h_min,h_max);
	lora_begin_packet(&lora);
	lora_tx(&lora, (uint8_t *)data_stt, strlen(data_stt));
	lora_end_packet(&lora);
	sprintf(buf,"Da gui stt cho node: %s\r\n",data_stt);
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}
bool check_format_data(const char *packet, int *id, float *x,float *y,float *t, float *h) {
    // Sử dụng sscanf để phân tích gói tin
    if (sscanf(packet, "%d %f %f %f %f", id, x, y, t, h) == 5) {
        // Nếu sscanf trả về 5, nghĩa là id, x, y, t, h được đọc thành công
        return true;
    }
    return false; // Nếu không đọc được đúng định dạng
}
int listen_data(void){
	uint8_t ret=0;
	tick=HAL_GetTick();
	while(!ret){
		if((HAL_GetTick()-now_tick)>=3000){
			time_out=1;
			return 1;
		}
		if((HAL_GetTick()-tick)>=250) return 0;
		ret = lora_prasePacket(&lora);
		if(ret){
			uint8_t i = 0;
			while (lora_available(&lora)) {
				buf[i] = lora_read(&lora);
				i++;
			}
			buf[i] = '\0'; // Kết thúc chuỗi
			sprintf(data_receive,buf);
			if(check_format_data(data_receive, &id_d, &x_d,&y_d, &t, &h)){
				return 1;
			}
			else {
				sprintf(buf,"Data nhan duoc khong hop le\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
			}
		}
	}
}
void send_ok(void){
	sprintf(buf,"Nhan duoc data: %d %.1f %.1f %.1f %.1f\r\n",id_d,x_d,y_d,t,h);
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
	sprintf(buf,"Ok to %d",id_r);
	lora_begin_packet(&lora);
	lora_tx(&lora, (uint8_t *)buf, strlen(buf));
	lora_end_packet(&lora);
	sprintf(buf,"Da gui ok cho node\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Định nghĩa hàm kiểm tra định dạng

// Hàm xử lý gói tin đăng ký


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
// Main Function
int main(void)
{
  HAL_Init();																	// HAL library Initialization
  SystemClock_Config();												// System clock Initialization
  MX_GPIO_Init();															// GPIO Pins Initialization
  MX_SPI1_Init();															// SPI Communication Initialization
  MX_USART3_UART_Init(); 											// UART1 Communication Initialization

	//lora_pins.dio0.port  = LORA_DIO0_PORT;
	//lora_pins.dio0.pin   = LORA_DIO0_PIN;
	lora_pins.nss.port   = LORA_SS_PORT;				// NSS pin to which port is connected
	lora_pins.nss.pin    = LORA_SS_PIN;					// NSS pin to which pin is connected
	lora_pins.reset.port = LORA_RESET_PORT;			// RESET pin to which port is connected
	lora_pins.reset.pin  = LORA_RESET_PIN;			// RESET pin to which pin is connected
	lora_pins.spi  			 = &hspi1;

	lora.pin = &lora_pins;
	//lora.frequency = FREQ_433MHZ;							// 433MHZ Frequency
	lora.frequency = FREQ_433_02MHZ;						// 433.06MHZ Frequency
	//lora.frequency = FREQ_433_15MHZ;							// 433.15 MHZ Frequency
	//lora.frequency = FREQ_865MHZ;								// 865MHZ Frequency
	//lora.frequency = FREQ_866MHZ;								// 866MHZ Frequency
	//lora.frequency = FREQ_867MHZ;								// 867MHZ Frequency

	sprintf(msg,"Configuring LoRa module\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t *)msg,strlen(msg),1000);

	while(lora_init(&lora)){										// Initialize the lora module
		sprintf(msg,"LoRa Failed\r\n");
		HAL_UART_Transmit(&huart3,(uint8_t *)msg,strlen(msg),1000);
		HAL_Delay(1000);
	}
	sprintf(msg,"Done configuring LoRaModule\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t *)msg,strlen(msg),1000);

  while (1)																		// Inifinite Loop
  {
	  	time_out =0;
		send_broadcast();
		listen_register();
		if(time_out) continue;
		send_stt();
		while(!listen_data()){
			send_stt();
		}
		if(time_out) continue;
		send_ok();
		HAL_Delay(1000);
  }
}
// END OF Main

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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
