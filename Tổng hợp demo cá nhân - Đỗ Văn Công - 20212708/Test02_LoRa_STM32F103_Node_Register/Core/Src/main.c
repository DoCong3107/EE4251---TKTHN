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
#include <stdbool.h>
#include "DHT.h"
#define TX

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
char msg[64],buf[60];
char data_receive[80];
typedef struct {
    int id ;     // ID của Node
    float x;    // Tọa độ x
    float y;    // Tọa độ y
} Node;
Node node ={1,10,10};
int id_receive,stt;
int timeout_listen_request_data =0;
int restart = 0;
// Biến toàn cục để lưu RTC handler
RTC_HandleTypeDef hrtc;
uint32_t now_tick;
float t_min, t_max, h_min, h_max,tem=22,humi=60,t,h;
void Read_DataDHT(void);
 DHT_DataTypedef DHT22_Data;
 float T = 0, H = 0;
/* USER CODE BEGIN PV */
#ifdef	__GNUC__
		#define PUTCHAR_PROTOTYPE int	__io_putchar(int ch)
#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
 PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch,1,0xFFFF);
	return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
lora_pins_t lora_pins;												// Structure variable for lora pins
lora_t lora;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void listen_Broadcast(void){
//cấu hình lora
	//lora_pins.dio0.port  = LORA_DIO0_PORT;
	//lora_pins.dio0.pin   = LORA_DIO0_PIN;
	lora_pins.nss.port   = LORA_SS_PORT;				// NSS pin to which port is connected
	lora_pins.nss.pin    = LORA_SS_PIN;					// NSS pin to which pin is connected
	lora_pins.reset.port = LORA_RESET_PORT;			// RESET pin to which port is connected
	lora_pins.reset.pin  = LORA_RESET_PIN;			// RESET pin to which pin is connected
	lora_pins.spi  			 = &hspi1;

	lora.pin = &lora_pins;
	lora.frequency = FREQ_433MHZ;								// 433MHZ Frequency
	//lora.frequency = FREQ_865MHZ;								// 865MHZ Frequency
	//lora.frequency = FREQ_866MHZ;								// 866MHZ Frequency
	//lora.frequency = FREQ_867MHZ;								// 867MHZ Frequency
	lora.bandwidth = BW_62_5KHz;
	sprintf(msg,"Configuring LoRa module\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t *)msg,strlen(msg),1000);

	while(lora_init(&lora)){										// Initialize the lora module
		sprintf(msg,"LoRa Failed\r\n");
		HAL_UART_Transmit(&huart3,(uint8_t *)msg,strlen(msg),1000);
		HAL_Delay(1000);
	}
	sprintf(msg,"Done configuring LoRaModule\r\n");
	HAL_UART_Transmit(&huart3,(uint8_t *)msg,strlen(msg),1000);
	uint8_t ret =0;
	while(!ret){
	  ret = lora_prasePacket(&lora);
	  if(ret){
	   //Nhận gói tin
	   uint8_t i = 0;
	    while (lora_available(&lora)) {
	   buf[i++] = lora_read(&lora);
	   }
	   buf[i] = '\0'; // Kết thúc chuỗi
	   sprintf(data_receive,"Da nhan duoc broadcast: %s\r\n",buf);
	   HAL_UART_Transmit(&huart3, (uint8_t*)data_receive, strlen(data_receive), HAL_MAX_DELAY);
	   sprintf(data_receive,buf);
	   if(!strcmp(data_receive, "Open")){
	   break;
	   }
	   else{
		   ret =0;
	   }
	  }
	}
}
void send_register(void){
	char data_register[30];
	sprintf(data_register, "%d %.1f %.1f",node.id,node.x,node.y);
	// Gửi gói tin đăng ký
	lora_begin_packet(&lora);
	lora_tx(&lora, (uint8_t *)data_register, strlen(data_register));
	lora_end_packet(&lora);
	sprintf(buf,"Da gui thong tin dang ky len Gateway\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 1000);
}
bool check_and_parse_STT(const char *packet, int *id, int *stt,
                            float *t_min, float *t_max, float *h_min, float *h_max) {
    // Kiểm tra định dạng gói tin
    if (sscanf(packet, "%d %d %f %f %f %f", id, stt, t_min, t_max, h_min, h_max) == 6) {
        // Kiểm tra tính hợp lệ của các giá trị
        return true; // Gói tin hợp lệ
    }
    return false; // Gói tin không hợp lệ
}
int listen_STT(void){
	uint8_t ret =0;
	now_tick = HAL_GetTick();
	while(!ret){
		if ((HAL_GetTick()-now_tick)>=60) //lắng nghe STT trong 100ms
			return 0;
		ret = lora_prasePacket(&lora);
		if(ret){
			//Nhận gói tin
			uint8_t i = 0;
			while (lora_available(&lora)) {
				buf[i++] = lora_read(&lora);
			}
			buf[i] = '\0'; // Kết thúc chuỗi
			sprintf(data_receive,buf);
			if(check_and_parse_STT(data_receive, &id_receive, &stt, &t_min, &t_max, &h_min, &h_max)){
				if(node.id == id_receive){
					sprintf(buf,"Nhan duoc STT: %s\r\n",data_receive);
					HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 1000);
					return 1;
				}
				else{
					sprintf(buf,"STT nhan ve co id khong khop\r\n");
					HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 1000);
					ret=0;
				}
		    }
			else{
				sprintf(buf,"STT nhan duoc khong hop le\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 1000);
				ret =0;
			}
		}
	}

//Hàm check gói tin yêu cầu gửi data từ GW
bool check_request_data(const char *str, int targetId) {
	int id;
	char r[2]; // Để chứa 'R' và ký tự kết thúc '\0'

	// Sử dụng sscanf để tách chuỗi và kiểm tra
	if (sscanf(str, "%d %1[R]", &id, r) == 2) {
	// Kiểm tra ký tự 'R' và so sánh id với targetId
		if (strcmp(r, "R") == 0 && id == targetId) {
			return true;
		}
	}
	    return false;
}
int listen_request_data(void){
	uint8_t ret=0;
	now_tick=HAL_GetTick();
	while(!ret){
		if((HAL_GetTick()-now_tick>=36000)){
			restart=1;
			return 0;
		}
		ret = lora_prasePacket(&lora);
		if(ret){
			//Nhận gói tin
			uint8_t i = 0;
			while (lora_available(&lora)) {
				buf[i++] = lora_read(&lora);
			}
				buf[i] = '\0'; // Kết thúc chuỗi
				sprintf(data_receive,buf);
				if(check_request_data(data_receive, node.id)){
					return 1;
				}
				else{
					ret=0;
				}
		}
	}
}
void Read_DataDHT(void)
{
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	DHT_GetData(&DHT22_Data);
	T = DHT22_Data.Temperature;
	H= DHT22_Data.Humidity;
	if(T > t_max)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
	}
	  else
	  {
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET);
	  }
	if(T < t_min)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
	}
	  else
	  {
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
	  }
	if (T >= t_min && T <= t_max)
	{
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
	}
	  else
	  {
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
	  }
}

void send_data(void){
	char data[60],buf[60];
	sprintf(data,"%d %.1f %.1f %.1f %.1f",node.id,node.x,node.y,T,H);
	// Gửi gói tin data
	lora_begin_packet(&lora);
	lora_tx(&lora, (uint8_t *)data, strlen(data));
	lora_end_packet(&lora);
	sprintf(buf,"Da gui du lieu t&h len Gateway\r\n");
	HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), 1000);
}

int listen_ok(void){
	uint8_t ret =0;
	now_tick=HAL_GetTick();
	while(!ret){
		if((HAL_GetTick()-now_tick)>=60){
			sprintf(buf,"time out on listen_ok\r\n");
			HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf),1000);
			return 0;
		}
		ret = lora_prasePacket(&lora);
		if(ret){
		//Nhận gói tin
			uint8_t i = 0;
			while (lora_available(&lora)) {
				buf[i++] = lora_read(&lora);
			}
			buf[i] = '\0'; // Kết thúc chuỗi
			sprintf(data_receive,buf);
			sprintf(buf,"%d ACK",node.id);
			if(!strcasecmp(data_receive,buf)){
				sprintf(buf,"Da nhan duoc Ok tu GW\r\n");
				HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf),1000);
				return 1;
			}
			else ret=0;
		}
	}
}
void send_ACK(void){
	uint8_t ret =0;
	now_tick=HAL_GetTick();
}

// Hàm đưa vi điều khiển vào chế độ ngủ trong thời gian tùy chọn (tính bằng giây)
void Enter_StopMode(uint32_t sleep_time) {
    // Tính thời gian (RTC wakeup hoạt động với LSE 32.768 kHz)
    uint32_t counter_value = sleep_time;

    // Đảm bảo thời gian không vượt quá giá trị lớn nhất của RTC
    if (counter_value > 0xFFFF) {
        counter_value = 0xFFFF;
    }

    // Cấu hình RTC Alarm để thức dậy
    HAL_RTCEx_SetSecond_IT(&hrtc); // Đặt ngắt RTC khi đến thời điểm thức dậy

    // Xóa cờ Wakeup
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    // Đưa vào chế độ STOP
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    // Sau khi thức dậy, cần cấu hình lại clock hệ thống
    SystemClock_Config();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	  HAL_Init();																	// HAL library Initialization
	  SystemClock_Config();												// System clock Initialization
	  MX_GPIO_Init();															// GPIO Pins Initialization
	  MX_SPI1_Init();															// SPI Communication Initialization
	  MX_USART3_UART_Init(); 											// UART1 Communication Initializatio
	  RTC_Config();
  while (1)
  {

	  int count_send_register =0;
	  int count_send_data =0;
	  restart =0;
	  listen_Broadcast();
	  send_register();
	  while(!listen_STT()){
		  if(count_send_register>3 ){
			restart =1;
			break;
		  }
		  else{
			  send_register();
			  count_send_register++;
		  }
	  }
	  if(restart) continue;
	  listen_request_data();
	  if(restart) continue;
	  Read_DataDHT();
	  send_data();
	  while(!listen_ok()){
		  if(count_send_data>3){
			  restart=1;
			  break;
		  }
		  else {
			  send_data();
			  count_send_data++;
		  }
	  }
	  if(restart) continue;
	  HAL_Delay(4000);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void RTC_Config(void) {
    // Kích hoạt đồng hồ cho RTC và PWR
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();

    // Cho phép ghi vào thanh ghi backup
    HAL_PWR_EnableBkUpAccess();

    // Cấu hình nguồn xung RTC (LSE)
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Kết nối LSE với RTC
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    // Khởi tạo RTC
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = 0x7FFF; // Giá trị Prescaler cho LSE (32768 Hz)
    HAL_RTC_Init(&hrtc);
}
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
