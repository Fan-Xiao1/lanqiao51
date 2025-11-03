
  
#include "main.h"
#include "gpio.h"
#include "led.h"
#include "key.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "dac.h"
#include "usart.h"
#include "string.h"
#include "tim.h"
u8 Interface;
u8 Number_C=0,Number_V=0,Number_I=8;
float Number_C_dat=3.5,Number_V_dat=2;


void SystemClock_Config(void);

__IO uint32_t ledTick = 0; 
u8 led_ctrl = 0xff; 
void LED_Process(void) 
{ 
	if(uwTick - ledTick < 100) return ; 
	ledTick = uwTick; 
	LED_Control(led_ctrl); 
	led_ctrl = ~led_ctrl; 
}

// 按键执行程序
__IO uint32_t keyTick = 0;
u16 key1_val = 0;
void Key_Process(void)
{
	if(uwTick - keyTick < 10) return ; 
	keyTick = uwTick; 
	
	Key_Read();
	if(Trg & 0x01)	//B1
	{
		if(Interface==0x00)
		{
			LCD_Clear(Blue);
			Interface=0x10;
		}
		else
		{
			LCD_Clear(Blue);
			Interface=0x00;
		}
	}
	if(Trg & 0x02)	//B2
	{
		if(Interface==0x10)
		{
		Number_C_dat += 0.5;
		Number_V_dat += 0.5;
		}
	}
	if(Trg & 0x04)	//B3
	{
		if(Interface==0x10)
		{
		Number_C_dat -= 0.5;
		Number_V_dat -= 0.5;
		}
	}
	if(Trg & 0x08)	//B4
	{
		if((TIM17->ARR == 499)&&(TIM17->CCR1 == 100))
		{
			
			TIM17->CCR1 = 0;
		}
		else if((TIM17->ARR == 499)&&(TIM17->CCR1 == 0))
		{
			TIM17->CCR1 = 100;
		}
	}
}

// ADC执行程序
u16 adc1_val,adc2_val;
float volt_r37,volt_r38,volt_mcp;
void ADC_Process(void)
{
	//RANK1 - CH5
	HAL_ADC_Start(&hadc1);
	volt_mcp = HAL_ADC_GetValue(&hadc1)/4096.0f*3.3f;
	//RANK2 - CH11
	HAL_ADC_Start(&hadc1);
	adc1_val = HAL_ADC_GetValue(&hadc1);
	volt_r38 = adc1_val/4096.0f*3.3f;
	
	//ADC2的采集
    HAL_ADC_Start(&hadc2);
	adc2_val = HAL_ADC_GetValue(&hadc2);
	volt_r37 = adc2_val/4096.0f*3.3f;
}


// EEPROM
u8 val_24c02=0;
u8 startup_times=0;
//MCP4017
u8 val_mcp=0;
//RTC
RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;
void RTC_Process()
{
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
}
//DAC
u16 dac_ch1_val,dac_ch2_val;
void DAC_Process()
{  
	dac_ch1_val = (1.1f/3.3f*4095);
	dac_ch2_val = (2.5f/3.3f*4095);
	
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_ch1_val);	//0-->0v  4095--> 3.3V    1.1v --> 1365
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	  
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_ch2_val);	//0-->0v  4095--> 3.3V    2.2v --> 2730
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
}
//串口接收
u8 uart_buf[2];
u8 rx_buf[100];
u8 rx_cnt = 0;
__IO uint32_t uartTick = 0;
void RxIdle_Process()
{
	if(uwTick - uartTick < 50) return ; 
	uartTick = uwTick; 
	//50ms执行一次
	rx_cnt = 0;
	memset(rx_buf,'\0',sizeof(rx_buf));		//清空接收缓存数组
}

//PWM捕获
u32 tim2_cnt1 = 0, tim2_cnt2 = 0;
u32 f40 = 0;
float d40 = 0;



// LCD执行程序
__IO uint32_t lcdTick = 0;

void LCD_Process(void)
{
	if(uwTick - lcdTick < 50) return ; 
	lcdTick = uwTick; 
	
	u8 display_buf[20];
	if(Interface==0X00)
	{
	
	sprintf((char*)display_buf,"       Data");
	LCD_DisplayStringLine(Line1,display_buf);
	
	
	sprintf((char*)display_buf,"   CNBR:%d",Number_C);
	LCD_DisplayStringLine(Line3,display_buf);
	
		sprintf((char*)display_buf,"   VNBR:%d",Number_V);
	LCD_DisplayStringLine(Line5,display_buf);
	
		sprintf((char*)display_buf,"   IDLE:%d",Number_I);
	LCD_DisplayStringLine(Line7,display_buf);
	}
	else
	{
		
		sprintf((char*)display_buf,"       Para");
	LCD_DisplayStringLine(Line1,display_buf);
		
		sprintf((char*)display_buf,"   CNBR:%-5.2f",Number_C_dat);
	LCD_DisplayStringLine(Line5,display_buf);
	
		sprintf((char*)display_buf,"   VNBR:%-5.2f",Number_V_dat);
	LCD_DisplayStringLine(Line7,display_buf);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  
  HAL_Init();

  
  SystemClock_Config();

  
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_DAC1_Init();
  MX_USART1_UART_Init();
  HAL_UART_Receive_IT(&huart1,uart_buf,1);	//开启串口接收中断
  MX_TIM2_Init();
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);//开启TIM2_CH1的输入捕获中断
  MX_TIM3_Init();
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);//开启TIM3_CH1的输入捕获中断
  MX_TIM17_Init();
  HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
  TIM17->ARR = 499;	// 周期是500us，对应频率2kHz
  TIM17->CCR1 = 0; // 80%占空比
  
  MX_TIM16_Init();
  HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
//  TIM16->ARR = 49;	// 周期是50us，对应频率20kHz
//  TIM16->CCR1 = 10; // 20%占空比
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
	LED_Control(0x00);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Blue);
	LCD_SetBackColor(Blue);
	LCD_SetTextColor(White);
	
	
	//EEPROM
	I2CInit();
	startup_times = EEPROM_Read(0x20);
	EEPROM_Write(0x20,++startup_times);
	
	//MCP4017
	MCP4017_Write(0x7f);
	val_mcp = MCP4017_Read();
	
	//串口发送
	printf("Hello World!\r\n");
  while (1)
  {
	/* USER CODE END WHILE */
	/* USER CODE BEGIN 3 */
	// LED_Process();
	Key_Process();
	ADC_Process();
	RTC_Process();
	DAC_Process();
	LCD_Process();
	RxIdle_Process();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_USART1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//串口接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartTick = uwTick; 		//重新开始计时50ms
	//接收3个字节长度数据，rx_buf[0]写入EEPROM，rx_buf[1]控制LED，...
	rx_buf[rx_cnt++] = uart_buf[0];	
	if(uart_buf[0] == '\n')		//接收到换行符 \r \n
	{
		rx_cnt = 0;
		LED_Control(rx_buf[1]);
	}
	HAL_UART_Receive_IT(&huart1,uart_buf,1);	//开启下一次串口接收
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
