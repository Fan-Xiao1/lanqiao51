

/* Includes ------------------------------------------------------------------*/
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

float volt_r37,volt_r38,volt_mcp;

u8 Interface = 0X00;
float R37_std_l=1.2,R37_std_h=2.2;
float R38_std_l=1.4,R38_std_h=3.0;
float PR37=0,PR38=0;
u8 R37_Pass_Data=0,R37_Sum_Data=0;
u8 R38_Pass_Data=0,R38_Sum_Data=0;
u8 uart_Flag=0;
u8 LED_Para=0X04;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


// LED执行程序
__IO uint32_t ledTick = 0; 
u8 led_ctrl = 0xff; 
void LED_Process(void) 
{ 
	if(uwTick - ledTick < 100) return ; 
	ledTick = uwTick; 
	LED_Control(LED_Para);
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
		if(Interface==0X00)
		{
			
			LCD_Clear(Black);
			LED_Para=0X08;
			Interface = 0X10;
//			LED_Control(LED_Para);
		}
		else if((Interface>>4)==0x1)
		{
			
			LCD_Clear(Black);
			LED_Para=0X10;
			Interface = 0X20;
//			LED_Control(LED_Para);
		}
		else if(Interface == 0X20)
		{
			
			LCD_Clear(Black);
			LED_Para=0X04;
			Interface = 0X00;
//			LED_Control(LED_Para);
		}
	}
	if(Trg & 0x02)	//B2
	{
		
		if(Interface == 0X00)
		{
			if((volt_r37>=R37_std_l)&&(volt_r37<=R37_std_h))
			{
				R37_Pass_Data++;
				LED_Para |= 0X01; // 假设LED1对应的是第0位
            LED_Control(LED_Para);
            HAL_Delay(1000);
            LED_Para &= 0x04; // 清除LED1对应的位
            LED_Control(LED_Para);
			}
			R37_Sum_Data++;
			PR37=(float)R37_Pass_Data/R37_Sum_Data*100;
		}
		else if((Interface>>4)==0x1)
		{
			if(Interface==0x10)
			{
				Interface=0x11;
			}
			else if(Interface==0x11)
			{
				Interface=0x12;
			}
			else if(Interface==0x12)
			{
				Interface=0x13;
			}
			else if(Interface==0x13)
			{
				Interface=0x10;
			}
			
	   }
   }
	if(Trg & 0x04)	//B3
	{
		if(Interface == 0X00)
		{
			if((volt_r38>=R38_std_l)&&(volt_r38<=R38_std_h))
			{
				R38_Pass_Data++;
				
				LED_Para |= 0X01; // 假设LED1对应的是第0位
            LED_Control(LED_Para);
            HAL_Delay(1000);
            LED_Para &= ~0X01; // 清除LED1对应的位
            LED_Control(LED_Para);
			}
			R38_Sum_Data++;
			PR38=(float)R38_Pass_Data/R38_Sum_Data*100;
		}
		else if((Interface>>4)==0x1)
		{
			if(Interface==0x10)
			{
				R37_std_h =R37_std_h + 0.2;
				if(R37_std_h>3)
					R37_std_h=2.2;
			}
			else if(Interface==0x11)
			{
				R37_std_l =R37_std_l + 0.2;
				if(R37_std_l>2)
					R37_std_l=1.2;
			}
			else if(Interface==0x12)
			{
				R38_std_h += 0.2;
				if(R38_std_h>3)
					R38_std_h=2.2;
			}
			else if(Interface==0x13)
			{
				R38_std_l += 0.2;
				if(R38_std_l>2)
					R38_std_l=1.2;
			}
			R37_Pass_Data=0;
			R37_Sum_Data=0;
		}
		
	}
	if(Trg & 0x08)	//B4
	{
		 if((Interface>>4)==0x1)
		{
			if(Interface==0x10)
			{
				R37_std_h -= 0.2;
				if(R37_std_h<2.2)
					R37_std_h=3;
			}
			else if(Interface==0x11)
			{
				R37_std_l -= 0.2;
				if(R37_std_l<1.2)
					R37_std_l=2;
			}
			else if(Interface==0x12)
			{
				R38_std_h -= 0.2f;
				if(R38_std_h<2.2f)
					R38_std_h=3;
			}
			else if(Interface==0x13)
			{
				R38_std_l -= 0.2;
				if(R38_std_l<1.2f)
					R38_std_l=2;
			}
			R38_Pass_Data=0;
			R38_Sum_Data=0;
		}
		else if((Interface>>4)==0x2)
		{
			R37_Pass_Data=0;
			R38_Pass_Data=0;
			R37_Sum_Data=0;
			R38_Sum_Data=0;
			PR38=0;
			PR37=0;
		}
	}
     
}

// ADC执行程序
u16 adc1_val,adc2_val;

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
u8 rx_buf[10];
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
// LCD执行程序
__IO uint32_t LCDTick = 0;
void LCD_Process(void)
{
	if(uwTick - LCDTick  < 100) return ; 
	LCDTick  = uwTick; 
	
	u8 display_buf[20];
//	PR37=R37_Pass_Data/R37_Sum_Data;
	
	if(Interface == 0X00)
	{
//		LED_Control(0x04);
		
	sprintf((char*)display_buf,"       GOODS      ");
	LCD_SetBackColor(Yellow);
	LCD_DisplayStringLine(Line1,display_buf);
	LCD_SetBackColor(Black);
		
	sprintf((char*)display_buf,"     R37:%4.2f",volt_r37);
	LCD_DisplayStringLine(Line3,display_buf);
	
	sprintf((char*)display_buf,"     R38:%4.2f",volt_r38);
	LCD_DisplayStringLine(Line4,display_buf);
	}
	
	else if((Interface>>4) == 0X1)
	{
//		LED_Control(0x08);
		sprintf((char*)display_buf,"      STANDARD");
	LCD_DisplayStringLine(Line1,display_buf);
		
		sprintf((char*)display_buf,"    SR37:%2.1f-%2.1f",R37_std_l,R37_std_h);
	LCD_DisplayStringLine(Line3,display_buf);
		
		sprintf((char*)display_buf,"    SR38:%2.1f-%2.1f",R38_std_l,R38_std_h);
	LCD_DisplayStringLine(Line4,display_buf);
	}
	else if(Interface == 0X20)
	{
//		LED_Control(0x10);
		sprintf((char*)display_buf,"        PASS");
	LCD_DisplayStringLine(Line1,display_buf);
		
		sprintf((char*)display_buf,"     PR37:%04.1f%%",PR37);
	LCD_DisplayStringLine(Line3,display_buf);
		
		sprintf((char*)display_buf,"     PR38:%04.1f%%",PR38);
	LCD_DisplayStringLine(Line4,display_buf);
	}
}

void uart_Process()
{
	if(uart_Flag==1)
	{
	  if(rx_buf[2]=='7')
	  {
		  printf("R37:%d,%d,%4.1f%%\r\n",R37_Pass_Data,R37_Sum_Data,PR37);
	  }
	  else if(rx_buf[2]=='8')
	  {
		printf("R37:%d,%d,%4.1f%%\r\n",R37_Pass_Data,R37_Sum_Data,PR37);
	  }
	  uart_Flag=0;
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
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
	LED_Control(0x00);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	
	//EEPROM
	I2CInit();
	startup_times = EEPROM_Read(0x20);
	EEPROM_Write(0x20,++startup_times);
	
	//MCP4017
	MCP4017_Write(0x7f);
	val_mcp = MCP4017_Read();
	
	//串口发送
//	printf("Hello World!\r\n");

LED_Control(0x04);
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
	  uart_Process();
	  LED_Process(); 
	  
	  
//	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);	//打开锁存器
//	
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);  
//	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
  }
  /* USER CODE END 3 */
  
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
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
	if(rx_cnt==3)		//接收到换行符 \r \n
	{
		rx_cnt = 0;
		
		if((rx_buf[0]=='R')&&(rx_buf[1]=='3')&&((rx_buf[2]=='7')||rx_buf[2]=='8'))
		{
			uart_Flag=1;
		}
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
