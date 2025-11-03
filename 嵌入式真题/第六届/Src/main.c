
#include "main.h"
#include "gpio.h"
#include "bsp_rcc\bsp_rcc.h"
#include "KEY_LED\key_led.h"
#include "LCD\lcd.h"
#include "UART\bsp_usart.h"
#include "I2C\i2c.h"
#include "ADC\bsp_adc.h"
#include "RTC\bsp_rtc.h"
__IO uint32_t   LED_Tick=0;
  unsigned char LED_Con=0xff;
 __IO uint32_t  KEY_Tick=0;
 unsigned char  uart_buf[2];
 unsigned char  RX_buf[10];
unsigned char   RX_Cont=0;
 __IO uint32_t  uart_Tick=0;
 u8 startup_times=0;
uint16_t  ADC_Val1=0;
uint16_t  ADC_Val2=0;
float volt_r37,volt_r38;
RTC_TimeTypeDef rtc_time;
RTC_DateTypeDef rtc_date;

u8 LCD_Page=0;
u8 k_dat=0;
u8 LED_Bit=0;
u8 Key_Bit=0;
 
 void LED_Process(void);
 void KEY_Process(void);
 void LCD_Process(void);
 void ADC_Process(void);
 void RTC_Process(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

  /* Configure the system clock */
 
  /* Initialize all configured peripherals */
    MX_GPIO_Init();
	KEY_LED_Init();
	UART1_Init();
	I2CInit();
	ADC1_Init();
	ADC2_Init();
	RTC_Init();
	
	startup_times = EEPROM_Read(0x70);
	EEPROM_Write(0x70,++startup_times);
	
//	HAL_UART_Transmit(&huart1,(unsigned char *)"hello word\r\n"  ,sizeof("hello word\r\n")-1,50);
	printf("hello\r\n");
	printf("data:%d\r\n",3);
	
	HAL_UART_Receive_IT(&huart1,uart_buf,1);
//	
//	LED_control(0xff);
	
	LCD_Init();
	LCD_Clear(Blue);
	LCD_SetBackColor(Blue);
	LCD_SetTextColor(White);
	LCD_Process();
	
	
	
  
  while (1)
  {
     LED_Process();
     KEY_Process();
//	  LED_control(0x00);
	  LCD_Process();
	  ADC_Process();
	 RTC_Process();
  }
 
}

void LED_Process(void)
  {
	if(uwTick-LED_Tick<500)return;
	  LED_Tick=uwTick;
//	  LED_control(LED_Con);
//	  LED_Con=~LED_Con;
  }
  
  void KEY_Process(void)
  {
	if(uwTick-KEY_Tick<10)return;
	  KEY_Tick=uwTick;
	  key_Read();
	  if(Trg & 0x01)	//B1
	  {
		  
		  
		
		
	}
	if(Trg & 0x02)
	{
		if(LCD_Page==0)
		{
			LCD_Page=1;
		}
		
		
	}
	
	  
  }
  void ADC_Process(void)
  {
	HAL_ADC_Start(&hadc1);
	ADC_Val1=HAL_ADC_GetValue(&hadc1);
	  volt_r38 = ADC_Val1/4096.0f*3.3f;
	  HAL_ADC_Start(&hadc2);
	ADC_Val2=HAL_ADC_GetValue(&hadc2);
	  volt_r37 = ADC_Val2/4096.0f*3.3f;
  };
  
  void LCD_Process(void)
{
	u8 display_buf[20];
	if(LCD_Page==0)
	{
	sprintf((char*)display_buf,"V1:%5.2f",volt_r37);
	LCD_DisplayStringLine(Line2,display_buf);
	
//	sprintf((char*)display_buf,"MCP:%5.2f",volt_mcp);
//	LCD_DisplayStringLine(Line2,display_buf);
	
	
	LCD_DisplayStringLine(Line3,"LED:ON");
	if(LED_Bit==1)
	{
		
	LCD_DisplayStringLine(Line4,"LED:ON");
	}
	else if(LED_Bit==0)
	{
		sprintf((char*)display_buf,"LED:ON",0);
	LCD_DisplayStringLine(Line4,display_buf);
	}
	
	sprintf((char*)display_buf,"T: %02d-%02d-%02d",rtc_time.Hours,rtc_time.Minutes,rtc_time.Seconds);
	LCD_DisplayStringLine(Line5,display_buf);
    }
	else if(LCD_Page==1)
	{
	sprintf((char*)display_buf,"      Setting",0);
	LCD_DisplayStringLine(Line2,display_buf);
		
	sprintf((char*)display_buf,"    %02d-%02d-%02d",rtc_time.Hours,rtc_time.Minutes,rtc_time.Seconds);
	LCD_DisplayStringLine(Line3,display_buf);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
//	uart_Tick=uwTick;
//	 RX_buf[RX_Cont++]=uart_buf[0];
////	if(RX_Cont == 3)
////	{
////		RX_Cont=0;
////	}
//	if(uart_buf[0] == '\n')
//	{
//		RX_Cont=0;
//	}
	
	 LED_control(uart_buf[0]);
	HAL_UART_Receive_IT(&huart1,uart_buf,1);
	
}
void RTC_Process(void)
{
	HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);
};
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


