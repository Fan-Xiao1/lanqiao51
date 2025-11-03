/*
 * 简介：LED相关程序
 * Copyright (c) 电子设计工坊 dianshe.taobao.com
 * All rights reserved
 */
#include "led.h"

void LED_Control(u8 led_ctrl)
{
	//先熄灭所有LED灯
	HAL_GPIO_WritePin(GPIOC,0xff00,GPIO_PIN_SET);		//让PC8~PC15输出高电平，熄灭LED
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);	//打开锁存器
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);	//关闭锁存器
	
	//根据led_ctrl来点亮对应的LED
	HAL_GPIO_WritePin(GPIOC,led_ctrl<<8,GPIO_PIN_RESET);//根据led_ctrl输出低电平，点亮LED
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);	//打开锁存器
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);	//关闭锁存器
}
