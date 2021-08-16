/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 *   Tan Wei Qiang, Benjamin A0180305R
 *   Teo You Qun A0183309A
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "temp.h"
#include "light.h"
#include "led7seg.h"
#include "uart2.h"
#include <stdio.h>
#include <math.h>

//System States
volatile uint8_t monitor = 0; //Caretaker = 0, Monitor = 1
volatile uint8_t DOUBLECLICK = 0;
volatile uint32_t FirstClick = 0;
volatile uint32_t SecondClick = 0;
volatile uint8_t BLINK_BLUE = 0;
volatile uint8_t BLINK_RED = 0;
volatile uint8_t LIGHT_TRIGGER = 0;
volatile uint16_t NAME_counter = 0;
volatile uint8_t MOVEMENT = 0;

//UART Variables
static char* msg = NULL;

//Joystick Variables
volatile uint8_t temp_threshold = 45;
volatile uint8_t temp_counter = 4;
volatile uint32_t led_temp = 0x0000;
volatile uint8_t light_threshold = 50;
volatile uint8_t light_counter = 0;
volatile uint32_t led_light = 0x0000;

//SysTick Handler
volatile uint32_t msTicks;

void SysTick_Handler(void){
	msTicks++;
}
uint32_t getTicks(void){
	return msTicks;
}

//RGB LED without Green
#define RGB_RED   0x01
#define RGB_BLUE  0x02

void rgb_init_self (void)
{
    GPIO_SetDir( 2, (1<<0), 1 );
    GPIO_SetDir( 0, (1<<26), 1 );
}

void rgb_setLeds_self (uint8_t ledMask)
{
    if ((ledMask & RGB_RED) != 0) {
        GPIO_SetValue( 2, 1);
    } else {
        GPIO_ClearValue( 2, 1 );
    }

    if ((ledMask & RGB_BLUE) != 0) {
        GPIO_SetValue( 0, (1<<26) );
    } else {
        GPIO_ClearValue( 0, (1<<26) );
    }
}

void rgb_blue_invert (void)
{
	int rgb_blue_state;
	rgb_blue_state = GPIO_ReadValue(0);
	GPIO_ClearValue(0, (rgb_blue_state & (1<<26)));
	GPIO_SetValue(0, ((~rgb_blue_state) & (1<<26)));
}

void rgb_red_invert (void)
{
	int rgb_red_state;
	rgb_red_state = GPIO_ReadValue(2);
	GPIO_ClearValue(2, (rgb_red_state & (1<<0)));
	GPIO_SetValue(2, ((~rgb_red_state) & (1<<0)));
}

void joystick_options (uint8_t joystick_stat)
{
	if ((joystick_stat == JOYSTICK_UP) && (temp_counter <8))
	{
		temp_counter++;
	}
	if ((joystick_stat == JOYSTICK_DOWN) && (temp_counter >0))
	{
		temp_counter--;
	}
	if ((joystick_stat == JOYSTICK_RIGHT) && (light_counter <8))
	{
		light_counter++;
	}
	if ((joystick_stat == JOYSTICK_LEFT) && (light_counter >0))
	{
		light_counter--;
	}
}

void temp_threshold_var (uint8_t counter)
{
	switch(counter)
	{
	case 0:
		temp_threshold = 25;
		led_temp = 0x0000;
		break;
	case 1:
		temp_threshold = 30;
		led_temp = 0x0001;
		break;
	case 2:
		temp_threshold = 35;
		led_temp = 0x0003;
		break;
	case 3:
		temp_threshold = 40;
		led_temp = 0x0007;
		break;
	case 4:
		temp_threshold = 45;
		led_temp = 0x000F;
		break;
	case 5:
		temp_threshold = 50;
		led_temp = 0x001F;
		break;
	case 6:
		temp_threshold = 55;
		led_temp = 0x003F;
		break;
	case 7:
		temp_threshold = 60;
		led_temp = 0x007F;
		break;
	case 8:
		temp_threshold = 65;
		led_temp = 0x00FF;
		break;
	default:
		temp_threshold = 45;
		led_temp = 0x000F;
	}
}

void light_threshold_var (uint8_t counter)
{
	switch(counter)
	{
	case 0:
		light_threshold = 50;
		led_light = 0x0000;
		break;
	case 1:
		light_threshold = 100;
		led_light = 0x8000;
		break;
	case 2:
		light_threshold = 150;
		led_light = 0xC000;
		break;
	case 3:
		light_threshold = 200;
		led_light = 0xE000;
		break;
	case 4:
		light_threshold = 250;
		led_light = 0xF000;
		break;
	case 5:
		light_threshold = 300;
		led_light = 0xF800;
		break;
	case 6:
		light_threshold = 350;
		led_light = 0xFC00;
		break;
	case 7:
		light_threshold = 400;
		led_light = 0xFE00;
		break;
	case 8:
		light_threshold = 450;
		led_light = 0xFF00;
		break;
	default:
		light_threshold = 50;
		led_light = 0x0000;
	}
}

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIOSW3(void)
{
	//Initialize button SW3 as EINT0
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);
}

static void init_GPIOSW4(void)
{
	// Initialize button SW4
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);
}

void pinsel_uart3(void){

	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 2;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	PinCfg.Pinnum = 1;
	PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){

	UART_CFG_Type uartCfg;
	uartCfg.Baud_rate = 115200;
	uartCfg.Databits = UART_DATABIT_8;
	uartCfg.Parity = UART_PARITY_NONE;
	uartCfg.Stopbits = UART_STOPBIT_1;

	//Pin select for UART3
	pinsel_uart3();

	//Supply power & setup working parameters for UART3
	UART_Init(LPC_UART3, &uartCfg);

	//Enable transit for UART3
	UART_TxCmd(LPC_UART3, ENABLE);
}

void EINT0_IRQHandler(void)
{
	if ((LPC_GPIOINT -> IO2IntStatF >> 10) & 0x1) //SW3 as EINT0 Interrupt
	{
		if (DOUBLECLICK == 0)
		{
			FirstClick = getTicks();
			DOUBLECLICK++;
		}
		else if (DOUBLECLICK == 1)
		{
			SecondClick = getTicks();
			if (SecondClick - FirstClick <= 1000)
			{
				monitor = 0;
				msg = "Leaving MONITOR mode\r\nEntering CARETAKER mode\r\n";
				UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING);
			}
			else
			{
				DOUBLECLICK = 0;
			}
		}
	}
	LPC_GPIOINT -> IO2IntClr = 1<<10;
	LPC_SC->EXTINT = (1<<0);
}

void EINT3_IRQHandler(void)
{
	if ((LPC_GPIOINT -> IO2IntStatF >> 5) & 0x1)
	{
		LIGHT_TRIGGER = 1;
		light_clearIrqStatus();
		LPC_GPIOINT -> IO2IntClr |= (1 << 5);
	}
}

int main (void) {
    int32_t xoff = 0;
    int32_t yoff = 0;
    int32_t zoff = 0;

    int8_t x = 0;
    int8_t y = 0;
    int8_t z = 0;

    int8_t x_bef = 0;
    int8_t y_bef = 0;
    int8_t z_bef = 0;
    float accel = 0;

    //Switch SW4
    uint8_t SW_MONITOR = 1;

    //Temperature
    SysTick_Config(SystemCoreClock/1000);
    temp_init(getTicks);
    uint32_t temp_value;

    //LED7Seg and Timer
    uint32_t time1 = 0;
    uint32_t time2 = 0;
    uint32_t time3 = 0;
    uint8_t timer = 0;
    uint8_t updater = 0;
    uint32_t seg7time_inverted[16] = {0x24, 0x7D, 0xE0, 0x70, 0x39, 0x32, 0x22, 0x7C, 0x20, 0x30, 0x28, 0x23, 0xA6, 0x61, 0xA2, 0xAA};

    //OLED
    char display_acc[40] = {};

    //UART
    char tera_display[40] = {};

    init_i2c();
    init_ssp();
    init_GPIOSW3();
    init_GPIOSW4();

    pca9532_init();
    acc_init();
    oled_init();
    led7seg_init();
    light_init();
    rgb_init_self();
    init_uart();
    joystick_init();

    //Joystick
    uint8_t joystick_value;

    //Light Sensor
    light_enable();
    uint32_t light_value;

    //SysTick Priority
    NVIC_SetPriority(SysTick_IRQn, 1); //Timer Priority 1

    //Enable EINT3 Interrupt for Light Sensor
    LPC_GPIOINT -> IO2IntEnF |= (1<<5); //Light Sensor
    light_setHiThreshold(900);
    light_setLoThreshold(50);
    light_clearIrqStatus();
    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_SetPriority(EINT3_IRQn, 3); //Light Sensor Priority 3
	NVIC_EnableIRQ(EINT3_IRQn);

    //Enable EINT0 Interrupt for SW3
    LPC_GPIOINT -> IO2IntEnF |= 1<<10;
    NVIC_ClearPendingIRQ(EINT0_IRQn);
    NVIC_SetPriority(EINT0_IRQn, 2); //SW3 Priority 2
	NVIC_EnableIRQ(EINT0_IRQn);

    //OLED Display Values
    uint32_t light_value_disp;
    uint32_t temp_value_disp;
    int8_t x_disp = 0;
    int8_t y_disp = 0;
    int8_t z_disp = 0;

    /*
     * Assume base board in zero-g position when reading first value.
     */
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 64-z;
    x_disp = x+xoff;
    y_disp = y+yoff;
	z_disp = z+zoff;
	temp_value_disp = temp_read();
	light_value_disp = light_read();

	msg = "Entering CARETAKER mode\r\n";
	UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING);

    while (1)
    {
    	SW_MONITOR = (GPIO_ReadValue(1) >> 31) & 0x1;
    	if (monitor == 0)
    	{
    		DOUBLECLICK = 0;
    		led7seg_setChar(0xFF, 1);
    		rgb_setLeds_self(0);
    		BLINK_BLUE = 0;
    		BLINK_RED = 0;
    		MOVEMENT = 0;
    		pca9532_setLeds(0x0000, 0xFFFF);
    		oled_clearScreen(OLED_COLOR_BLACK);
    		if (SW_MONITOR == 0)
    		{
    			monitor = 1;
    			time2 = getTicks();
    			time3 = getTicks();
    			timer = 0;
    			msg = "Leaving CARETAKER mode\r\nEntering MONITOR mode\r\n";
    			UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING);
    		}
    	}
    	else if (monitor == 1)
        {
    		/*LED7Seg Display and Timer*/
    		time1 = getTicks();
			if (time1 - time3 >= 500)
			{
				if ((LIGHT_TRIGGER == 1)&&(MOVEMENT == 1))
				{
					BLINK_BLUE = 1;
				}
				if (BLINK_BLUE == 1)
				{
					rgb_blue_invert();
				}
				if (BLINK_RED == 1)
				{
					rgb_red_invert();
				}
				time3 = time1;
			}
			if (time1 - time2 >= 1000)
			{
				time2 = time1;
				timer++;
				if (timer == 16)
				{
					timer = 0;
				}
			}
			led7seg_setChar(seg7time_inverted[timer], 1);

			/* ####### Accelerometer and LEDs  ###### */
			/* # */

			temp_value = temp_read();
			light_value = light_read();
			x_bef = x;
			y_bef = y;
			z_bef = z;
			acc_read(&x,&y,&z);
			x = x+xoff;
			y = y+yoff;
			z = z+zoff;

			sprintf(display_acc, "MONITOR\n", 1);
			oled_putString(20,0, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			if ((timer == 5 || timer == 10 || timer == 15) && updater == 0)
			{
				temp_value_disp = temp_value;
				light_value_disp = light_value;
				x_disp = x;
				y_disp = y;
				z_disp = z;
				updater = 1;
				if (timer == 15)
				{
					if (BLINK_RED == 1)
					{
						msg = "Fire detected\r\n";
						UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING);
					}
					if (BLINK_BLUE == 1)
					{
						msg = "Movement in darkness detected\r\n";
						UART_Send(LPC_UART3, (uint8_t *)msg, strlen(msg), BLOCKING);
					}
					sprintf(tera_display, "%03d - T: %3.1f, L: %3d, AX: %2d AY: %2d AZ: %2d\n\r", NAME_counter, temp_value_disp/10.0, light_value_disp, x_disp, y_disp, z_disp, 1);
					UART_Send(LPC_UART3, tera_display , strlen(tera_display), BLOCKING);
					NAME_counter++;
				}
			}
			else if (timer == 6 || timer == 11 || timer == 0)
			{
				updater = 0;
			}
			sprintf(display_acc, "Accelerometer:\r\n", 1);
			oled_putString(0, 10, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			sprintf(display_acc, "x = %d; y = %d\r\n",x_disp, y_disp, 1);
			oled_putString(0, 20, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			sprintf(display_acc, "z = %d\r\n",z_disp, 1);
			oled_putString(0, 30, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			sprintf(display_acc, "Temp: %2.1f\r\n", temp_value_disp/10.0);
			oled_putString(0, 40, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			sprintf(display_acc, "Light: %d\r\n", light_value_disp);
			oled_putString(0, 50, display_acc, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			if (temp_value/10.0 > temp_threshold)
			{
				BLINK_RED = 1;
			}

			accel = sqrt((pow((x - x_bef),2)+pow((y - y_bef),2)+pow((z - z_bef),2))/3);
			if (accel >= 16)
			{
				MOVEMENT = 1;
			}
			if (light_value > light_threshold)
			{
				LIGHT_TRIGGER = 0;
			}

			joystick_value = joystick_read();
			joystick_options(joystick_value);
			light_threshold_var(light_counter);
			light_setLoThreshold(light_threshold);
			temp_threshold_var(temp_counter);

			pca9532_setLeds((led_temp | led_light), 0xFFFF);
        }
    }

}

void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}

