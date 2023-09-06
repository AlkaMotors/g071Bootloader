/*
 * WS2812.c
 *
 *  Created on: Sep 6, 2023
 *      Author: frank26080115
 */

#include "main.h"
#ifdef USE_LED_STRIP
#include "WS2812.h"

uint8_t led_buffer_h[24];

void send_LED_RGB(uint8_t red, uint8_t green, uint8_t blue)
{
	// arrange the bit order
	uint32_t twenty_four_bit_color_number = green << 16 | red << 8 | blue ;

	// pre-cache the TxH time for every bit
	for (int i = 0; i < 24 ; i ++) {
		led_buffer_h[i] = (((twenty_four_bit_color_number >> (23 - i)) & 1) * 13) + 8;
	}

	// bit-bang out all the bits
	for (int i = 0; i < 24 ; i ++) {
		TIM2->CNT = 0;
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8);
		uint8_t th = led_buffer_h[i];
		while (TIM2->CNT < th) {
		}
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
		while (TIM2->CNT < 33) {
		}
	}
}

void WS2812_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

#endif
