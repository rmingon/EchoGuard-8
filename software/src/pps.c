#include "pps.h"

#include <string.h>

#include "stm32f1xx_hal.h"

#define PPS_TIMEOUT_MS 1500u

static volatile uint32_t overflow_hi;
static volatile PpsInfo last;

void Pps_Init(void) {
	overflow_hi = 0;
	memset((void *)&last, 0, sizeof(last));

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();

	/* Pull-down: no spurious edges when module #6 is not populated. */
	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = GPIO_PIN_1;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &gpio);

	uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
	if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
		tim_clk *= 2u;
	}

	TIM2->CR1 = 0;
	TIM2->PSC = (uint16_t)((tim_clk / 1000000u) - 1u);
	TIM2->ARR = 0xFFFFu;
	/* CC2 as input on TI2, filter fCK_INT N=8 (125 ns at 64 MHz), rising edge. */
	TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_CC2S | TIM_CCMR1_IC2F | TIM_CCMR1_IC2PSC)) |
		      TIM_CCMR1_CC2S_0 | (3u << TIM_CCMR1_IC2F_Pos);
	TIM2->CCER = TIM_CCER_CC2E;
	TIM2->CNT = 0;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->SR = 0;
	TIM2->DIER = TIM_DIER_CC2IE | TIM_DIER_UIE;

	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->CR1 = TIM_CR1_CEN;
}

void Pps_TimIrqHandler(void) {
	if ((TIM2->SR & TIM_SR_CC2IF) != 0) {
		uint16_t cap = (uint16_t)TIM2->CCR2; /* Clears CC2IF. */
		/*
		 * An overflow still pending here happened before we read the capture. If the captured
		 * value is small, the edge came after that overflow, so count it now.
		 */
		uint32_t hi = overflow_hi;
		if ((TIM2->SR & TIM_SR_UIF) != 0 && cap < 0x8000u) {
			hi++;
		}
		uint32_t now = (hi << 16) | cap;

		uint32_t period = 0;
		if (last.count > 0) {
			uint32_t d = now - last.last_us;
			if (d >= PPS_PERIOD_MIN_US && d <= PPS_PERIOD_MAX_US) {
				period = d;
			}
		}
		last.period_us = period;
		last.last_us = now;
		last.last_tick = HAL_GetTick();
		last.count++;
	}

	if ((TIM2->SR & TIM_SR_UIF) != 0) {
		TIM2->SR = ~TIM_SR_UIF;
		overflow_hi++;
	}
}

uint32_t Pps_NowUs(void) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	uint32_t hi = overflow_hi;
	uint16_t cnt = (uint16_t)TIM2->CNT;
	if ((TIM2->SR & TIM_SR_UIF) != 0 && cnt < 0x8000u) {
		hi++;
	}
	__set_PRIMASK(primask);
	return (hi << 16) | cnt;
}

bool Pps_GetLast(PpsInfo *out) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	memcpy(out, (const void *)&last, sizeof(*out));
	__set_PRIMASK(primask);

	return out->count > 0 && (HAL_GetTick() - out->last_tick) <= PPS_TIMEOUT_MS;
}
