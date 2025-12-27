#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gnss_uart.h"

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

static void LedTask(void *argument) {
	(void)argument;
	while (1) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

void SVC_Handler(void) {
	vPortSVCHandler();
}

void PendSV_Handler(void) {
	xPortPendSVHandler();
}

void SysTick_Handler(void) {
	HAL_IncTick();
	xPortSysTickHandler();
}

int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	GnssUart_GpioInit();
	GnssUart_HardwareUartsInit(9600);

	xTaskCreate(LedTask, "led", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
	vTaskStartScheduler();

	while (1) {
	}
}

static void SystemClock_Config(void) {
	RCC_OscInitTypeDef oscillator_config = {0};
	RCC_ClkInitTypeDef clock_config = {0};

	/* Typical BluePill: 8MHz HSE -> 72MHz SYSCLK. */
	oscillator_config.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	oscillator_config.HSEState = RCC_HSE_ON;
	oscillator_config.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	oscillator_config.PLL.PLLState = RCC_PLL_ON;
	oscillator_config.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	oscillator_config.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&oscillator_config) != HAL_OK) {
		while (1) {
		}
	}

	clock_config.ClockType =
		RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clock_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clock_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clock_config.APB1CLKDivider = RCC_HCLK_DIV2;
	clock_config.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&clock_config, FLASH_LATENCY_2) != HAL_OK) {
		while (1) {
		}
	}
}

static void MX_GPIO_Init(void) {
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = GPIO_PIN_13;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &gpio);

	/* BluePill PC13 LED is typically active-low. Start off. */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void vApplicationMallocFailedHook(void) {
	taskDISABLE_INTERRUPTS();
	while (1) {
	}
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
	(void)task;
	(void)task_name;
	taskDISABLE_INTERRUPTS();
	while (1) {
	}
}
