#include "gnss_uart.h"

#include <stddef.h>

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart3;

const GnssUartPins kGnssUartPins[GNSS_MODULE_COUNT] = {
	{.module_index = 1,
	 .tx_port = GPIOB,
	 .tx_pin = GPIO_PIN_10,
	 .rx_port = GPIOB,
	 .rx_pin = GPIO_PIN_11,
	 .uart_instance = USART3},
	{.module_index = 2,
	 .tx_port = GPIOA,
	 .tx_pin = GPIO_PIN_2,
	 .rx_port = GPIOA,
	 .rx_pin = GPIO_PIN_3,
	 .uart_instance = USART2},
	{.module_index = 3,
	 .tx_port = GPIOB,
	 .tx_pin = GPIO_PIN_6,
	 .rx_port = GPIOB,
	 .rx_pin = GPIO_PIN_7,
	 .uart_instance = USART1},
	{.module_index = 4,
	 .tx_port = GPIOB,
	 .tx_pin = GPIO_PIN_8,
	 .rx_port = GPIOB,
	 .rx_pin = GPIO_PIN_9,
	 .uart_instance = NULL},
	{.module_index = 5,
	 .tx_port = GPIOB,
	 .tx_pin = GPIO_PIN_12,
	 .rx_port = GPIOB,
	 .rx_pin = GPIO_PIN_13,
	 .uart_instance = NULL},
	{.module_index = 6,
	 .tx_port = GPIOB,
	 .tx_pin = GPIO_PIN_14,
	 .rx_port = GPIOB,
	 .rx_pin = GPIO_PIN_15,
	 .uart_instance = NULL},
	{.module_index = 7,
	 .tx_port = GPIOA,
	 .tx_pin = GPIO_PIN_4,
	 .rx_port = GPIOA,
	 .rx_pin = GPIO_PIN_5,
	 .uart_instance = NULL},
	{.module_index = 8,
	 .tx_port = GPIOA,
	 .tx_pin = GPIO_PIN_6,
	 .rx_port = GPIOA,
	 .rx_pin = GPIO_PIN_7,
	 .uart_instance = NULL},
};

static UART_HandleTypeDef *hardware_handle_for_instance(USART_TypeDef *instance) {
	if (instance == USART1) {
		return &huart1;
	}
	if (instance == USART2) {
		return &huart2;
	}
	if (instance == USART3) {
		return &huart3;
	}
	return NULL;
}

UART_HandleTypeDef *GnssUart_GetHardwareHandle(uint8_t module_index) {
	for (size_t i = 0; i < GNSS_MODULE_COUNT; i++) {
		if (kGnssUartPins[i].module_index == module_index) {
			return hardware_handle_for_instance(kGnssUartPins[i].uart_instance);
		}
	}
	return NULL;
}

void GnssUart_GpioInit(void) {
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* GNSS #3 uses USART1 remapped to PB6/PB7. */
	__HAL_AFIO_REMAP_USART1_ENABLE();

	for (size_t i = 0; i < GNSS_MODULE_COUNT; i++) {
		const GnssUartPins *pins = &kGnssUartPins[i];

		GPIO_InitTypeDef gpio = {0};

		/* RX: input (works for both HW UART and future SW UART). */
		gpio.Pin = pins->rx_pin;
		gpio.Mode = GPIO_MODE_INPUT;
		gpio.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(pins->rx_port, &gpio);

		/* TX: HW UART uses alternate function; otherwise default to GPIO output. */
		gpio.Pin = pins->tx_pin;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_HIGH;
		gpio.Mode = (pins->uart_instance != NULL) ? GPIO_MODE_AF_PP : GPIO_MODE_OUTPUT_PP;
		HAL_GPIO_Init(pins->tx_port, &gpio);
	}
}

static void uart_init(UART_HandleTypeDef *huart, USART_TypeDef *instance, uint32_t baudrate) {
	huart->Instance = instance;
	huart->Init.BaudRate = baudrate;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.Parity = UART_PARITY_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart->Init.OverSampling = UART_OVERSAMPLING_16;
	(void)HAL_UART_Init(huart);
}

void GnssUart_HardwareUartsInit(uint32_t baudrate) {
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();

	/*
	 * Note: only GNSS modules #1..#3 are wired to valid STM32F103 hardware USART pin pairs:
	 * - #1: USART3 (PB10/PB11)
	 * - #2: USART2 (PA2/PA3)
	 * - #3: USART1 remap (PB6/PB7)
	 *
	 * Modules #4..#8 require a software UART (or other acquisition method) if you intend to
	 * receive from all 8 modules concurrently.
	 */
	uart_init(&huart1, USART1, baudrate);
	uart_init(&huart2, USART2, baudrate);
	uart_init(&huart3, USART3, baudrate);
}
