#include "gnss_uart.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart3;

#ifndef GNSS_UART_RING_SIZE
#define GNSS_UART_RING_SIZE 256u
#endif

typedef struct {
	volatile uint16_t head;
	volatile uint16_t tail;
	uint8_t buffer[GNSS_UART_RING_SIZE];
} RingBuffer;

static RingBuffer rb_usart1;
static RingBuffer rb_usart2;
static RingBuffer rb_usart3;

typedef enum {
	SOFT_RX_IDLE = 0,
	SOFT_RX_START = 1,
	SOFT_RX_DATA = 2,
	SOFT_RX_STOP = 3,
} SoftRxState;

typedef struct {
	SoftRxState state;
	uint8_t sub_tick;
	uint8_t bit_index;
	uint8_t byte;
	RingBuffer rb;
} SoftUartChannel;

static SoftUartChannel soft_channels[GNSS_MODULE_COUNT];

static uint8_t rx_byte_usart1;
static uint8_t rx_byte_usart2;
static uint8_t rx_byte_usart3;

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

static RingBuffer *ring_for_instance(USART_TypeDef *instance) {
	if (instance == USART1) {
		return &rb_usart1;
	}
	if (instance == USART2) {
		return &rb_usart2;
	}
	if (instance == USART3) {
		return &rb_usart3;
	}
	return NULL;
}

static uint8_t *rx_byte_for_instance(USART_TypeDef *instance) {
	if (instance == USART1) {
		return &rx_byte_usart1;
	}
	if (instance == USART2) {
		return &rx_byte_usart2;
	}
	if (instance == USART3) {
		return &rx_byte_usart3;
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
		gpio.Mode = (pins->uart_instance != NULL) ? GPIO_MODE_AF_PP : GPIO_MODE_INPUT;
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

	HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void GnssUart_StartHardwareRx(void) {
	(void)HAL_UART_Receive_IT(&huart1, &rx_byte_usart1, 1);
	(void)HAL_UART_Receive_IT(&huart2, &rx_byte_usart2, 1);
	(void)HAL_UART_Receive_IT(&huart3, &rx_byte_usart3, 1);
}

static void ring_push_byte(RingBuffer *rb, uint8_t byte) {
	uint16_t head = rb->head;
	uint16_t next = (uint16_t)((head + 1u) % (uint16_t)sizeof(rb->buffer));
	if (next == rb->tail) {
		return;
	}
	rb->buffer[head] = byte;
	rb->head = next;
}

static size_t ring_pop_bytes(RingBuffer *rb, uint8_t *dst, size_t max_len) {
	size_t count = 0;
	while (count < max_len) {
		uint16_t tail = rb->tail;
		if (tail == rb->head) {
			break;
		}
		dst[count++] = rb->buffer[tail];
		rb->tail = (uint16_t)((tail + 1u) % (uint16_t)sizeof(rb->buffer));
	}
	return count;
}

size_t GnssUart_ReadBytes(uint8_t module_index, uint8_t *dst, size_t max_len) {
	UART_HandleTypeDef *huart = GnssUart_GetHardwareHandle(module_index);
	if (huart != NULL && huart->Instance != NULL) {
		RingBuffer *rb = ring_for_instance(huart->Instance);
		if (rb == NULL) {
			return 0;
		}
		return ring_pop_bytes(rb, dst, max_len);
	}

	if (module_index < 1 || module_index > GNSS_MODULE_COUNT) {
		return 0;
	}
	return ring_pop_bytes(&soft_channels[module_index - 1].rb, dst, max_len);
}

void GnssUart_IrqHandler(USART_TypeDef *instance) {
	UART_HandleTypeDef *huart = hardware_handle_for_instance(instance);
	if (huart == NULL) {
		return;
	}
	HAL_UART_IRQHandler(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == NULL || huart->Instance == NULL) {
		return;
	}
	RingBuffer *rb = ring_for_instance(huart->Instance);
	uint8_t *byte = rx_byte_for_instance(huart->Instance);
	if (rb == NULL || byte == NULL) {
		return;
	}
	ring_push_byte(rb, *byte);
	(void)HAL_UART_Receive_IT(huart, byte, 1);
}

static bool read_pin(GPIO_TypeDef *port, uint16_t pin) {
	return (port->IDR & pin) != 0;
}

static void soft_uart_tick(void) {
	for (size_t i = 0; i < GNSS_MODULE_COUNT; i++) {
		const GnssUartPins *pins = &kGnssUartPins[i];
		if (pins->uart_instance != NULL) {
			continue;
		}

		SoftUartChannel *ch = &soft_channels[i];
		bool level = read_pin(pins->rx_port, pins->rx_pin);

		switch (ch->state) {
		case SOFT_RX_IDLE:
			if (!level) {
				ch->state = SOFT_RX_START;
				ch->sub_tick = 0;
			}
			break;
		case SOFT_RX_START:
			ch->sub_tick++;
			if (ch->sub_tick == 4) {
				if (!level) {
					ch->state = SOFT_RX_DATA;
					ch->sub_tick = 0;
					ch->bit_index = 0;
					ch->byte = 0;
				} else {
					ch->state = SOFT_RX_IDLE;
				}
			}
			break;
		case SOFT_RX_DATA:
			ch->sub_tick++;
			if (ch->sub_tick == 8) {
				if (level) {
					ch->byte |= (uint8_t)(1u << ch->bit_index);
				}
				ch->bit_index++;
				ch->sub_tick = 0;
				if (ch->bit_index >= 8) {
					ch->state = SOFT_RX_STOP;
				}
			}
			break;
		case SOFT_RX_STOP:
			ch->sub_tick++;
			if (ch->sub_tick == 8) {
				if (level) {
					ring_push_byte(&ch->rb, ch->byte);
				}
				ch->state = SOFT_RX_IDLE;
				ch->sub_tick = 0;
			}
			break;
		default:
			ch->state = SOFT_RX_IDLE;
			ch->sub_tick = 0;
			break;
		}
	}
}

void GnssUart_SoftUartInit(uint32_t baudrate) {
	memset(soft_channels, 0, sizeof(soft_channels));

	__HAL_RCC_TIM2_CLK_ENABLE();

	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
	uint32_t tim_clk = pclk1;
	uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1);
	if (ppre1 != RCC_CFGR_PPRE1_DIV1) {
		tim_clk = pclk1 * 2u;
	}

	uint32_t base_hz = 1000000u;
	uint32_t prescaler = (tim_clk / base_hz);
	if (prescaler == 0) {
		prescaler = 1;
	}
	prescaler -= 1u;

	uint32_t tick_hz = baudrate * 8u;
	uint32_t period = (base_hz + (tick_hz / 2u)) / tick_hz;
	if (period == 0) {
		period = 1;
	}
	period -= 1u;

	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->PSC = (uint16_t)(prescaler > 0xFFFFu ? 0xFFFFu : prescaler);
	TIM2->ARR = (uint16_t)(period > 0xFFFFu ? 0xFFFFu : period);
	TIM2->CNT = 0;
	TIM2->EGR = TIM_EGR_UG;
	TIM2->SR = 0;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CR1 |= TIM_CR1_CEN;
}

void GnssUart_TimIrqHandler(void) {
	if ((TIM2->SR & TIM_SR_UIF) != 0) {
		TIM2->SR &= ~TIM_SR_UIF;
		soft_uart_tick();
	}
}
