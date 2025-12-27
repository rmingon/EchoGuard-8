#pragma once

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_MODULE_COUNT 8u

typedef struct {
	uint8_t module_index; /* 1..8 */

	GPIO_TypeDef *tx_port;
	uint16_t tx_pin;

	GPIO_TypeDef *rx_port;
	uint16_t rx_pin;

	USART_TypeDef *uart_instance; /* NULL when not on a HW USART pin pair. */
} GnssUartPins;

extern const GnssUartPins kGnssUartPins[GNSS_MODULE_COUNT];

void GnssUart_GpioInit(void);
void GnssUart_HardwareUartsInit(uint32_t baudrate);
UART_HandleTypeDef *GnssUart_GetHardwareHandle(uint8_t module_index);

#ifdef __cplusplus
}
#endif

