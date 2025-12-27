#pragma once

#include "stm32f1xx_hal.h"

#include <stddef.h>

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

void GnssUart_StartHardwareRx(void);
size_t GnssUart_ReadBytes(uint8_t module_index, uint8_t *dst, size_t max_len);
void GnssUart_IrqHandler(USART_TypeDef *instance);

void GnssUart_SoftUartInit(uint32_t baudrate);
void GnssUart_TimIrqHandler(void);

#ifdef __cplusplus
}
#endif
