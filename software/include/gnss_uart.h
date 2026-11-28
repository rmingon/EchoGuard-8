#pragma once

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_MODULE_COUNT 8u

/* Largest single write to a software-UART module (one CASIC command fits easily). */
#ifndef GNSS_UART_TX_SIZE
#define GNSS_UART_TX_SIZE 96u
#endif

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

/* Software UART for modules #4..#8: RX and TX sampled/driven from the TIM3 update tick. */
void GnssUart_SoftUartInit(uint32_t baudrate);
void GnssUart_TimIrqHandler(void);

/*
 * Send bytes to a module (1..8). Hardware USARTs send blocking; software channels are clocked
 * out by the TIM3 tick while the caller waits (sleeping if the scheduler runs). Returns false on
 * timeout or if len exceeds GNSS_UART_TX_SIZE on a software channel. One writer per module.
 */
bool GnssUart_Write(uint8_t module_index, const uint8_t *data, size_t len, uint32_t timeout_ms);
bool GnssUart_TxBusy(uint8_t module_index);

#ifdef __cplusplus
}
#endif
