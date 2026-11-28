#include "spi_fusion.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "gnss.h"
#include "gnss_fusion.h"
#include "stm32f1xx_hal.h"

#define SPI_FUSION_MAGIC 0x31464745u /* 'EGF1' little-endian */

static volatile uint8_t tx_buf[SPI_FUSION_PACKET_SIZE];
static volatile uint8_t tx_index;
static volatile uint8_t selected;

static volatile uint8_t rx_buf[SPI_FUSION_PACKET_SIZE];
static volatile uint8_t rx_index;
static volatile uint8_t commands_accepted;

static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFFu;
	for (size_t i = 0; i < len; i++)
	{
		crc ^= (uint16_t)data[i] << 8;
		for (uint8_t b = 0; b < 8; b++)
		{
			if ((crc & 0x8000u) != 0)
			{
				crc = (uint16_t)((crc << 1) ^ 0x1021u);
			}
			else
			{
				crc <<= 1;
			}
		}
	}
	return crc;
}

static void put_u16_le(uint8_t *dst, uint16_t v)
{
	dst[0] = (uint8_t)(v & 0xFFu);
	dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_u32_le(uint8_t *dst, uint32_t v)
{
	dst[0] = (uint8_t)(v & 0xFFu);
	dst[1] = (uint8_t)((v >> 8) & 0xFFu);
	dst[2] = (uint8_t)((v >> 16) & 0xFFu);
	dst[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void put_i32_le(uint8_t *dst, int32_t v)
{
	put_u32_le(dst, (uint32_t)v);
}

static void build_packet_from_isr(uint8_t out[SPI_FUSION_PACKET_SIZE])
{
	GnssFusionResult r = {0};
	(void)GnssFusion_GetResultFromISR(&r);

	memset(out, 0, SPI_FUSION_PACKET_SIZE);
	put_u32_le(&out[0], SPI_FUSION_MAGIC);
	put_u32_le(&out[4], r.last_update_tick);
	put_i32_le(&out[8], r.lat_e7);
	put_i32_le(&out[12], r.lon_e7);
	put_i32_le(&out[16], r.alt_cm);
	put_u16_le(&out[20], r.avg_hdop_centi);
	put_u16_le(&out[22], r.max_residual_cm);
	out[24] = (uint8_t)r.status;
	out[25] = r.used_modules;
	out[26] = r.rejected_modules;
	out[27] = r.has_fix ? 1u : 0u;
	out[28] = commands_accepted;

	uint16_t crc = crc16_ccitt(out, SPI_FUSION_PACKET_SIZE - 2u);
	put_u16_le(&out[SPI_FUSION_PACKET_SIZE - 2u], crc);
}

static uint16_t get_u16_le(const uint8_t *src)
{
	return (uint16_t)(src[0] | ((uint16_t)src[1] << 8));
}

static uint32_t get_u32_le(const uint8_t *src)
{
	return (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) |
	       ((uint32_t)src[3] << 24);
}

static void store_rx_byte(uint8_t byte)
{
	if (rx_index < SPI_FUSION_PACKET_SIZE)
	{
		rx_buf[rx_index++] = byte;
	}
}

/* Called from the SS rising edge: run the command received during the transaction, if any. */
static void handle_command_from_isr(void)
{
	if (rx_index < SPI_COMMAND_SIZE)
	{
		return;
	}
	uint8_t frame[SPI_COMMAND_SIZE];
	for (size_t i = 0; i < SPI_COMMAND_SIZE; i++)
	{
		frame[i] = rx_buf[i];
	}
	if (get_u32_le(&frame[0]) != SPI_COMMAND_MAGIC ||
	    crc16_ccitt(frame, SPI_COMMAND_SIZE - 2u) != get_u16_le(&frame[SPI_COMMAND_SIZE - 2u]))
	{
		return;
	}

	switch (frame[4])
	{
	case SPI_COMMAND_RESET_MODULES:
		Gnss_RequestReset();
		break;
	default:
		return; /* Unknown command: not acknowledged. */
	}
	commands_accepted++;
}

void SpiFusion_Init(void)
{
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();

	/* PB3/PB4 are JTAG pins; free them while keeping SWD. */
	__HAL_AFIO_REMAP_SWJ_NOJTAG();
	__HAL_AFIO_REMAP_SPI1_ENABLE();

	/* J11 pinout:
	 * 1=SS(PB1), 2=MOSI(PB5), 3=MISO(PB4), 4=SCK(PB3), 5=GND, 6=+3.3V
	 */
	GPIO_InitTypeDef gpio = {0};

	gpio.Pin = GPIO_PIN_3 | GPIO_PIN_5;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &gpio);

	gpio.Pin = GPIO_PIN_4;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio);

	gpio.Pin = GPIO_PIN_1;
	gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
	gpio.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &gpio);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_SetPriority(SPI1_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(SPI1_IRQn);

	/* Configure SPI1 slave, 8-bit, Mode 0, software NSS forced selected (SSI=0). */
	SPI1->CR1 = 0;
	SPI1->CR2 = 0;
	SPI1->CR1 |= SPI_CR1_SSM; /* NSS managed in software. */
	SPI1->CR1 |= SPI_CR1_SSI; /* internal NSS high => not selected */
	SPI1->CR1 &= ~SPI_CR1_CPOL;
	SPI1->CR1 &= ~SPI_CR1_CPHA;
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
	SPI1->CR1 &= ~SPI_CR1_MSTR;
	SPI1->CR1 &= ~SPI_CR1_DFF;

	/* Enable interrupts only while SS is asserted to avoid continuous TXE interrupts. */
	SPI1->CR2 |= SPI_CR2_ERRIE;
	SPI1->CR1 |= SPI_CR1_SPE;

	uint8_t packet[SPI_FUSION_PACKET_SIZE];
	build_packet_from_isr(packet);
	memcpy((void *)tx_buf, packet, sizeof(packet));
	tx_index = 0;
	selected = 0;
}

void SpiFusion_SpiIrqHandler(void)
{
	uint16_t sr = SPI1->SR;

	if ((sr & SPI_SR_RXNE) != 0)
	{
		uint8_t byte = (uint8_t)SPI1->DR;
		if (selected != 0)
		{
			store_rx_byte(byte);
		}
	}

	if ((sr & SPI_SR_TXE) != 0)
	{
		uint8_t byte = 0x00;
		if (selected != 0 && tx_index < SPI_FUSION_PACKET_SIZE)
		{
			byte = tx_buf[tx_index++];
		}
		SPI1->DR = byte;
	}

	if ((sr & SPI_SR_OVR) != 0)
	{
		(void)SPI1->DR;
		(void)SPI1->SR;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
	if (gpio_pin != GPIO_PIN_1)
	{
		return;
	}

	GPIO_PinState ss = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	if (ss == GPIO_PIN_RESET)
	{
		selected = 1;

		uint8_t packet[SPI_FUSION_PACKET_SIZE];
		build_packet_from_isr(packet);

		SPI1->CR2 &= ~(SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
		(void)SPI1->DR;
		(void)SPI1->SR;
		SPI1->CR1 &= ~SPI_CR1_SSI; /* internal NSS low => selected */
		tx_index = 0;
		rx_index = 0;
		memcpy((void *)tx_buf, packet, sizeof(packet));
		if ((SPI1->SR & SPI_SR_TXE) != 0)
		{
			SPI1->DR = tx_buf[tx_index++];
		}
		SPI1->CR2 |= (SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
	}
	else
	{
		SPI1->CR2 &= ~(SPI_CR2_RXNEIE | SPI_CR2_TXEIE);
		/* EXTI1 outranks SPI1 at equal priority: the last byte may still be waiting. */
		if (selected != 0 && (SPI1->SR & SPI_SR_RXNE) != 0)
		{
			store_rx_byte((uint8_t)SPI1->DR);
		}
		if (selected != 0)
		{
			handle_command_from_isr();
		}
		selected = 0;
		tx_index = 0;
		SPI1->CR1 |= SPI_CR1_SSI; /* internal NSS high => not selected */
		(void)SPI1->DR;
		(void)SPI1->SR;
	}
}
