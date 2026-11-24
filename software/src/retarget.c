#include "stm32f1xx_hal.h"

#include <errno.h>
#include <string.h>
#include <sys/unistd.h>

/*
 * printf() goes to SEGGER RTT up-channel 0 (read over SWD by J-Link, no extra pin).
 * SWO/ITM can't be used on this board: PB3 (TRACESWO) is wired as SPI1 SCK.
 *
 * Minimal RTT control block, layout-compatible with SEGGER's. The J-Link host finds it by
 * scanning RAM for the "SEGGER RTT" id. Writes are non-blocking: when the host isn't reading
 * and the buffer is full, extra output is dropped.
 */

#ifndef RTT_UP_BUFFER_SIZE
#define RTT_UP_BUFFER_SIZE 1024u
#endif

typedef struct {
	const char *name;
	char *buffer;
	unsigned size;
	volatile unsigned wr_off;
	volatile unsigned rd_off;
	unsigned flags;
} RttRingBuffer;

typedef struct {
	char id[16];
	int max_up_buffers;
	int max_down_buffers;
	RttRingBuffer up[1];
	RttRingBuffer down[1];
} RttControlBlock;

static char rtt_up_buffer[RTT_UP_BUFFER_SIZE];
static char rtt_down_buffer[16];
RttControlBlock _SEGGER_RTT;

void Rtt_Init(void)
{
	RttControlBlock *cb = &_SEGGER_RTT;

	memset(cb, 0, sizeof(*cb));
	cb->max_up_buffers = 1;
	cb->max_down_buffers = 1;

	cb->up[0].name = "Terminal";
	cb->up[0].buffer = rtt_up_buffer;
	cb->up[0].size = sizeof(rtt_up_buffer);

	cb->down[0].name = "Terminal";
	cb->down[0].buffer = rtt_down_buffer;
	cb->down[0].size = sizeof(rtt_down_buffer);

	/* Publish the id last so the host never sees a half-initialised block. */
	__DMB();
	memcpy(&cb->id[7], "RTT", 4);
	__DMB();
	memcpy(&cb->id[0], "SEGGER ", 7);
	__DMB();
}

static void rtt_write(const char *data, unsigned len)
{
	RttRingBuffer *rb = &_SEGGER_RTT.up[0];
	if (rb->buffer == NULL)
	{
		return;
	}

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	unsigned wr = rb->wr_off;
	unsigned rd = rb->rd_off;
	unsigned free_bytes = (rd > wr) ? (rd - wr - 1u) : (rb->size - (wr - rd) - 1u);
	if (len > free_bytes)
	{
		len = free_bytes;
	}

	while (len-- > 0u)
	{
		rb->buffer[wr++] = *data++;
		if (wr >= rb->size)
		{
			wr = 0;
		}
	}
	__DMB();
	rb->wr_off = wr;

	__set_PRIMASK(primask);
}

int _write(int file, char *ptr, int len)
{
	(void)file;
	if (ptr == NULL || len <= 0)
	{
		errno = EINVAL;
		return -1;
	}

	rtt_write(ptr, (unsigned)len);
	return len;
}
