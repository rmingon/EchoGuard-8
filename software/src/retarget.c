#include "stm32f1xx_hal.h"

#include <errno.h>
#include <sys/unistd.h>

int _write(int file, char *ptr, int len)
{
	(void)file;
	if (ptr == NULL || len <= 0)
	{
		errno = EINVAL;
		return -1;
	}

	for (int i = 0; i < len; i++)
	{
		char c = ptr[i];

		if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0)
		{
			continue;
		}
		if ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0)
		{
			continue;
		}
		if ((ITM->TER & 1u) == 0)
		{
			continue;
		}

		while (ITM->PORT[0].u32 == 0)
		{
		}
		ITM->PORT[0].u8 = (uint8_t)c;
	}

	return len;
}

