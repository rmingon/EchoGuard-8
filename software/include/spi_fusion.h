#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_FUSION_PACKET_SIZE 32u

void SpiFusion_Init(void);
void SpiFusion_SpiIrqHandler(void);

#ifdef __cplusplus
}
#endif

