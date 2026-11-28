#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_FUSION_PACKET_SIZE 32u

/*
 * Host -> board command, sent on MOSI in the first bytes of a normal packet read:
 *   u32 magic 'EGC1', u8 command, u8 argument, u16 crc16_ccitt over the first 6 bytes.
 * A host that clocks out zeros (no command) is ignored. The command runs once SS rises, and
 * byte 28 of the following packets (commands accepted, mod 256) acknowledges it.
 */
#define SPI_COMMAND_MAGIC 0x31434745u /* 'EGC1' little-endian */
#define SPI_COMMAND_SIZE 8u
#define SPI_COMMAND_RESET_MODULES 0x01u /* Pulse the shared nRESET of the 8 GNSS modules. */

void SpiFusion_Init(void);
void SpiFusion_SpiIrqHandler(void);

#ifdef __cplusplus
}
#endif

