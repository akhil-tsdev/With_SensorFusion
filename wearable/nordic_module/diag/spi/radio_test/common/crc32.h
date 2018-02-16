#include <stddef.h>
#include <stdint.h>

/// Computes 32-bit CRC of |buf|.
uint32_t crc32(uint32_t crc, const void *buf, size_t size);
