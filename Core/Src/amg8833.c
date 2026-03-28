/*
 * amg8833.c — AMG8833 thermal array sensor driver (I2C1, DMA + double buffering)
 *
 * Manages non-blocking DMA reads of 8x8 thermal frames from the AMG8833.
 * Two 128-byte buffers alternate: DMA fills one while the CPU processes the
 * other. The DMA-complete ISR swaps buffer pointers.
 *
 * Owner:
 */

#include "amg8833.h"v
