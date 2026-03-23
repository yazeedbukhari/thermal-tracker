/*
 * amg8833.h — AMG8833 thermal array sensor driver (I2C1, DMA + double buffering)
 *
 * Manages non-blocking DMA reads of 8x8 thermal frames from the AMG8833.
 * Two 128-byte buffers alternate: DMA fills one while the CPU processes the
 * other. The DMA-complete ISR swaps buffer pointers.
 *
 * Public API:  AMG8833_Init, AMG8833_StartFrame, AMG8833_FrameReady,
 *              AMG8833_SwapAndGetFrame, AMG8833_DMACompleteCallback
 *
 * Owner:
 */

#ifndef INC_AMG8833_H_
#define INC_AMG8833_H_



#endif /* INC_AMG8833_H_ */
