/*
 * uart_stream.h — USART3 binary frame protocol for PC visualization
 *
 * Packs the 64x64 upscaled thermal frame, FSM state, servo angles, and
 * centroid data into a compact binary packet and transmits it over USART3
 * (PD8/PD9, routed through ST-LINK VCP). The PC visualization tool (Ali's
 * domain) parses these packets for real-time display.
 *
 * Public API:  UART_StreamInit, UART_StreamFrame
 *
 * Owner:
 */

#ifndef INC_UART_STREAM_H_
#define INC_UART_STREAM_H_



#endif /* INC_UART_STREAM_H_ */
