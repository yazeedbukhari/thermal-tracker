/*
 * uart_stream.c — USART3 binary frame protocol for PC visualization
 *
 * Packs the 64x64 upscaled thermal frame, FSM state, servo angles, and
 * centroid data into a compact binary packet and transmits it over USART3
 * (routed through ST-LINK VCP) for real-time PC display.
 *
 * Owner:
 */

#include "uart_stream.h"
