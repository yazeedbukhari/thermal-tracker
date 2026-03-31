/*
 * uart_stream.h — USART3 binary frame protocol for PC visualization
 *
 * Packs the 64x64 upscaled thermal frame, FSM state, servo angles, and
 * centroid data into a compact binary packet and transmits it over USART3
 * (PD8/PD9, routed through ST-LINK VCP). The PC visualization tool (Ali's
 * domain) parses these packets for real-time display.
 *
 * Public API:
 *   print_uart            — transmit a null-terminated string over USART3
 *   UartStream_SendFrame  — render, upscale, and stream one sensor frame
 *
 * Owner:
 */

#ifndef INC_UART_STREAM_H_
#define INC_UART_STREAM_H_

#include <stdint.h>
#include "thermal.h"
#include "object_tracker.h"
#include "main.h"

void print_uart(char *msg);

void UartStream_SendFrame(
  const float               *frame,
  float                     *upscaled,
  const ThermalObjectsResult *objs,
  const ThermalDetection    *det,
  uint32_t                   now
);

#endif /* INC_UART_STREAM_H_ */
