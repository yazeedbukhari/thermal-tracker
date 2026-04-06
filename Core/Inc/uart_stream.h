/* UART telemetry and compact frame streaming helpers. */
#ifndef INC_UART_STREAM_H_
#define INC_UART_STREAM_H_

#include "amg8833.h"
#include "thermal.h"
#include <stdint.h>

#define UPSCALE_W        32U
#define UPSCALE_H        32U
#define Q_TEMP_MIN_C     18.0f
#define Q_TEMP_MAX_C     35.0f
#define STREAM_OBJ_COUNT THERMAL_MAX_OBJECTS

void uart_send(const char *msg);
void uart_send_frame_csv(const float frame[AMG8833_PIXEL_COUNT]);
void uart_send_meta_csv(const ThermalDetection *det, float servo_angle_deg);
void uart_send_um64_packet(uint16_t seq, const float *up, const ThermalObjectsResult *objs,
                           uint8_t selected_idx, int16_t servo_deg_x10);

#endif /* INC_UART_STREAM_H_ */



