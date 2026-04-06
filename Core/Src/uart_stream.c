/* UART message and packet formatting/transmit routines. */
#include "uart_stream.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

void print_uart(char *msg)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

void uart_send(const char *msg)
{
    return;
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
}

static int append_fixed(char *dst, size_t cap, size_t *idx, float value, int decimals)
{
    int32_t scale      = (decimals == 1) ? 10 : 100;
    int32_t scaled     = (int32_t)(value * (float)scale);
    int32_t abs_scaled = (scaled < 0) ? -scaled : scaled;
    int written;

    if (decimals == 1) {
        written = snprintf(dst + *idx, cap - *idx, "%s%ld.%01ld", (scaled < 0) ? "-" : "",
                           (long)(abs_scaled / 10), (long)(abs_scaled % 10));
    } else {
        written = snprintf(dst + *idx, cap - *idx, "%s%ld.%02ld", (scaled < 0) ? "-" : "",
                           (long)(abs_scaled / 100), (long)(abs_scaled % 100));
    }

    if ((written <= 0) || ((size_t)written >= (cap - *idx))) {
        return 0;
    }

    *idx += (size_t)written;
    return 1;
}

void uart_send_frame_csv(const float frame[AMG8833_PIXEL_COUNT])
{
    char line[900];
    size_t idx = 0;

    idx += (size_t)snprintf(line + idx, sizeof(line) - idx, "FRAME,");
    for (uint32_t i = 0; i < AMG8833_PIXEL_COUNT; i++) {
        if (!append_fixed(line, sizeof(line), &idx, frame[i], 2)) {
            uart_send("ERR_FMT\r\n");
            return;
        }

        if (i < (AMG8833_PIXEL_COUNT - 1U)) {
            int written = snprintf(line + idx, sizeof(line) - idx, ",");
            if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) {
                uart_send("ERR_FMT\r\n");
                return;
            }
            idx += (size_t)written;
        } else {
            int written = snprintf(line + idx, sizeof(line) - idx, "\r\n");
            if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) {
                uart_send("ERR_FMT\r\n");
                return;
            }
            idx += (size_t)written;
        }
    }

    HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)idx, 200);
}

void uart_send_meta_csv(const ThermalDetection *det, float servo_angle_deg)
{
    char line[240];
    size_t idx = 0;

    int written = snprintf(line + idx, sizeof(line) - idx, "META,%u,%d,%d,%d,%d,",
                           (unsigned)det->target_found, (int)det->min_x, (int)det->max_x,
                           (int)det->min_y, (int)det->max_y);
    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx))) {
        uart_send("ERR_FMT\r\n");
        return;
    }
    idx += (size_t)written;

    if (!append_fixed(line, sizeof(line), &idx, det->centroid_x, 2))
        return;
    written = snprintf(line + idx, sizeof(line) - idx, ",");
    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx)))
        return;
    idx += (size_t)written;

    if (!append_fixed(line, sizeof(line), &idx, det->centroid_y, 2))
        return;
    written = snprintf(line + idx, sizeof(line) - idx, ",");
    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx)))
        return;
    idx += (size_t)written;

    if (!append_fixed(line, sizeof(line), &idx, det->avg_temp_c, 2))
        return;
    written = snprintf(line + idx, sizeof(line) - idx, ",");
    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx)))
        return;
    idx += (size_t)written;

    if (!append_fixed(line, sizeof(line), &idx, det->threshold_c, 2))
        return;
    written = snprintf(line + idx, sizeof(line) - idx, ",");
    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx)))
        return;
    idx += (size_t)written;

    if (!append_fixed(line, sizeof(line), &idx, det->max_temp_c, 2))
        return;
    written = snprintf(line + idx, sizeof(line) - idx, ",%u,", (unsigned)det->hot_count);
    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx)))
        return;
    idx += (size_t)written;

    if (!append_fixed(line, sizeof(line), &idx, servo_angle_deg, 1))
        return;
    written = snprintf(line + idx, sizeof(line) - idx, "\r\n");
    if ((written <= 0) || ((size_t)written >= (sizeof(line) - idx)))
        return;
    idx += (size_t)written;

    HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)idx, 200);
}

static int16_t to_fixed100(float v)
{
    return (int16_t)(v * 100.0f);
}

static uint8_t quantize_u8(float t)
{
    float n = (t - Q_TEMP_MIN_C) / (Q_TEMP_MAX_C - Q_TEMP_MIN_C);
    if (n < 0.0f)
        n = 0.0f;
    if (n > 1.0f)
        n = 1.0f;
    return (uint8_t)(n * 255.0f);
}

void uart_send_um64_packet(uint16_t seq, const float *up, const ThermalObjectsResult *objs,
                           uint8_t selected_idx, int16_t servo_deg_x10)
{
    uint8_t header[20];
    uint8_t objtab[STREAM_OBJ_COUNT * 12U];
    uint16_t payload_len = (uint16_t)(UPSCALE_W * UPSCALE_H);
    static uint8_t payload[UPSCALE_W * UPSCALE_H];

    for (uint16_t i = 0; i < payload_len; i++) {
        payload[i] = quantize_u8(up[i]);
    }

    int16_t avg = to_fixed100(objs->avg_temp_c);
    int16_t thr = to_fixed100(objs->threshold_c);
    int16_t mx  = to_fixed100(objs->max_temp_c);

    header[0]  = 'U';
    header[1]  = 'M';
    header[2]  = '6';
    header[3]  = '4';
    header[4]  = (uint8_t)(seq & 0xFF);
    header[5]  = (uint8_t)((seq >> 8) & 0xFF);
    header[6]  = (uint8_t)UPSCALE_W;
    header[7]  = (uint8_t)UPSCALE_H;
    header[8]  = objs->count;
    header[9]  = (objs->count > 0U) ? selected_idx : 0xFFU;
    header[10] = (uint8_t)(avg & 0xFF);
    header[11] = (uint8_t)((avg >> 8) & 0xFF);
    header[12] = (uint8_t)(thr & 0xFF);
    header[13] = (uint8_t)((thr >> 8) & 0xFF);
    header[14] = (uint8_t)(mx & 0xFF);
    header[15] = (uint8_t)((mx >> 8) & 0xFF);
    header[16] = (uint8_t)(servo_deg_x10 & 0xFF);
    header[17] = (uint8_t)((servo_deg_x10 >> 8) & 0xFF);
    header[18] = (uint8_t)(payload_len & 0xFF);
    header[19] = (uint8_t)((payload_len >> 8) & 0xFF);

    for (uint8_t i = 0U; i < STREAM_OBJ_COUNT; i++) {
        const ThermalObject *obj = &objs->objects[i];
        uint8_t *p               = &objtab[i * 12U];
        p[0]                     = obj->valid;
        p[1]                     = (uint8_t)obj->min_x;
        p[2]                     = (uint8_t)obj->max_x;
        p[3]                     = (uint8_t)obj->min_y;
        p[4]                     = (uint8_t)obj->max_y;
        {
            int16_t cx = to_fixed100(obj->centroid_x);
            int16_t cy = to_fixed100(obj->centroid_y);
            int16_t pk = to_fixed100(obj->peak_temp_c);
            p[5]       = (uint8_t)(cx & 0xFF);
            p[6]       = (uint8_t)((cx >> 8) & 0xFF);
            p[7]       = (uint8_t)(cy & 0xFF);
            p[8]       = (uint8_t)((cy >> 8) & 0xFF);
            p[9]       = (uint8_t)(pk & 0xFF);
            p[10]      = (uint8_t)((pk >> 8) & 0xFF);
        }
        p[11] = (obj->hot_count > 255U) ? 255U : (uint8_t)obj->hot_count;
    }

    HAL_UART_Transmit(&huart3, header, sizeof(header), 200);
    HAL_UART_Transmit(&huart3, objtab, sizeof(objtab), 200);
    HAL_UART_Transmit(&huart3, payload, payload_len, 400);
}




