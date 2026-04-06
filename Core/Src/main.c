#include "main.h"
#include "amg8833.h"
#include "config.h"
#include "fsm.h"
#include "joystick.h"
#include "laser.h"
#include "servo.h"
#include "st7735_cn8_spi_test.h"
#include "thermal.h"
#include "tracking.h"
#include "uart_stream.h"
#include <stdio.h>
#include <string.h>

#define PIN_SW GPIO_PIN_3 // Pin PC_3 (A2)
#define MODE_LED_GPIO_Port GPIOD
#define MODE_LED_Pin       GPIO_PIN_13

#define AMG8833_BRINGUP_TEST       1U
#define AMG_FRAME_PERIOD_MS        50U  /* 20 FPS */
#define STREAM_MULTI_OBJECT_BINARY 0U
#define UPSCALED_STREAM_PERIOD_MS  50U  /* send each sensor frame (~20 FPS) */
#define THERMAL_SERVO_TRACK_TEST   1U
#define ST7735_CN8_SPI_BOX_TEST    0U
#define ST7735_CN8_SPI_LIVE_VIEW   1U
#define USE_FAST_NEAREST_UPSCALE   0U

/* Keep in sync with laser.c lock behavior. */
#define LASER_LOCK_TRACK_REQUIRED 1U

/* Manual joystick control profile:
 * convert analog [-1..1] into 3 discrete movement modes to reduce jitter/work. */
#define MANUAL_JOY_DEADZONE      0.10f
#define MANUAL_JOY_SLOW_EDGE     0.18f
#define MANUAL_JOY_MED_EDGE      0.30f
#define MANUAL_STEP_SLOW_DEG     0.9f
#define MANUAL_STEP_MEDIUM_DEG   2.0f
#define MANUAL_STEP_FAST_DEG     9.0f
#define MANUAL_PERIOD_SLOW_MS    60U
#define MANUAL_PERIOD_MEDIUM_MS  25U
#define MANUAL_PERIOD_FAST_MS    1U
#define MANUAL_PAN_MIN_DEG       30.0f
#define MANUAL_PAN_MAX_DEG       160.0f
#define MANUAL_TILT_MIN_DEG      30.0f
#define MANUAL_TILT_MAX_DEG      130.0f

/* Optional Kalman pass-through scaffold (kept disabled by default). */
#define USE_KALMAN_FILTER 1U

#if USE_KALMAN_FILTER
#include "kalman.h"
#define KALMAN_Q_POS 0.01f
#define KALMAN_Q_VEL 0.01f
#define KALMAN_R     0.5f

static void apply_kalman_filter(KalmanAxis *kalman_cx, KalmanAxis *kalman_cy,
                                const FSM_Output *fsm_out, ThermalDetection *filtered_det,
                                uint32_t now_ms, uint32_t *last_frame_ms)
{
    float dt_s = (float)(now_ms - *last_frame_ms) * 0.001f;
    if (dt_s < 0.001f)
        dt_s = 0.1f;
    *last_frame_ms = now_ms;

    if (fsm_out->det.target_found != 0U) {
        Kalman_Update(kalman_cx, fsm_out->det.centroid_x, dt_s);
        Kalman_Update(kalman_cy, fsm_out->det.centroid_y, dt_s);
    } else {
        Kalman_Predict(kalman_cx, dt_s);
        Kalman_Predict(kalman_cy, dt_s);
    }

    *filtered_det = fsm_out->det;
    if (fsm_out->det.target_found != 0U) {
        filtered_det->centroid_x = Kalman_GetPosition(kalman_cx);
        filtered_det->centroid_y = Kalman_GetPosition(kalman_cy);
    }
}
#endif

static volatile uint8_t g_next_object_request = 0U;
static volatile uint8_t g_btn_released_request = 0U;
static volatile uint32_t g_last_b1_ms = 0U;
static volatile uint32_t g_last_sw_ms = 0U;

/* Defined here, referenced by FSM module. */
int state_manual = 0;

static void update_mode_led(void)
{
    HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin,
                      (state_manual != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static float absf_local(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float dominant_axis_abs(const JoystickReading *r)
{
    float ax = absf_local(r->vr_x);
    float ay = absf_local(r->vr_y);
    return (ax > ay) ? ax : ay;
}

static uint32_t manual_period_ms(float joy_abs)
{
    if (joy_abs < MANUAL_JOY_DEADZONE) {
        return MANUAL_PERIOD_SLOW_MS;
    }
    if (joy_abs < MANUAL_JOY_SLOW_EDGE) {
        return MANUAL_PERIOD_SLOW_MS;
    }
    if (joy_abs < MANUAL_JOY_MED_EDGE) {
        return MANUAL_PERIOD_MEDIUM_MS;
    }
    return MANUAL_PERIOD_FAST_MS;
}

static float quantized_step(float axis)
{
    float a = absf_local(axis);
    if (a < MANUAL_JOY_DEADZONE) {
        return 0.0f;
    }

    float step = MANUAL_STEP_FAST_DEG;
    if (a < MANUAL_JOY_SLOW_EDGE) {
        step = MANUAL_STEP_SLOW_DEG;
    } else if (a < MANUAL_JOY_MED_EDGE) {
        step = MANUAL_STEP_MEDIUM_DEG;
    }
    return (axis < 0.0f) ? -step : step;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART3_UART_Init();
    Laser_Init();

#if ST7735_CN8_SPI_BOX_TEST || ST7735_CN8_SPI_LIVE_VIEW || !AMG8833_BRINGUP_TEST
    MX_DMA_Init();
#endif

#if ST7735_CN8_SPI_BOX_TEST || ST7735_CN8_SPI_LIVE_VIEW
    MX_SPI3_Init();
    ST7735_CN8_Test_Init();
#endif

#if ST7735_CN8_SPI_BOX_TEST
    ST7735_CN8_Test_DrawBox();
    uart_send("ST7735 CN8 SPI box test drawn\r\n");
    while (1) {
        HAL_Delay(1000);
    }
#endif

#if AMG8833_BRINGUP_TEST
    MX_I2C1_Init();
#if THERMAL_SERVO_TRACK_TEST
    MX_ADC1_Init();
    Joystick_Init();
    MX_TIM3_Init();
    Servo_Init();
    Tracking_Init();
    FSM_Init();
#endif
#else
    MX_USB_OTG_FS_PCD_Init();
    MX_ADC1_Init();
    MX_TIM3_Init();
    Joystick_Init();
    Servo_Init();
#endif

#if AMG8833_BRINGUP_TEST
    uint8_t amg_addr = 0;
    float frame[AMG8833_PIXEL_COUNT];
    static float upscaled[UPSCALE_W * UPSCALE_H];
    ThermalObjectsResult objs;
    FSM_Input fsm_in;
    FSM_Output fsm_out;
    ThermalDetection track_det;

    uint32_t next_frame_ms = HAL_GetTick();
    uint32_t next_manual_step_ms = next_frame_ms;
    uint8_t was_manual_mode = 0U;
#if STREAM_MULTI_OBJECT_BINARY
    uint32_t next_up_tx_ms = HAL_GetTick();
    uint16_t up_seq = 0U;
#endif

#if USE_KALMAN_FILTER
    KalmanAxis kalman_cx;
    KalmanAxis kalman_cy;
    uint32_t last_frame_ms = next_frame_ms;
    Kalman_Init(&kalman_cx, KALMAN_Q_POS, KALMAN_Q_VEL, KALMAN_R);
    Kalman_Init(&kalman_cy, KALMAN_Q_POS, KALMAN_Q_VEL, KALMAN_R);
#endif

    uart_send("AMG8833 bring-up test\r\n");
    if (AMG8833_Probe(&hi2c1, &amg_addr) != HAL_OK) {
        uart_send("ERR_PROBE (check wiring/address)\r\n");
        while (1) {
            Laser_Update(0U);
            HAL_Delay(100);
        }
    }

    if (AMG8833_Init(&hi2c1, amg_addr) != HAL_OK) {
        uart_send("ERR_INIT\r\n");
        while (1) {
            Laser_Update(0U);
            HAL_Delay(100);
        }
    }

    {
        char addr_msg[32];
        (void)snprintf(addr_msg, sizeof(addr_msg), "READY addr=0x%02X\r\n", amg_addr);
        uart_send(addr_msg);
    }

    memset(&objs, 0, sizeof(objs));
#endif

    while (1) {
#if AMG8833_BRINGUP_TEST
        uint32_t now = HAL_GetTick();
        update_mode_led();

        if ((state_manual != 0) || (g_next_object_request != 0U) || (g_btn_released_request != 0U)) {
            JoystickReading joy = read_joystick_adc();

            fsm_in.objs = &objs;
            fsm_in.now_ms = now;
            fsm_in.btn_next = (g_next_object_request != 0U);
            fsm_in.btn_released = (g_btn_released_request != 0U);
            fsm_in.joy = joy;
            g_next_object_request = 0U;
            g_btn_released_request = 0U;

            FSM_Update(&fsm_in, &fsm_out);

#if THERMAL_SERVO_TRACK_TEST
            if (fsm_out.state == FSM_STATE_MANUAL) {
                Tracking_Enable(0U);

                float joy_abs = dominant_axis_abs(&joy);
                uint32_t period_ms = manual_period_ms(joy_abs);
                if ((int32_t)(now - next_manual_step_ms) >= 0) {
                    float next_pan;
                    float next_tilt;
                    next_manual_step_ms = now + period_ms;

                    next_pan = Servo_GetPan() + quantized_step(joy.vr_x);
                    next_tilt = Servo_GetTilt() + quantized_step(joy.vr_y);
                    if (next_pan < MANUAL_PAN_MIN_DEG) {
                        next_pan = MANUAL_PAN_MIN_DEG;
                    }
                    if (next_pan > MANUAL_PAN_MAX_DEG) {
                        next_pan = MANUAL_PAN_MAX_DEG;
                    }
                    if (next_tilt < MANUAL_TILT_MIN_DEG) {
                        next_tilt = MANUAL_TILT_MIN_DEG;
                    }
                    if (next_tilt > MANUAL_TILT_MAX_DEG) {
                        next_tilt = MANUAL_TILT_MAX_DEG;
                    }

                    Servo_SetPan(next_pan);
                    Servo_SetTilt(next_tilt);
                }
                Laser_Update(0U);
            }
#endif
        }

        if ((int32_t)(now - next_frame_ms) < 0) {
            HAL_Delay(1);
            continue;
        }
        next_frame_ms = now + AMG_FRAME_PERIOD_MS;

        if (AMG8833_ReadFrameCelsius(&hi2c1, amg_addr, frame) != HAL_OK) {
            uart_send("ERR_READ\r\n");
            Laser_Update(0U);
            continue;
        }

        Thermal_DetectObjects8x8(frame, &objs);

        fsm_in.objs = &objs;
        fsm_in.now_ms = now;
        fsm_in.btn_next = (g_next_object_request != 0U);
        fsm_in.btn_released = (g_btn_released_request != 0U);
        fsm_in.joy = read_joystick_adc();
        g_next_object_request = 0U;
        g_btn_released_request = 0U;

        FSM_Update(&fsm_in, &fsm_out);

#if THERMAL_SERVO_TRACK_TEST
        {
            uint8_t is_manual_mode = (fsm_out.state == FSM_STATE_MANUAL) ? 1U : 0U;
            if ((was_manual_mode != 0U) && (is_manual_mode == 0U)) {
                Tracking_ResetSearchTimer();
            }
            was_manual_mode = is_manual_mode;
        }
#endif

#if USE_KALMAN_FILTER
        apply_kalman_filter(&kalman_cx, &kalman_cy, &fsm_out, &track_det, now, &last_frame_ms);
#else
        track_det = fsm_out.det;
#endif

#if THERMAL_SERVO_TRACK_TEST
        if (fsm_out.state == FSM_STATE_MANUAL) {
            Tracking_Enable(0U);
        } else {
            Tracking_Enable(1U);
            Tracking_UpdateFromDetection(&track_det);
        }
#endif

        {
            uint8_t laser_locked = 0U;
#if LASER_LOCK_TRACK_REQUIRED
            if ((fsm_out.state == FSM_STATE_TRACK) && (track_det.target_found != 0U)) {
                laser_locked = 1U;
            }
#else
            if (track_det.target_found != 0U) {
                laser_locked = 1U;
            }
#endif
            Laser_Update(laser_locked);
        }

#if USE_FAST_NEAREST_UPSCALE
        Thermal_UpscaleNearest8x8(frame, upscaled, UPSCALE_W, UPSCALE_H);
#else
        Thermal_UpscaleBilinear8x8(frame, upscaled, UPSCALE_W, UPSCALE_H);
#endif

#if ST7735_CN8_SPI_LIVE_VIEW
        ST7735_CN8_RenderFrame32x32(upscaled, &objs, fsm_out.selected_idx);
#endif

#if STREAM_MULTI_OBJECT_BINARY
        if ((int32_t)(now - next_up_tx_ms) >= 0) {
            next_up_tx_ms = now + UPSCALED_STREAM_PERIOD_MS;
            uart_send_um64_packet(up_seq++, upscaled, &objs, fsm_out.selected_idx,
                                  (int16_t)(Servo_GetPan() * 10.0f));
        }
#elif !ST7735_CN8_SPI_LIVE_VIEW
        uart_send("BEGIN\r\n");
        uart_send_frame_csv(frame);
        uart_send_meta_csv(&track_det, Servo_GetPan());
        uart_send("END\r\n");
#endif
#endif
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();
    if (GPIO_Pin == USER_Btn_Pin) {
#if AMG8833_BRINGUP_TEST
        if ((now - g_last_b1_ms) > 180U) {
            g_last_b1_ms = now;
            g_next_object_request = 1U;
        }
#endif
        return;
    }

    if (GPIO_Pin == PIN_SW) {
        if ((now - g_last_sw_ms) > 180U) {
            g_last_sw_ms = now;
            g_btn_released_request = 1U;
        }
    }
}
