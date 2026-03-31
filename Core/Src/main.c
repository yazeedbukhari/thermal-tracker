/*
 * main.c — Application entry point and super loop
 *
 * Initializes all peripherals, then runs the 10 Hz frame-synchronous pipeline:
 *   DMA frame ready → swap buffers → interpolate → centroid → [Kalman] →
 *   FSM → [PID] → servo update → laser control → UART stream
 *
 * ISR callbacks for I2C DMA complete and EXTI (USER button) live here or
 * in stm32f4xx_it.c. The loop spins on AMG8833_FrameReady() between frames.
 */

#include "main.h"
#include "amg8833.h"
#include "config.h"
#include "fsm.h"
#include "joystick.h"
#include "kalman.h"
#include "servo.h"
#include "st7735_cn8_spi_test.h"
#include "thermal.h"
#include "tracking.h"
#include "uart_stream.h"
#include <stdio.h>

#define PIN_SW                     GPIO_PIN_3 // Pin PC_3 (A2)
#define AMG8833_BRINGUP_TEST       1U
#define AMG_FRAME_PERIOD_MS        100U /* AMG8833 configured at 10 FPS */
#define STREAM_MULTI_OBJECT_BINARY 1U
#define UPSCALED_STREAM_PERIOD_MS  100U /* send each sensor frame (~10 FPS) */
#define THERMAL_SERVO_TRACK_TEST   1U
#define ST7735_CN8_SPI_BOX_TEST    0U
#define ST7735_CN8_SPI_LIVE_VIEW   1U

/* Kalman filter tuning — process and measurement noise */
#define KALMAN_Q_POS 0.01f /* process noise: position (tune for responsiveness) */
#define KALMAN_Q_VEL 0.01f /* process noise: velocity                          */
#define KALMAN_R     0.5f  /* measurement noise: centroid jitter               */

static volatile uint8_t g_next_object_request = 0U;
static volatile uint32_t g_last_b1_ms         = 0U;

int state_manual = 0;

/* ---- Kalman helper ---- */

static void apply_kalman_filter(KalmanAxis *kalman_cx, KalmanAxis *kalman_cy,
                                const FSM_Output *fsm_out, ThermalDetection *filtered_det,
                                uint32_t now_ms, uint32_t *last_frame_ms)
{
    float dt_s = (float)(now_ms - *last_frame_ms) * 0.001f;
    if (dt_s < 0.001f)
        dt_s = 0.1f; /* fallback for first frame */
    *last_frame_ms = now_ms;

    if (fsm_out->det.target_found != 0U) {
        /* Target visible: update filters with measurement */
        Kalman_Update(kalman_cx, fsm_out->det.centroid_x, dt_s);
        Kalman_Update(kalman_cy, fsm_out->det.centroid_y, dt_s);
    } else {
        /* Target lost: prediction-only (coast with velocity estimate) */
        Kalman_Predict(kalman_cx, dt_s);
        Kalman_Predict(kalman_cy, dt_s);
    }

    /* Copy filtered detection for downstream (Tracking module) */
    *filtered_det = fsm_out->det;
    if (fsm_out->det.target_found != 0U) {
        filtered_det->centroid_x = Kalman_GetPosition(kalman_cx);
        filtered_det->centroid_y = Kalman_GetPosition(kalman_cy);
    }
}

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART3_UART_Init();

#if ST7735_CN8_SPI_BOX_TEST || ST7735_CN8_SPI_LIVE_VIEW
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
#    if THERMAL_SERVO_TRACK_TEST
    MX_TIM3_Init();
    Servo_Init();
    Tracking_Init();
    FSM_Init();
#    endif
#else
    MX_USB_OTG_FS_PCD_Init();
    MX_DMA_Init();   // must come before ADC init
    MX_ADC1_Init();  // Initialize ADC1 for joystick (PA3=VRx, PC0=VRy)
    MX_TIM3_Init();  // Initialize TIM3 PWM for servos (PA6=pan, PA7=tilt)
    Joystick_Init(); // Start ADC1 DMA2 continuous scan
    Servo_Init();    // Start TIM3 PWM, center both servos at 90°
#endif

#if AMG8833_BRINGUP_TEST
    uint8_t amg_addr = 0;
    float frame[AMG8833_PIXEL_COUNT];
    static float upscaled[UPSCALE_W * UPSCALE_H];
    ThermalObjectsResult objs;
    FSM_Input fsm_in;
    FSM_Output fsm_out;

    /* Kalman filters: one per axis for centroid smoothing */
    KalmanAxis kalman_cx, kalman_cy;
    Kalman_Init(&kalman_cx, KALMAN_Q_POS, KALMAN_Q_VEL, KALMAN_R);
    Kalman_Init(&kalman_cy, KALMAN_Q_POS, KALMAN_Q_VEL, KALMAN_R);

    ThermalDetection filtered_det; /* Kalman output goes here */

    uint32_t next_frame_ms    = HAL_GetTick();
    uint32_t last_frame_ms    = next_frame_ms;
    uint8_t tft_render_toggle = 0U;
#    if STREAM_MULTI_OBJECT_BINARY && !ST7735_CN8_SPI_LIVE_VIEW
    uint32_t next_up_tx_ms = HAL_GetTick();
    uint16_t up_seq        = 0;
#    endif

    uart_send("AMG8833 bring-up test\r\n");
    if (AMG8833_Probe(&hi2c1, &amg_addr) != HAL_OK) {
        uart_send("ERR_PROBE (check wiring/address)\r\n");
        while (1) {
            HAL_Delay(100);
        }
    }

    if (AMG8833_Init(&hi2c1, amg_addr) != HAL_OK) {
        uart_send("ERR_INIT\r\n");
        while (1) {
            HAL_Delay(100);
        }
    }

    char addr_msg[32];
    (void)snprintf(addr_msg, sizeof(addr_msg), "READY addr=0x%02X\r\n", amg_addr);
    uart_send(addr_msg);
#endif

    while (1) {
        if (state_manual) {
            JoystickReading r = read_joystick_adc();

            Servo_SetPan(Servo_GetPan() + r.vr_x);
            Servo_SetTilt(Servo_GetTilt() + r.vr_y);
            HAL_Delay(1);
            continue;
        }

#if AMG8833_BRINGUP_TEST
        uint32_t now = HAL_GetTick();
        if ((int32_t)(now - next_frame_ms) < 0) {
            continue;
        }
        next_frame_ms = now + AMG_FRAME_PERIOD_MS;

        if (AMG8833_ReadFrameCelsius(&hi2c1, amg_addr, frame) != HAL_OK) {
            uart_send("ERR_READ\r\n");
            continue;
        }

        Thermal_DetectObjects8x8(frame, &objs);

        /* --- FSM tick --- */
        fsm_in.objs           = &objs;
        fsm_in.now_ms         = now;
        fsm_in.btn_next       = (g_next_object_request != 0U);
        fsm_in.btn_released   = false; /* TODO: wire dedicated manual button */
        fsm_in.joy.vr_x       = 0.0f;
        fsm_in.joy.vr_y       = 0.0f;
        g_next_object_request = 0U;

        FSM_Update(&fsm_in, &fsm_out);

        /* ---- Kalman filter: smooth & estimate velocity ---- */
        //apply_kalman_filter(&kalman_cx, &kalman_cy, &fsm_out, &filtered_det, now, &last_frame_ms);

#    if THERMAL_SERVO_TRACK_TEST
        Tracking_UpdateFromDetection(&fsm_out.det);
#    endif
        Thermal_UpscaleBilinear8x8(frame, upscaled, UPSCALE_W, UPSCALE_H);
#    if ST7735_CN8_SPI_LIVE_VIEW
        tft_render_toggle ^= 1U;
        if (tft_render_toggle != 0U) {
            ST7735_CN8_RenderFrame32x32(upscaled, &objs, fsm_out.selected_idx);
        }
#    endif

#    if STREAM_MULTI_OBJECT_BINARY && !ST7735_CN8_SPI_LIVE_VIEW
        if ((int32_t)(now - next_up_tx_ms) >= 0) {
            next_up_tx_ms = now + UPSCALED_STREAM_PERIOD_MS;
            uart_send_um64_packet(up_seq++, upscaled, &objs, fsm_out.selected_idx,
                                  (int16_t)(Servo_GetPan() * 10.0f));
        }
#    elif !ST7735_CN8_SPI_LIVE_VIEW
        uart_send("BEGIN\r\n");
        uart_send_frame_csv(frame);
        uart_send_meta_csv(&fsm_out.det, Servo_GetPan());
        uart_send("END\r\n");
#    endif
#endif
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == USER_Btn_Pin) {
#if AMG8833_BRINGUP_TEST
        uint32_t now = HAL_GetTick();
        if ((now - g_last_b1_ms) > 180U) {
            g_last_b1_ms          = now;
            g_next_object_request = 1U;
        }
#endif
        return;
    }

    if (GPIO_Pin == PIN_SW) {
#if AMG8833_BRINGUP_TEST
        uart_send("SW\r\n");
#else
        char msg[40];
        int pan  = (int)Servo_GetPan();
        int tilt = (int)Servo_GetTilt();
        int len  = sprintf(msg, "pan: %d, tilt: %d\r\n", pan, tilt);
        HAL_UART_Transmit(&huart3, (uint8_t *)msg, len, 100);
#endif
    }
}
