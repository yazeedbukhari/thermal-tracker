/*
 * fsm.h — System finite state machine (SCAN / TRACK / SEARCH / MANUAL)
 *
 * Consumes per-frame pipeline data (centroid, velocity, joystick, button) and
 * produces control outputs (pixel error for PID, manual overrides, laser state).
 * The FSM does not compute servo angles — it outputs errors that the PID
 * controller converts into servo deltas, keeping control law and state logic
 * decoupled.
 *
 * Public API:  FSM_Init, FSM_Update  (+ FSM_Input / FSM_Output structs)
 *
 * Owner:
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include <stdbool.h>
#include "joystick.h"

/* Frame center for 64×64 interpolated grid */
#define FSM_FRAME_CENTER_X  32.0f
#define FSM_FRAME_CENTER_Y  32.0f

/* Frames without detection before TRACK → SEARCH */
#define FSM_LOST_THRESHOLD  2

/* Milliseconds in SEARCH before giving up → SCAN */
#define FSM_SEARCH_TIMEOUT_MS  3000

typedef enum {
  FSM_STATE_SCAN = 0,
  FSM_STATE_TRACK,
  FSM_STATE_SEARCH,
  FSM_STATE_MANUAL,
} fsm_state_t;

/* Per-frame inputs consumed by FSM_Update */
typedef struct {
    bool            target_detected;
    float           cx, cy;         /* centroid in pixel space (64×64)   */
    float           vx, vy;         /* Kalman velocity estimate          */
    JoystickReading joy;
    bool            btn_released;   /* rising edge of mode-toggle button */
} FSM_Input;

/* Per-frame outputs produced by FSM_Update */
typedef struct {
    fsm_state_t state;
    float       err_x, err_y;          /* pixel error for PID           */
    float       manual_vx, manual_vy;  /* joystick pass-through (MANUAL)*/
    bool        laser_on;
} FSM_Output;

void FSM_Init(void);
void FSM_Update(const FSM_Input *in, FSM_Output *out);

#endif /* INC_FSM_H_ */
