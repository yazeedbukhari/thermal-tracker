/*
 * fsm.h — System finite state machine (TRACK / SEEK / FALLBACK / MANUAL)
 *
 * Manages multi-object selection, seek-other logic, and fallback reassignment.
 * Consumes per-frame thermal objects + button events, produces a selected
 * object index and ThermalDetection for the downstream servo tracker.
 *
 * Public API:  FSM_Init, FSM_Update  (+ FSM_Input / FSM_Output structs)
 *
 * Owner:
 */

#ifndef INC_FSM_H_
#define INC_FSM_H_

#include "joystick.h"
#include "thermal.h"
#include <stdbool.h>
#include <stdint.h>

/* Tuning constants */
#define FSM_OBJ_ASSOC_MAX_DIST     2.6f  /* max manhattan dist for ID match  */
#define FSM_OBJ_TRACK_STALE_MS     1500U /* drop track after this silence    */
#define FSM_SEEK_MIN_MANHATTAN     1.4f  /* min dist to accept "other" obj   */
#define FSM_SEEK_PRE_SCAN_WAIT_MS  2200U /* grace before forcing lost-target */
#define FSM_SEEK_ACCEPT_VISIBLE_MS 600U  /* lock visible obj after this      */
#define FSM_FALLBACK_LAG_MS        1200U /* wait before reassigning to [0]   */

extern int state_manual;

typedef enum {
    FSM_STATE_TRACK = 0, /* locked onto selected object ID            */
    FSM_STATE_SEEK,      /* button pressed, scanning for different obj */
    FSM_STATE_FALLBACK,  /* selected obj lost, waiting to reassign     */
    FSM_STATE_MANUAL,    /* joystick control (dedicated button)        */
} fsm_state_t;

/* Per-frame inputs consumed by FSM_Update */
typedef struct {
    const ThermalObjectsResult *objs; /* multi-object detection result     */
    uint32_t now_ms;                  /* HAL_GetTick()                     */
    bool btn_next;                    /* rising edge: cycle/seek btn */
    bool btn_released;                /* rising edge: manual toggle  */
    JoystickReading joy;
} FSM_Input;

/* Per-frame outputs produced by FSM_Update */
typedef struct {
    fsm_state_t state;
    uint8_t selected_idx;       /* index into objs->objects[], 0xFF=none */
    ThermalDetection det;       /* detection for selected object         */
    float manual_vx, manual_vy; /* joystick pass-through (MANUAL)    */
    bool laser_on;
} FSM_Output;

void FSM_Init(void);
void FSM_Update(const FSM_Input *in, FSM_Output *out);

#endif /* INC_FSM_H_ */
