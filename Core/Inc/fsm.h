/* Mode logic and target selection for the tracker. */
#ifndef INC_FSM_H_
#define INC_FSM_H_

#include "joystick.h"
#include "thermal.h"
#include <stdbool.h>
#include <stdint.h>

/* Tuning constants */
#define FSM_OBJ_ASSOC_MAX_DIST        2.6f
#define FSM_OBJ_ASSOC_TEMP_WEIGHT     0.32f
#define FSM_OBJ_ASSOC_SIZE_WEIGHT     0.08f
#define FSM_OBJ_SWITCH_TEMP_BONUS_C   0.8f
#define FSM_OBJ_TRACK_STALE_MS        1500U
#define FSM_SEEK_MIN_MANHATTAN        1.4f
#define FSM_SEEK_PRE_SCAN_WAIT_MS     2200U
#define FSM_SEEK_ACCEPT_VISIBLE_MS    50U
#define FSM_SELECTED_MISS_DEBOUNCE_MS 220U
#define FSM_FALLBACK_LAG_MS        250U

extern int state_manual;

typedef enum {
    FSM_STATE_TRACK = 0,
    FSM_STATE_SEEK,
    FSM_STATE_FALLBACK,
    FSM_STATE_MANUAL,
} fsm_state_t;

/* Per-frame inputs consumed by FSM_Update */
typedef struct {
    const ThermalObjectsResult *objs;
    uint32_t now_ms;
    bool btn_next; 
    bool btn_released;
    JoystickReading joy;
} FSM_Input;

/* Per-frame outputs produced by FSM_Update */
typedef struct {
    fsm_state_t state;
    uint8_t selected_idx;
    ThermalDetection det; 
    float manual_vx, manual_vy;
    bool laser_on;
} FSM_Output;

void FSM_Init(void);
void FSM_Update(const FSM_Input *in, FSM_Output *out);

#endif /* INC_FSM_H_ */




