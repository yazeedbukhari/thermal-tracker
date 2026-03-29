/*
 * fsm.c — System finite state machine (SCAN / TRACK / SEARCH / MANUAL)
 *
 * Consumes per-frame pipeline data (centroid, velocity, joystick, button) and
 * produces control outputs (pixel error for PID, manual overrides, laser state).
 * The FSM does not compute servo angles — it outputs errors that the PID
 * controller converts into servo deltas.
 *
 * Owner:
 */

#include "fsm.h"
#include "config.h"

typedef struct {
    fsm_state_t state;
    uint8_t     lost_frames;    /* consecutive frames without target   */
    uint32_t    search_start;   /* HAL tick when SEARCH was entered    */
    float       last_cx, last_cy; /* last-known centroid for SEARCH    */
} FSM_Ctx;

static FSM_Ctx ctx;

void FSM_Init(void)
{
    ctx.state        = FSM_STATE_SCAN;
    ctx.lost_frames  = 0;
    ctx.search_start = 0;
    ctx.last_cx      = FSM_FRAME_CENTER_X;
    ctx.last_cy      = FSM_FRAME_CENTER_Y;
}

void FSM_Update(const FSM_Input *in, FSM_Output *out)
{
    /* ------ button toggle (highest priority) ------ */
    if (in->btn_released) {
        if (ctx.state == FSM_STATE_MANUAL) {
            ctx.state       = FSM_STATE_SCAN;
            ctx.lost_frames = 0;
        } else {
            ctx.state = FSM_STATE_MANUAL;
        }
    }

    /* ------ state transitions ------ */
    switch (ctx.state) {

    case FSM_STATE_SCAN:
        if (in->target_detected) {
            ctx.state       = FSM_STATE_TRACK;
            ctx.lost_frames = 0;
        }
        break;

    case FSM_STATE_TRACK:
        if (in->target_detected) {
            ctx.lost_frames = 0;
            ctx.last_cx     = in->cx;
            ctx.last_cy     = in->cy;
        } else {
            ctx.lost_frames++;
            if (ctx.lost_frames >= FSM_LOST_THRESHOLD) {
                ctx.state        = FSM_STATE_SEARCH;
                ctx.search_start = HAL_GetTick();
            }
        }
        break;

    case FSM_STATE_SEARCH:
        if (in->target_detected) {
            ctx.state       = FSM_STATE_TRACK;
            ctx.lost_frames = 0;
        } else if ((HAL_GetTick() - ctx.search_start) >= FSM_SEARCH_TIMEOUT_MS) {
            ctx.state       = FSM_STATE_SCAN;
            ctx.lost_frames = 0;
        }
        break;

    case FSM_STATE_MANUAL:
        /* no automatic transitions — only button toggles out */
        break;
    }

    /* ------ produce outputs ------ */
    out->state      = ctx.state;
    out->err_x      = 0.0f;
    out->err_y      = 0.0f;
    out->manual_vx  = 0.0f;
    out->manual_vy  = 0.0f;
    out->laser_on   = false;

    switch (ctx.state) {

    case FSM_STATE_SCAN:
        /* servos hold position, laser off, waiting for detection */
        break;

    case FSM_STATE_TRACK:
        out->err_x    = in->cx - FSM_FRAME_CENTER_X;
        out->err_y    = in->cy - FSM_FRAME_CENTER_Y;
        out->laser_on = true;
        break;

    case FSM_STATE_SEARCH:
        /* hold error toward last-known position */
        out->err_x    = ctx.last_cx - FSM_FRAME_CENTER_X;
        out->err_y    = ctx.last_cy - FSM_FRAME_CENTER_Y;
        out->laser_on = true;
        break;

    case FSM_STATE_MANUAL:
        out->manual_vx = in->joy.vr_x;
        out->manual_vy = in->joy.vr_y;
        out->laser_on  = true;
        break;
    }
}
