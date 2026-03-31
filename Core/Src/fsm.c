/*
 * fsm.c — System finite state machine (TRACK / SEEK / FALLBACK / MANUAL)
 *
 * Manages multi-object ID association, object selection/cycling, seek-other
 * logic, and fallback reassignment.  Produces a ThermalDetection for the
 * selected object that the downstream servo tracker consumes directly.
 *
 * Owner:
 */

#include "fsm.h"
#include "config.h"
#include <string.h>

#define OBJ_ID_NONE 0xFFU

/* ---- persistent object tracks ---- */
typedef struct {
    uint8_t active;
    float cx, cy;
    uint32_t last_seen_ms;
} ObjTrack;

/* ---- internal context ---- */
typedef struct {
    fsm_state_t state;

    /* object-ID association */
    ObjTrack tracks[THERMAL_MAX_OBJECTS];
    uint8_t obj_ids[THERMAL_MAX_OBJECTS]; /* per-detection-slot → track ID */

    /* selection */
    uint8_t selected_id;  /* persistent track ID           */
    uint8_t selected_idx; /* index into current objs[]     */

    /* seek state */
    uint8_t seek_has_ref;
    float seek_ref_cx, seek_ref_cy;
    uint32_t seek_enter_ms;
    uint32_t seek_visible_since_ms;

    /* fallback state */
    uint32_t fallback_enter_ms;
} FSM_Ctx;

static FSM_Ctx ctx;

/* ---- helpers ---- */

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

static uint8_t track_find_free(uint32_t now_ms, const uint8_t used[THERMAL_MAX_OBJECTS])
{
    for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
        if ((used[t] == 0U) && ((ctx.tracks[t].active == 0U) ||
                                ((now_ms - ctx.tracks[t].last_seen_ms) > FSM_OBJ_TRACK_STALE_MS)))
            return t;
    }
    for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
        if (used[t] == 0U)
            return t;
    }
    return 0U;
}

static void associate_ids(const ThermalObjectsResult *objs, uint32_t now_ms)
{
    uint8_t used[THERMAL_MAX_OBJECTS] = {0U};

    for (uint8_t i = 0U; i < THERMAL_MAX_OBJECTS; i++)
        ctx.obj_ids[i] = OBJ_ID_NONE;

    /* match existing tracks */
    for (uint8_t i = 0U; i < objs->count; i++) {
        const ThermalObject *obj = &objs->objects[i];
        if (obj->valid == 0U)
            continue;

        float best      = 9999.0f;
        uint8_t best_id = OBJ_ID_NONE;
        for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
            if ((ctx.tracks[t].active == 0U) || (used[t] != 0U))
                continue;
            if ((now_ms - ctx.tracks[t].last_seen_ms) > FSM_OBJ_TRACK_STALE_MS)
                continue;
            float d =
                absf(obj->centroid_x - ctx.tracks[t].cx) + absf(obj->centroid_y - ctx.tracks[t].cy);
            if ((d < best) && (d <= FSM_OBJ_ASSOC_MAX_DIST)) {
                best    = d;
                best_id = t;
            }
        }
        if (best_id != OBJ_ID_NONE) {
            ctx.obj_ids[i]                   = best_id;
            used[best_id]                    = 1U;
            ctx.tracks[best_id].active       = 1U;
            ctx.tracks[best_id].cx           = obj->centroid_x;
            ctx.tracks[best_id].cy           = obj->centroid_y;
            ctx.tracks[best_id].last_seen_ms = now_ms;
        }
    }

    /* assign new tracks to unmatched detections */
    for (uint8_t i = 0U; i < objs->count; i++) {
        const ThermalObject *obj = &objs->objects[i];
        if ((obj->valid == 0U) || (ctx.obj_ids[i] != OBJ_ID_NONE))
            continue;

        uint8_t id                  = track_find_free(now_ms, used);
        ctx.obj_ids[i]              = id;
        used[id]                    = 1U;
        ctx.tracks[id].active       = 1U;
        ctx.tracks[id].cx           = obj->centroid_x;
        ctx.tracks[id].cy           = obj->centroid_y;
        ctx.tracks[id].last_seen_ms = now_ms;
    }

    /* expire stale tracks */
    for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
        if ((ctx.tracks[t].active != 0U) &&
            ((now_ms - ctx.tracks[t].last_seen_ms) > FSM_OBJ_TRACK_STALE_MS))
            ctx.tracks[t].active = 0U;
    }
}

static int8_t find_idx_by_id(const ThermalObjectsResult *objs, uint8_t id)
{
    if (id == OBJ_ID_NONE)
        return -1;
    for (uint8_t i = 0U; i < objs->count; i++) {
        if ((objs->objects[i].valid != 0U) && (ctx.obj_ids[i] == id))
            return (int8_t)i;
    }
    return -1;
}

static uint8_t pick_next_visible_id(const ThermalObjectsResult *objs, uint8_t current_id)
{
    uint8_t ids[THERMAL_MAX_OBJECTS];
    uint8_t n = 0U;
    for (uint8_t i = 0U; i < objs->count; i++) {
        if (objs->objects[i].valid == 0U)
            continue;
        ids[n++] = ctx.obj_ids[i];
    }
    if (n == 0U)
        return OBJ_ID_NONE;
    if (n == 1U)
        return ids[0];
    for (uint8_t i = 0U; i < n; i++) {
        if (ids[i] == current_id)
            return ids[(uint8_t)((i + 1U) % n)];
    }
    return ids[0];
}

static void fill_detection(ThermalDetection *det, const ThermalObjectsResult *objs, uint8_t idx)
{
    det->avg_temp_c  = objs->avg_temp_c;
    det->threshold_c = objs->threshold_c;
    det->max_temp_c  = objs->max_temp_c;
    det->hot_count   = objs->total_hot_count;

    if ((idx == 0xFFU) || (idx >= objs->count) || (objs->objects[idx].valid == 0U)) {
        det->target_found = 0U;
        det->min_x        = -1;
        det->max_x        = -1;
        det->min_y        = -1;
        det->max_y        = -1;
        det->centroid_x   = -1.0f;
        det->centroid_y   = -1.0f;
        return;
    }

    const ThermalObject *obj = &objs->objects[idx];
    det->target_found        = obj->valid;
    det->min_x               = obj->min_x;
    det->max_x               = obj->max_x;
    det->min_y               = obj->min_y;
    det->max_y               = obj->max_y;
    det->centroid_x          = obj->centroid_x;
    det->centroid_y          = obj->centroid_y;
}

/* ---- state helpers ---- */

static void handle_manual_toggle(const FSM_Input *in, uint32_t now)
{
    if (!in->btn_released)
        return;

    if (ctx.state == FSM_STATE_MANUAL) {
        ctx.state             = FSM_STATE_FALLBACK;
        ctx.fallback_enter_ms = now;
        state_manual          = 0;
    } else {
        ctx.state    = FSM_STATE_MANUAL;
        state_manual = 1;
    }
}

static void handle_btn_next(const FSM_Input *in, const ThermalObjectsResult *objs, uint32_t now)
{
    if (!(in->btn_next) || (ctx.state == FSM_STATE_MANUAL))
        return;

    if (objs->count > 1U) {
        /* multiple objects visible: cycle to next */
        ctx.selected_id           = pick_next_visible_id(objs, ctx.selected_id);
        ctx.state                 = FSM_STATE_TRACK;
        ctx.fallback_enter_ms     = 0U;
        ctx.seek_enter_ms         = 0U;
        ctx.seek_has_ref          = 0U;
        ctx.seek_visible_since_ms = 0U;
    } else {
        /* single/no object: toggle seek mode */
        if (ctx.state == FSM_STATE_SEEK) {
            /* cancel seek → back to track */
            ctx.state                 = FSM_STATE_TRACK;
            ctx.seek_has_ref          = 0U;
            ctx.seek_enter_ms         = 0U;
            ctx.seek_visible_since_ms = 0U;
        } else {
            ctx.state                 = FSM_STATE_SEEK;
            ctx.seek_enter_ms         = now;
            ctx.seek_visible_since_ms = 0U;
            ctx.seek_has_ref          = 0U;

            int8_t ci = find_idx_by_id(objs, ctx.selected_id);
            if ((ci >= 0) && (objs->objects[ci].valid != 0U)) {
                ctx.seek_ref_cx  = objs->objects[ci].centroid_x;
                ctx.seek_ref_cy  = objs->objects[ci].centroid_y;
                ctx.seek_has_ref = 1U;
            }
        }
    }
}

static void update_track_state(const ThermalObjectsResult *objs, uint32_t now)
{
    int8_t si = find_idx_by_id(objs, ctx.selected_id);
    if (si >= 0) {
        ctx.selected_idx = (uint8_t)si;
    } else if (objs->count > 0U) {
        /* selected ID not visible — enter fallback */
        ctx.state             = FSM_STATE_FALLBACK;
        ctx.fallback_enter_ms = now;
        ctx.selected_idx      = 0xFFU;
    } else {
        ctx.selected_idx = 0xFFU;
    }
}

static void update_seek_state(const ThermalObjectsResult *objs, uint32_t now)
{
    uint8_t switched = 0U;

    /* look for a different / far-enough object */
    if (objs->count > 0U) {
        for (uint8_t i = 0U; i < objs->count; i++) {
            if (objs->objects[i].valid == 0U)
                continue;

            uint8_t id_diff    = (ctx.obj_ids[i] != ctx.selected_id) ? 1U : 0U;
            uint8_t far_enough = 0U;
            if (ctx.seek_has_ref != 0U) {
                float md = absf(objs->objects[i].centroid_x - ctx.seek_ref_cx) +
                           absf(objs->objects[i].centroid_y - ctx.seek_ref_cy);
                if (md >= FSM_SEEK_MIN_MANHATTAN)
                    far_enough = 1U;
            }

            if ((id_diff != 0U) || (far_enough != 0U)) {
                ctx.selected_id           = ctx.obj_ids[i];
                ctx.state                 = FSM_STATE_TRACK;
                ctx.seek_has_ref          = 0U;
                ctx.seek_enter_ms         = 0U;
                ctx.seek_visible_since_ms = 0U;
                ctx.fallback_enter_ms     = 0U;
                switched                  = 1U;
                break;
            }
        }

        if (switched == 0U) {
            /* stable-visible fallback */
            if (ctx.seek_visible_since_ms == 0U)
                ctx.seek_visible_since_ms = now;

            if ((now - ctx.seek_visible_since_ms) >= FSM_SEEK_ACCEPT_VISIBLE_MS) {
                for (uint8_t i = 0U; i < objs->count; i++) {
                    if (objs->objects[i].valid != 0U) {
                        ctx.selected_id           = ctx.obj_ids[i];
                        ctx.state                 = FSM_STATE_TRACK;
                        ctx.seek_has_ref          = 0U;
                        ctx.seek_enter_ms         = 0U;
                        ctx.seek_visible_since_ms = 0U;
                        ctx.fallback_enter_ms     = 0U;
                        switched                  = 1U;
                        break;
                    }
                }
            }

            /* no-ref multi-object fallback */
            if ((switched == 0U) && (ctx.seek_has_ref == 0U) && (objs->count > 1U)) {
                ctx.selected_id           = pick_next_visible_id(objs, ctx.selected_id);
                ctx.state                 = FSM_STATE_TRACK;
                ctx.seek_has_ref          = 0U;
                ctx.seek_enter_ms         = 0U;
                ctx.seek_visible_since_ms = 0U;
                ctx.fallback_enter_ms     = 0U;
                switched                  = 1U;
            }
        }
    } else {
        ctx.seek_visible_since_ms = 0U;
    }

    /* resolve selected_idx for output */
    if (switched != 0U) {
        int8_t si        = find_idx_by_id(objs, ctx.selected_id);
        ctx.selected_idx = (si >= 0) ? (uint8_t)si : 0xFFU;
    } else {
        /* still seeking — decide what to show */
        uint32_t seek_ms = now - ctx.seek_enter_ms;
        if ((seek_ms < FSM_SEEK_PRE_SCAN_WAIT_MS) && (objs->count > 0U) &&
            (objs->objects[0].valid != 0U)) {
            /* grace window: keep tracking any visible object */
            ctx.selected_idx = 0U;
        } else {
            /* after grace: force lost to trigger scan behavior */
            ctx.selected_idx = 0xFFU;
        }
    }
}

static void update_fallback_state(const ThermalObjectsResult *objs, uint32_t now)
{
    int8_t si = find_idx_by_id(objs, ctx.selected_id);
    if (si >= 0) {
        /* selected came back */
        ctx.state             = FSM_STATE_TRACK;
        ctx.selected_idx      = (uint8_t)si;
        ctx.fallback_enter_ms = 0U;
    } else if (objs->count > 0U) {
        if ((now - ctx.fallback_enter_ms) >= FSM_FALLBACK_LAG_MS) {
            /* timeout — reassign to first valid */
            ctx.selected_id       = ctx.obj_ids[0];
            ctx.selected_idx      = 0U;
            ctx.state             = FSM_STATE_TRACK;
            ctx.fallback_enter_ms = 0U;
        } else {
            ctx.selected_idx = 0xFFU;
        }
    } else {
        ctx.selected_idx = 0xFFU;
    }
}

static void produce_output(const FSM_Input *in, const ThermalObjectsResult *objs, FSM_Output *out)
{
    out->state        = ctx.state;
    out->selected_idx = ctx.selected_idx;
    out->manual_vx    = 0.0f;
    out->manual_vy    = 0.0f;
    out->laser_on     = false;

    fill_detection(&out->det, objs, ctx.selected_idx);

    switch (ctx.state) {
    case FSM_STATE_TRACK:
        out->laser_on = (ctx.selected_idx != 0xFFU);
        break;
    case FSM_STATE_SEEK:
        out->laser_on = (ctx.selected_idx != 0xFFU);
        break;
    case FSM_STATE_FALLBACK:
        out->laser_on = false;
        break;
    case FSM_STATE_MANUAL:
        out->manual_vx = in->joy.vr_x;
        out->manual_vy = in->joy.vr_y;
        out->laser_on  = true;
        break;
    }
}

/* ---- public API ---- */

void FSM_Init(void)
{
    memset(&ctx, 0, sizeof(ctx));
    ctx.state        = FSM_STATE_TRACK;
    ctx.selected_id  = OBJ_ID_NONE;
    ctx.selected_idx = 0xFFU;
}

void FSM_Update(const FSM_Input *in, FSM_Output *out)
{
    const ThermalObjectsResult *objs = in->objs;
    uint32_t now                     = in->now_ms;

    /* ---- ID association (runs every frame) ---- */
    associate_ids(objs, now);

    /* ---- input processing (in priority order) ---- */
    handle_manual_toggle(in, now);
    handle_btn_next(in, objs, now);

    /* ---- state machine update ---- */
    switch (ctx.state) {
    case FSM_STATE_TRACK:
        update_track_state(objs, now);
        break;
    case FSM_STATE_SEEK:
        update_seek_state(objs, now);
        break;
    case FSM_STATE_FALLBACK:
        update_fallback_state(objs, now);
        break;
    case FSM_STATE_MANUAL:
        ctx.selected_idx = 0xFFU;
        break;
    }

    /* ---- produce outputs ---- */
    produce_output(in, objs, out);
}
