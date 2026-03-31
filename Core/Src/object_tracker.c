/*
 * object_tracker.c — Frame-to-frame object ID association and seek FSM
 *
 * Associates detected thermal objects to stable IDs using nearest-neighbour
 * matching on centroid Manhattan distance. Implements the seek-other FSM and
 * target-selection logic that was previously embedded in main.c.
 */

#include "object_tracker.h"
#include <stddef.h>

/* ── Internal types ──────────────────────────────────────────────────────── */

typedef struct {
  uint8_t  active;
  float    cx;
  float    cy;
  uint32_t last_seen_ms;
} ObjTrack;

/* ── Module state ────────────────────────────────────────────────────────── */

static ObjTrack g_tracks[THERMAL_MAX_OBJECTS];
static uint8_t  g_obj_ids[THERMAL_MAX_OBJECTS];

static uint8_t  g_selected_object    = 0xFFU; /* frame index, 0xFF = none   */
static uint8_t  g_selected_object_id = OBJ_ID_NONE;

static volatile uint8_t  g_next_object_request    = 0U;
static volatile uint8_t  g_seek_other_object       = 0U;

static uint8_t  g_seek_has_ref              = 0U;
static float    g_seek_ref_cx               = 0.0f;
static float    g_seek_ref_cy               = 0.0f;
static uint32_t g_selected_missing_since_ms = 0U;
static uint32_t g_seek_enter_ms             = 0U;
static uint32_t g_seek_visible_since_ms     = 0U;

/* ── Private helpers ─────────────────────────────────────────────────────── */

static float absf_local(float v)
{
  return (v < 0.0f) ? -v : v;
}

static void seek_state_reset(void)
{
  g_seek_other_object        = 0U;
  g_seek_has_ref             = 0U;
  g_selected_missing_since_ms = 0U;
  g_seek_enter_ms            = 0U;
  g_seek_visible_since_ms    = 0U;
}

static uint8_t track_find_free(uint32_t now_ms, const uint8_t used[THERMAL_MAX_OBJECTS])
{
  for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
    if ((used[t] == 0U) &&
        ((g_tracks[t].active == 0U) ||
         ((now_ms - g_tracks[t].last_seen_ms) > OBJ_TRACK_STALE_MS))) {
      return t;
    }
  }
  for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
    if (used[t] == 0U) return t;
  }
  return 0U;
}

static int8_t find_obj_index_by_id(const ThermalObjectsResult *objs, uint8_t id)
{
  if (id == OBJ_ID_NONE) return -1;
  for (uint8_t i = 0U; i < objs->count; i++) {
    if ((objs->objects[i].valid != 0U) && (g_obj_ids[i] == id)) return (int8_t)i;
  }
  return -1;
}

static uint8_t pick_next_visible_id(const ThermalObjectsResult *objs, uint8_t current_id)
{
  uint8_t ids[THERMAL_MAX_OBJECTS];
  uint8_t n = 0U;
  for (uint8_t i = 0U; i < objs->count; i++) {
    if (objs->objects[i].valid == 0U) continue;
    ids[n++] = g_obj_ids[i];
  }
  if (n == 0U) return OBJ_ID_NONE;
  if (n == 1U) return ids[0];
  for (uint8_t i = 0U; i < n; i++) {
    if (ids[i] == current_id) return ids[(uint8_t)((i + 1U) % n)];
  }
  return ids[0];
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void ObjTracker_Init(void)
{
  for (uint8_t i = 0U; i < THERMAL_MAX_OBJECTS; i++) {
    g_tracks[i].active       = 0U;
    g_tracks[i].cx           = 0.0f;
    g_tracks[i].cy           = 0.0f;
    g_tracks[i].last_seen_ms = 0U;
    g_obj_ids[i]             = OBJ_ID_NONE;
  }
  g_selected_object    = 0xFFU;
  g_selected_object_id = OBJ_ID_NONE;
  seek_state_reset();
}

/*
 * ObjTracker_Associate — nearest-neighbour ID assignment for current frame.
 * Must be called once per frame immediately after Thermal_DetectObjects8x8.
 */
void ObjTracker_Associate(const ThermalObjectsResult *objs, uint32_t now_ms)
{
  uint8_t used_track[THERMAL_MAX_OBJECTS] = {0U};

  for (uint8_t i = 0U; i < THERMAL_MAX_OBJECTS; i++) {
    g_obj_ids[i] = OBJ_ID_NONE;
  }

  /* Pass 1: match existing tracks to current objects. */
  for (uint8_t i = 0U; i < objs->count; i++) {
    const ThermalObject *obj = &objs->objects[i];
    if (obj->valid == 0U) continue;

    float   best    = 9999.0f;
    uint8_t best_id = OBJ_ID_NONE;

    for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
      if ((g_tracks[t].active == 0U) || (used_track[t] != 0U)) continue;
      if ((now_ms - g_tracks[t].last_seen_ms) > OBJ_TRACK_STALE_MS) continue;

      float d = absf_local(obj->centroid_x - g_tracks[t].cx) +
                absf_local(obj->centroid_y - g_tracks[t].cy);
      if ((d < best) && (d <= OBJ_ASSOC_MAX_DIST)) {
        best    = d;
        best_id = t;
      }
    }

    if (best_id != OBJ_ID_NONE) {
      g_obj_ids[i]                   = best_id;
      used_track[best_id]            = 1U;
      g_tracks[best_id].active       = 1U;
      g_tracks[best_id].cx           = obj->centroid_x;
      g_tracks[best_id].cy           = obj->centroid_y;
      g_tracks[best_id].last_seen_ms = now_ms;
    }
  }

  /* Pass 2: assign new IDs to unmatched objects. */
  for (uint8_t i = 0U; i < objs->count; i++) {
    const ThermalObject *obj = &objs->objects[i];
    if ((obj->valid == 0U) || (g_obj_ids[i] != OBJ_ID_NONE)) continue;

    uint8_t id = track_find_free(now_ms, used_track);
    g_obj_ids[i]             = id;
    used_track[id]           = 1U;
    g_tracks[id].active      = 1U;
    g_tracks[id].cx          = obj->centroid_x;
    g_tracks[id].cy          = obj->centroid_y;
    g_tracks[id].last_seen_ms = now_ms;
  }

  /* Expire stale tracks. */
  for (uint8_t t = 0U; t < THERMAL_MAX_OBJECTS; t++) {
    if ((g_tracks[t].active != 0U) &&
        ((now_ms - g_tracks[t].last_seen_ms) > OBJ_TRACK_STALE_MS)) {
      g_tracks[t].active = 0U;
    }
  }
}

const uint8_t *ObjTracker_GetObjIds(void)
{
  return g_obj_ids;
}

/* Called from button ISR — sets a flag, no FSM logic here. */
void ObjTracker_RequestNext(void)
{
  g_next_object_request = 1U;
}

/*
 * ObjTracker_Update — runs the seek FSM and resolves g_selected_object.
 * Call once per frame after ObjTracker_Associate.
 */
void ObjTracker_Update(const ThermalObjectsResult *objs, uint32_t now_ms)
{
  g_selected_object = 0xFFU;

  /* ── Handle button request ─────────────────────────────────────────────── */
  if (g_next_object_request != 0U) {
    g_next_object_request = 0U;

    if (objs->count > 1U) {
      /* Multiple objects: cycle to next visible ID. */
      g_selected_object_id = pick_next_visible_id(objs, g_selected_object_id);
      seek_state_reset();
    } else {
      /* Single/no object: toggle seek-other mode. */
      if (g_seek_other_object == 0U) {
        int8_t current_idx = find_obj_index_by_id(objs, g_selected_object_id);
        g_seek_other_object = 1U;
        g_seek_has_ref      = 0U;
        if ((current_idx >= 0) && (objs->objects[current_idx].valid != 0U)) {
          g_seek_ref_cx  = objs->objects[current_idx].centroid_x;
          g_seek_ref_cy  = objs->objects[current_idx].centroid_y;
          g_seek_has_ref = 1U;
        }
        g_seek_enter_ms         = now_ms;
        g_seek_visible_since_ms = 0U;
      } else {
        seek_state_reset();
      }
    }
  }

  /* ── Seek-other FSM ────────────────────────────────────────────────────── */
  if ((g_seek_other_object != 0U) && (objs->count > 0U)) {
    uint8_t switched = 0U;

    for (uint8_t i = 0U; i < objs->count; i++) {
      if (objs->objects[i].valid == 0U) continue;

      uint8_t id_diff    = (g_obj_ids[i] != g_selected_object_id) ? 1U : 0U;
      uint8_t far_enough = 0U;
      if (g_seek_has_ref != 0U) {
        float md = absf_local(objs->objects[i].centroid_x - g_seek_ref_cx) +
                   absf_local(objs->objects[i].centroid_y - g_seek_ref_cy);
        if (md >= SEEK_OTHER_MIN_MANHATTAN) far_enough = 1U;
      }

      if ((id_diff != 0U) || (far_enough != 0U)) {
        g_selected_object_id = g_obj_ids[i];
        seek_state_reset();
        switched = 1U;
        break;
      }
    }

    if (switched != 0U) {
      g_seek_visible_since_ms = 0U;
    } else {
      if (g_seek_visible_since_ms == 0U) g_seek_visible_since_ms = now_ms;

      /* Fallback: lock onto something stable after timeout. */
      if ((now_ms - g_seek_visible_since_ms) >= SEEK_ACCEPT_VISIBLE_MS) {
        for (uint8_t i = 0U; i < objs->count; i++) {
          if (objs->objects[i].valid != 0U) {
            g_selected_object_id = g_obj_ids[i];
            seek_state_reset();
            switched = 1U;
            break;
          }
        }
      }

      /* Fallback: no reference and multiple objects — pick next. */
      if ((switched == 0U) && (g_seek_has_ref == 0U) && (objs->count > 1U)) {
        g_selected_object_id = pick_next_visible_id(objs, g_selected_object_id);
        seek_state_reset();
      }
    }
  } else {
    g_seek_visible_since_ms = 0U;
  }

  /* ── Resolve selected index ────────────────────────────────────────────── */
  {
    int8_t sel_idx = find_obj_index_by_id(objs, g_selected_object_id);

    if (sel_idx >= 0) {
      g_selected_missing_since_ms = 0U;
    } else if ((objs->count > 0U) && (g_seek_other_object == 0U)) {
      if (g_selected_missing_since_ms == 0U) g_selected_missing_since_ms = now_ms;

      if ((now_ms - g_selected_missing_since_ms) >= SELECT_FALLBACK_LAG_MS) {
        g_selected_object_id        = g_obj_ids[0];
        sel_idx                     = 0;
        g_selected_missing_since_ms = 0U;
      }
    } else {
      g_selected_missing_since_ms = 0U;
    }

    g_selected_object = (sel_idx < 0) ? 0xFFU : (uint8_t)sel_idx;
  }
}

uint8_t ObjTracker_GetSelected(void)
{
  return g_selected_object;
}

/*
 * ObjTracker_BuildDetection — fill det from the selected object and apply
 * seek overrides (grace window / force-lost for scan trigger).
 */
void ObjTracker_BuildDetection(
  ThermalDetection *det,
  const ThermalObjectsResult *objs,
  uint32_t now_ms
)
{
  /* Initialise from frame-level stats. */
  det->target_found = 0U;
  det->min_x        = -1;
  det->max_x        = -1;
  det->min_y        = -1;
  det->max_y        = -1;
  det->centroid_x   = -1.0f;
  det->centroid_y   = -1.0f;
  det->avg_temp_c   = objs->avg_temp_c;
  det->threshold_c  = objs->threshold_c;
  det->max_temp_c   = objs->max_temp_c;
  det->hot_count    = objs->total_hot_count;

  if ((objs->count > 0U) && (g_selected_object != 0xFFU) && (g_selected_object < objs->count)) {
    const ThermalObject *obj = &objs->objects[g_selected_object];
    det->target_found = obj->valid;
    det->min_x        = obj->min_x;
    det->max_x        = obj->max_x;
    det->min_y        = obj->min_y;
    det->max_y        = obj->max_y;
    det->centroid_x   = obj->centroid_x;
    det->centroid_y   = obj->centroid_y;
  }

  /* Seek override: grace window or force-lost. */
  if ((g_seek_other_object != 0U) && (objs->count <= 1U)) {
    uint32_t seek_ms = now_ms - g_seek_enter_ms;
    if (seek_ms < SEEK_PRE_SCAN_WAIT_MS) {
      /* Grace window: stay on any visible object. */
      if ((det->target_found == 0U) && (objs->count > 0U) && (objs->objects[0].valid != 0U)) {
        const ThermalObject *obj = &objs->objects[0];
        det->target_found = obj->valid;
        det->min_x        = obj->min_x;
        det->max_x        = obj->max_x;
        det->min_y        = obj->min_y;
        det->max_y        = obj->max_y;
        det->centroid_x   = obj->centroid_x;
        det->centroid_y   = obj->centroid_y;
      }
    } else {
      /* Force lost-target to trigger scan sweep. */
      det->target_found = 0U;
      det->min_x        = -1;
      det->max_x        = -1;
      det->min_y        = -1;
      det->max_y        = -1;
      det->centroid_x   = -1.0f;
      det->centroid_y   = -1.0f;
    }
  }
}
