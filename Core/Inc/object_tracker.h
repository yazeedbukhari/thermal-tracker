/*
 * object_tracker.h — Frame-to-frame object ID association and seek FSM
 *
 * Maintains stable IDs for detected thermal objects across frames using
 * nearest-neighbour association. Exposes the seek-other and target-selection
 * FSM that was previously embedded in main.c.
 *
 * Public API:
 *   ObjTracker_Init          — reset all state
 *   ObjTracker_Associate     — call once per frame after Thermal_DetectObjects8x8
 *   ObjTracker_GetObjIds     — read-only view of per-slot IDs (size THERMAL_MAX_OBJECTS)
 *   ObjTracker_RequestNext   — called from button ISR to cycle/seek target
 *   ObjTracker_Update        — run seek FSM + resolve selection, call after Associate
 *   ObjTracker_GetSelected   — frame index of currently selected object (0xFF = none)
 *   ObjTracker_BuildDetection — fill ThermalDetection from current selection
 */

#ifndef INC_OBJECT_TRACKER_H_
#define INC_OBJECT_TRACKER_H_

#include <stdint.h>
#include "thermal.h"

#define OBJ_ID_NONE            0xFFU
#define OBJ_ASSOC_MAX_DIST     2.6f
#define OBJ_TRACK_STALE_MS     1500U
#define SEEK_OTHER_MIN_MANHATTAN 1.4f
#define SELECT_FALLBACK_LAG_MS 1200U
#define SEEK_PRE_SCAN_WAIT_MS  2200U
#define SEEK_ACCEPT_VISIBLE_MS 600U

void    ObjTracker_Init(void);
void    ObjTracker_Associate(const ThermalObjectsResult *objs, uint32_t now_ms);
const uint8_t *ObjTracker_GetObjIds(void);
void    ObjTracker_RequestNext(void);
void    ObjTracker_Update(const ThermalObjectsResult *objs, uint32_t now_ms);
uint8_t ObjTracker_GetSelected(void);
void    ObjTracker_BuildDetection(ThermalDetection *det, const ThermalObjectsResult *objs, uint32_t now_ms);

#endif /* INC_OBJECT_TRACKER_H_ */
