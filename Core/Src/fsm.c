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
