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



#endif /* INC_FSM_H_ */
