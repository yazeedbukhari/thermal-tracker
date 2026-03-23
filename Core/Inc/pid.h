/*
 * pid.h — Generic PID controller (one instance per axis)
 *
 * Converts a pixel-space error signal (centroid offset from frame center) into
 * a servo angle delta. Two independent instances are used for pan and tilt.
 * Supports integral anti-windup (clamp) and accepts an external velocity
 * estimate for the derivative term once the Kalman filter is active.
 *
 * Public API:  PID_Init, PID_Update, PID_Reset
 *
 * Owner:
 */

#ifndef INC_PID_H_
#define INC_PID_H_



#endif /* INC_PID_H_ */
