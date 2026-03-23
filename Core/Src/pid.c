/*
 * pid.c — Generic PID controller (one instance per axis)
 *
 * Converts pixel-space error (centroid offset from frame center) into a servo
 * angle delta. Supports integral anti-windup and an external velocity estimate
 * for the derivative term once the Kalman filter is active.
 *
 * Owner:
 */

#include "pid.h"
