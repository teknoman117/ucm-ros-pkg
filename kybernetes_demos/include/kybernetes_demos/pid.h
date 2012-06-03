// Header File
#ifndef __PID_H
#define __PID_H

#include <ros/ros.h>
//-----------------------------------------------------------------------------

typedef struct sPID
{
float gain; // Controller gain
float kp; // Coefficient for proportional term
float ki; // Coefficient for integral term
float kd; // Coefficient for derivative term
float sp; // Setpoint
float i; // Integral term
float i_max; // Integral part limit
float pv; // Process value
float pv_last; // Last process value
float cv; // Control value
float cv_min; // Minimum control value
float cv_max; // Control value limit
} tPID;

typedef tPID *pPID;
//-----------------------------------------------------------------------------

void pid_init(pPID pid);
void pid_update(pPID pid, float dt);
void pid_dump(pPID pid);
//-----------------------------------------------------------------------------

#endif