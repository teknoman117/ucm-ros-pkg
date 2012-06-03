// C-File
// Module..: PID.C
// Version.: 1.0
// Compiler: IAR ARM
// Date....: August 2007
// Chip....: STM32
//-----------------------------------------------------------------------------

#include "kybernetes_demos/pid.h"
//-----------------------------------------------------------------------------

void pid_init(pPID pid)
{
// Preset coefficients and variables
pid->gain = 1.0;
pid->kp = 2.0;
pid->ki = 1.0;
pid->kd = 0.5;
pid->cv_min = 5.0; // Minimum Control Value
pid->cv_max = 255.0; // Maximum Control Value

pid->sp = 0.0; // Setpoint
pid->pv = 0.0; // Process Value
pid->pv_last = 0.0;
pid->i = 0.0;
pid->i_max = 10.0;
pid->cv = 0.0;
}
//-----------------------------------------------------------------------------

void pid_update(pPID pid, float dt)
{
float e, d;

// Error = setpoint - process value
e = pid->sp - pid->pv;

// Calculate integral term
pid->i += e * dt;

// Check integral term for upper limit
if (pid->i > pid->i_max)
{
pid->i = pid->i_max;
}

// Check integral term for lower limit
if (pid->i < -pid->i_max)
{
pid->i = -pid->i_max;
}

// Derivative part = (process value - last process value) / dt
if (dt > 0.00001) // Avoid division by zero
{
d = (pid->pv - pid->pv_last) / dt;
}
else
{
d = 0.0;
}

// Control value = gain * (kp * e + ki * i - kd * d)
pid->cv = pid->gain * (pid->kp * e + pid->ki * pid->i - pid->kd * d);

// Add min to control value
if (pid->cv >= 0.0 && pid->cv < pid->cv_min)
{
pid->cv += pid->cv_min;
}
else if (pid->cv < 0.0 && pid->cv > -pid->cv_min)
{
pid->cv -= pid->cv_min;
}

// Check control value upper limits
if (pid->cv > pid->cv_max)
{
pid->cv = pid->cv_max;
}

if (pid->cv < -pid->cv_max)
{
pid->cv = -pid->cv_max;
}

// Store reference value for next control cycle
pid->pv_last = pid->pv;
}

/*float gain; // Controller gain
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
float cv_max; // Control value limit*/

void pid_dump(pPID pid)
{
    ROS_INFO("Constants: Kd = %f, Ki = %f, Kd = %f", pid->kp, pid->ki, pid->kd);
    ROS_INFO("Control Value: %f < %f < %f, ", pid->cv_min, pid->cv, pid->cv_max);
    ROS_INFO("Setpoint = %f, I = %f, Im = %f", pid->sp, pid->i, pid->i_max);
}