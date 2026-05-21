#include "PID_5_Sensors.h"

PID_5_Sensors::PID_5_Sensors()
{
    // Default geometry (e.g. 14mm pitch curved array)
    // Can be overridden via setSensorGeometry()
    const float defaultX[NB_SENSORS] = { -28.0f, -14.0f,  0.0f, 14.0f, 28.0f };
    const float defaultY[NB_SENSORS] = {  10.0f,   5.0f,  0.0f,  5.0f, 10.0f };
    setSensorGeometry(defaultX, defaultY);

    for (int i = 0; i < NB_SENSORS; i++) {
        threshold[i]       = 1.65f;
        thresholdGround[i] = 3.3f;
    }
    resetPID();
}

// ---------------------------------------------------------
//  CONFIGURATION & CALIBRATION
// ---------------------------------------------------------

void PID_5_Sensors::setSensorGeometry(const float xOffsets[NB_SENSORS], const float yOffsets[NB_SENSORS])
{
    for (int i = 0; i < NB_SENSORS; i++) {
        sensorX[i] = xOffsets[i];
        sensorY[i] = yOffsets[i];
    }
}

void PID_5_Sensors::setThreshold(int index, float thresh, float ground)
{
    if (index < 0 || index >= NB_SENSORS) return;
    threshold[index]       = thresh;
    thresholdGround[index] = ground;
}

void PID_5_Sensors::setWhiteLine(bool white)
{
    whiteLine = white;
}

// ---------------------------------------------------------
//  SENSOR PROCESSING
// ---------------------------------------------------------

bool PID_5_Sensors::onLine(int index, float rawValue)
{
    if (index < 0 || index >= NB_SENSORS) return false;
    return whiteLine ? (rawValue >= threshold[index])
                     : (rawValue <= threshold[index]);
}

// Kinematic Y-axis correction:
// A sensor offset longitudinally by dy observes the line slightly ahead/behind.
// During the time dt = dy / vLin, the robot rotates by omega * dt.
// This function compensates for the resulting lateral shift.
float PID_5_Sensors::_correctedX(int index, float vLin, float omega)
{
    float x  = sensorX[index];
    float dy = sensorY[index] / 10.0f; // mm to cm
    
    // Avoid division by zero at very low speeds
    if (abs(vLin) > 2.0f) {
        x += sensorX[index] * omega * dy / vLin;
    }
    return x;
}

// Computes the weighted centroid position (range approximately -2.0 to +2.0)
// Returns NAN if the line is completely lost.
// rawValues[] : analog voltages of the NB_SENSORS
// vLin (cm/s) and omega (rad/s) from encoders (optional, pass 0 if unknown)
float PID_5_Sensors::centroid(float rawValues[NB_SENSORS], float vLin, float omega)
{
    float sum = 0.0f, weights = 0.0f;
    int   n   = 0;

    for (int i = 0; i < NB_SENSORS; i++) {
        if (!onLine(i, rawValues[i])) continue;
        
        float w = abs(rawValues[i] - thresholdGround[i]);
        float x = _correctedX(i, vLin, omega);
        
        // Normalize by typical sensor spacing (14.0f)
        sum     += (x / 14.0f) * w;
        weights += w;
        n++;
    }
    return (n > 0) ? sum / weights : NAN;
}

// ---------------------------------------------------------
//  PID COMPUTATION
// ---------------------------------------------------------

void PID_5_Sensors::resetPID()
{
    _errPrev  = 0.0f;
    _integral = 0.0f;
    _tPrev    = micros();
}

// Returns the left motor speed (ready to be applied to PWM)
int PID_5_Sensors::computeLeft(float position, float dt)
{
    // Apply urgency gain if deviation is critical
    float kp      = (abs(position) >= 1.8f) ? KpUrgency : Kp;
    
    _integral    += position * dt;
    _integral     = constrain(_integral, -50.0f, 50.0f);
    
    float deriv   = (dt > 0) ? (position - _errPrev) / dt : 0.0f;
    _errPrev      = position;
    
    float corr    = kp * position + Ki * _integral + Kd * deriv;
    return constrain((int)(baseSpeed + corr), -maxSpeed, maxSpeed);
}

// Returns the right motor speed (ready to be applied to PWM)
int PID_5_Sensors::computeRight(float position, float dt)
{
    float kp      = (abs(position) >= 1.8f) ? KpUrgency : Kp;
    float corr    = kp * position + Ki * _integral + Kd * (position - _errPrev) / max(dt, 0.0001f);
    return constrain((int)(baseSpeed - corr), -maxSpeed, maxSpeed);
}

// ---------------------------------------------------------
//  LINE DETECTION HELPERS
// ---------------------------------------------------------

bool PID_5_Sensors::isUrgencyRight(float rawValues[NB_SENSORS])
{
    return onLine(0, rawValues[0]);
}

bool PID_5_Sensors::isUrgencyLeft(float rawValues[NB_SENSORS])
{
    return onLine(4, rawValues[4]);
}

bool PID_5_Sensors::isLost(float rawValues[NB_SENSORS])
{
    for (int i = 0; i < NB_SENSORS; i++)
        if (onLine(i, rawValues[i])) return false;
    return true;
}

bool PID_5_Sensors::isCrossing(float rawValues[NB_SENSORS])
{
    int n = 0;
    for (int i = 0; i < NB_SENSORS; i++)
        if (onLine(i, rawValues[i])) n++;
    return (n >= 4); // Intersection detected if 4 or 5 sensors see the line
}

bool PID_5_Sensors::isCenterDetected(float rawValues[NB_SENSORS])
{
    return onLine(2, rawValues[2]);
}
