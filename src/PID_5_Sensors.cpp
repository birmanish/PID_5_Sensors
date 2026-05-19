#include "PID_5_Sensors.h"

PID_5_Sensors::PID_5_Sensors()
{
    for (int i = 0; i < NB_SENSORS; i++) {
        threshold[i]       = 1.65f;
        thresholdGround[i] = 3.3f;
    }
    resetPID();
}

// ─────────────────────────────────────────────────────────
//  CALIBRATION
// ─────────────────────────────────────────────────────────

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

// ─────────────────────────────────────────────────────────
//  LECTURE CAPTEURS
// ─────────────────────────────────────────────────────────

bool PID_5_Sensors::onLine(int index, float rawValue)
{
    if (index < 0 || index >= NB_SENSORS) return false;
    return whiteLine ? (rawValue >= threshold[index])
                     : (rawValue <= threshold[index]);
}

// Correction Y : un capteur en retrait de dy voit ce que
// le capteur central verra dans dt = dy / vLin.
// Pendant dt, le robot pivote de omega*dt → décalage X corrigé.
float PID_5_Sensors::_correctedX(int index, float vLin, float omega)
{
    float x  = sensorX[index];
    float dy = sensorY[index] / 10.0f; // mm → cm
    if (abs(vLin) > 2.0f)
        x += sensorX[index] * omega * dy / vLin;
    return x;
}

// Retourne la position centroïde (-2 à +2), NAN si ligne perdue.
// rawValues[] : tensions des NB_SENSORS capteurs avant
// vLin (cm/s) et omega (rad/s) depuis les encodeurs
float PID_5_Sensors::centroid(float rawValues[NB_SENSORS], float vLin, float omega)
{
    float sum = 0.0f, weights = 0.0f;
    int   n   = 0;

    for (int i = 0; i < NB_SENSORS; i++) {
        if (!onLine(i, rawValues[i])) continue;
        float w = abs(rawValues[i] - thresholdGround[i]);
        float x = _correctedX(i, vLin, omega);
        sum     += (x / 14.0f) * w;
        weights += w;
        n++;
    }
    return (n > 0) ? sum / weights : NAN;
}

// ─────────────────────────────────────────────────────────
//  PID
// ─────────────────────────────────────────────────────────

void PID_5_Sensors::resetPID()
{
    _errPrev  = 0.0f;
    _integral = 0.0f;
    _tPrev    = micros();
}

// Retourne la vitesse moteur gauche (appliquer directement)
int PID_5_Sensors::computeLeft(float position, float dt)
{
    float kp      = (abs(position) >= 1.8f) ? KpUrgency : Kp;
    _integral    += position * dt;
    _integral     = constrain(_integral, -50.0f, 50.0f);
    float deriv   = (dt > 0) ? (position - _errPrev) / dt : 0.0f;
    _errPrev      = position;
    float corr    = kp * position + Ki * _integral + Kd * deriv;
    return constrain((int)(baseSpeed + corr), -maxSpeed, maxSpeed);
}

// Retourne la vitesse moteur droit (appliquer directement)
int PID_5_Sensors::computeRight(float position, float dt)
{
    float kp      = (abs(position) >= 1.8f) ? KpUrgency : Kp;
    float corr    = kp * position + Ki * _integral + Kd * (position - _errPrev) / max(dt, 0.0001f);
    return constrain((int)(baseSpeed - corr), -maxSpeed, maxSpeed);
}

// ─────────────────────────────────────────────────────────
//  DÉTECTION
// ─────────────────────────────────────────────────────────

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
    return (n >= 4);
}

bool PID_5_Sensors::isCenterDetected(float rawValues[NB_SENSORS])
{
    return onLine(2, rawValues[2]);
}
