#ifndef PID_5_SENSORS_H
#define PID_5_SENSORS_H

#include <Arduino.h>
#include <math.h>

#define NB_SENSORS 5

class PID_5_Sensors {
public:

    // ─── Positions physiques des capteurs (mm depuis centre)
    //     X : position latérale  |  Y : recul par rapport au capteur central
    //     Ordre : [0]=droit extrême ... [4]=gauche extrême
    float sensorX[NB_SENSORS] = { -28.0f, -14.0f,  0.0f, 14.0f, 28.0f };
    float sensorY[NB_SENSORS] = {  10.0f,   5.0f,  0.0f,  5.0f, 10.0f };

    // ─── Seuils calibrés (à remplir depuis le code principal)
    float threshold[NB_SENSORS];
    float thresholdGround[NB_SENSORS];
    bool  whiteLine = false;

    // ─── PID
    float Kp         = 30.0f;
    float Ki         =  0.0f;
    float Kd         = 10.0f;
    float KpUrgency  = 60.0f;

    // ─── Vitesses
    int   baseSpeed  = 55;
    int   maxSpeed   = 110;

    // ─── Constructeur
    PID_5_Sensors();

    // ─── Calibration
    void  setThreshold(int index, float thresh, float ground);
    void  setWhiteLine(bool white);

    // ─── Lecture capteurs
    //     rawValues[] : tableau de NB_SENSORS tensions lues depuis le code principal
    bool  onLine(int index, float rawValue);
    float centroid(float rawValues[NB_SENSORS], float vLin, float omega);

    // ─── PID
    void  resetPID();
    int   computeLeft(float position, float dt);
    int   computeRight(float position, float dt);

    // ─── Détection
    bool  isUrgencyLeft(float rawValues[NB_SENSORS]);
    bool  isUrgencyRight(float rawValues[NB_SENSORS]);
    bool  isLost(float rawValues[NB_SENSORS]);
    bool  isCrossing(float rawValues[NB_SENSORS]);
    bool  isCenterDetected(float rawValues[NB_SENSORS]);

private:
    float _errPrev;
    float _integral;
    unsigned long _tPrev;

    float _correctedX(int index, float vLin, float omega);
};

#endif
