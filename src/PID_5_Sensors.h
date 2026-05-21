#ifndef PID_5_SENSORS_H
#define PID_5_SENSORS_H

#include <Arduino.h>
#include <math.h>

#define NB_SENSORS 5

class PID_5_Sensors {
public:
    // --- Physical sensor positions (mm from the array's center)
    // X: lateral position | Y: longitudinal offset relative to the central sensor
    // Order: [0]=Far Right ... [4]=Far Left
    float sensorX[NB_SENSORS];
    float sensorY[NB_SENSORS];

    // --- Calibrated thresholds
    float threshold[NB_SENSORS];
    float thresholdGround[NB_SENSORS];
    bool  whiteLine = false;

    // --- PID Constants
    float Kp         = 30.0f;
    float Ki         =  0.0f;
    float Kd         = 10.0f;
    float KpUrgency  = 60.0f;

    // --- Speed Constraints
    int   baseSpeed  = 55;
    int   maxSpeed   = 110;

    // --- Constructor
    PID_5_Sensors();

    // --- Configuration & Calibration
    void  setSensorGeometry(const float xOffsets[NB_SENSORS], const float yOffsets[NB_SENSORS]);
    void  setThreshold(int index, float thresh, float ground);
    void  setWhiteLine(bool white);

    // --- Sensor Processing
    // rawValues[] : array of NB_SENSORS analog readings provided by the main sketch
    bool  onLine(int index, float rawValue);
    float centroid(float rawValues[NB_SENSORS], float vLin, float omega);

    // --- PID Computation
    void  resetPID();
    int   computeLeft(float position, float dt);
    int   computeRight(float position, float dt);

    // --- Detection Helpers
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
