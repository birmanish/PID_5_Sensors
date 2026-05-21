#include <PID_5_Sensors.h>

/*
 * Basic Line Follower Example
 * * This sketch demonstrates how to integrate the PID_5_Sensors library 
 * into a generic Arduino project. The sensor readings and motor commands 
 * must be adapted according to your specific hardware setup.
 */

PID_5_Sensors pid;

// Define your analog pins (Adjust according to your microcontroller)
const int sensorPins[5] = {A0, A1, A2, A3, A4};

void setup()
{
    Serial.begin(115200);

    // 1. Configure PID constraints
    pid.Kp        = 30.0f;
    pid.Kd        = 10.0f;
    pid.baseSpeed = 55;
    pid.maxSpeed  = 110;

    // 2. Set track mode (false = black line on white background)
    pid.setWhiteLine(false);

    // 3. Set thresholds for each sensor 
    // Format: pid.setThreshold(index, midpoint_threshold, ground_value)
    // Note: It is highly recommended to implement an auto-calibration routine 
    // to populate these values dynamically.
    pid.setThreshold(0, 2.1f, 2.8f);
    pid.setThreshold(1, 2.0f, 2.7f);
    pid.setThreshold(2, 2.2f, 2.9f);
    pid.setThreshold(3, 2.1f, 2.8f);
    pid.setThreshold(4, 2.0f, 2.7f);

    pid.resetPID();
    Serial.println("System Ready.");
}

void loop()
{
    // 4. Read analog voltages from the 5 sensors
    float raw[NB_SENSORS];
    for (int i = 0; i < NB_SENSORS; i++) {
        // Convert standard 10-bit analogRead (0-1023) to Voltage (0-5V or 0-3.3V)
        // Adjust the multiplier based on your microcontroller's operating voltage
        raw[i] = (analogRead(sensorPins[i]) / 1023.0f) * 5.0f; 
    }

    // 5. Provide current robot speeds (if encoders are available)
    // Pass 0.0f if dynamic Y-axis kinematic correction is not required.
    float linearVelocity  = 0.0f; // cm/s
    float angularVelocity = 0.0f; // rad/s

    // 6. Check for track safety conditions
    if (pid.isLost(raw)) {
        // Line lost -> Stop motors
        // setMotorLeft(0); setMotorRight(0);
        return;
    }

    if (pid.isCrossing(raw)) {
        // Intersection detected -> Implement intersection logic here
    }

    // 7. Compute time elapsed (dt) in seconds
    static unsigned long tPrev = micros();
    unsigned long tNow = micros();
    float dt = (tNow - tPrev) / 1e6f;
    tPrev = tNow;

    // 8. Compute centroid and subsequent PID motor speeds
    float position = pid.centroid(raw, linearVelocity, angularVelocity);
    
    int leftSpeed  = pid.computeLeft(position, dt);
    int rightSpeed = pid.computeRight(position, dt);

    // 9. Apply speeds to your motor driver
    // setMotorLeft(leftSpeed);
    // setMotorRight(rightSpeed);

    // Debugging output
    Serial.printf("Pos: %.2f | Motor L: %d | Motor R: %d\n", position, leftSpeed, rightSpeed);
    
    delay(10); // Small delay for stability
}
