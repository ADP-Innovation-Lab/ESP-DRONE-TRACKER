#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

// Define thresholds
#define ALTITUDE_THRESHOLD 1.0
#define VELOCITY_THRESHOLD 0.5
#define ACCELERATION_THRESHOLD 1.0

// Define states
enum DroneState {
    LANDED,
    TAKING_OFF,
    FLYING,
    LANDING,
    MOVING,
    STOPPED
};

// Function declarations
void readSensorData(float &altitude, float &velocity, float &accelX, float &accelY, float &accelZ, float &roll, float &pitch, float &yaw);
void determineState(float altitude, float velocity, float accelZ);
void stateTask(void *pvParameters);

extern volatile DroneState currentState;

#endif // STATE_H