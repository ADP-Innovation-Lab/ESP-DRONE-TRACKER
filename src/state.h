#ifndef STATE_H
#define STATE_H

#include <Arduino.h>

// Define thresholds
#define ALTITUDE_THRESHOLD 2
#define VELOCITY_THRESHOLD 2
#define ACCELERATION_THRESHOLD 1.0

// Define states
enum DroneState
{
    LANDED,
    TAKING_OFF,
    FLYING,
    LANDING,
    MOVING,
    STOPPED
};
// Define Device event
enum DroneEvent
{
    DataUpdate,
    Tampering,
    LowBattery,
    ChargerConnected,
    ChargerDisconnected
};

extern volatile DroneState currentState;
extern volatile DroneEvent currentEvent;

// Function declarations
void readSensorData(float &altitude, float &velocity, float &accelX, float &accelY, float &accelZ, float &roll, float &pitch, float &yaw);
void determineState(float altitude, float velocity, float accelZ);
void stateTask(void *pvParameters);

void set_droneState(DroneState state);
DroneState get_droneState();
const char *get_droneState_str();
void set_droneEvent(DroneEvent event);
DroneEvent get_droneEvent();
const char *get_droneEvent_str();


#endif // STATE_H