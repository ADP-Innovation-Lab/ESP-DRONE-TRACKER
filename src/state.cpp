#include "state.h"
#include "myGPS.h"
#include "sensors.h"
#include "myLTE.h"

// Global variable for current state
volatile DroneState currentState = LANDED;

// Function to read sensor data
void readSensorData(float &altitude, float &velocity, float &accelX, float &accelY, float &accelZ, float &roll, float &pitch, float &yaw) {
    // Get altitude from BMP280
    altitude = bmp_getRelativeAltitude();

    // Get velocity from GPS
    velocity = 0.0; //ubxM6.getSpeed();

    // Get IMU data
    accelX = imu_getAccelX();
    accelY = imu_getAccelY();
    accelZ = imu_getAccelZ();
    roll = imu_getRoll();
    pitch = imu_getPitch();
    yaw = imu_getYaw();
}

// Function to determine state
void determineState(float altitude, float velocity, float accelZ) {
    switch (currentState) {
        case LANDED:
            if (accelZ > ACCELERATION_THRESHOLD) {
                currentState = TAKING_OFF;
            }
            break;
        case TAKING_OFF:
            if (altitude > ALTITUDE_THRESHOLD) {
                currentState = FLYING;
            }
            break;
        case FLYING:
            if (accelZ < -ACCELERATION_THRESHOLD) {
                currentState = LANDING;
            }
            break;
        case LANDING:
            if (altitude < ALTITUDE_THRESHOLD && velocity < VELOCITY_THRESHOLD) {
                currentState = LANDED;
            }
            break;
        case MOVING:
            if (velocity < VELOCITY_THRESHOLD) {
                currentState = STOPPED;
            }
            break;
        case STOPPED:
            if (velocity > VELOCITY_THRESHOLD) {
                currentState = MOVING;
            }
            break;
    }
}

// RTOS Task for State Detection
void stateTask(void *pvParameters) {
    float altitude = 0;
    float velocity = 0;
    float accelX = 0;
    float accelY = 0;
    float accelZ = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    while (1) {
        // Read sensor data
        readSensorData(altitude, velocity, accelX, accelY, accelZ, roll, pitch, yaw);

        // Determine current state
        determineState(altitude, velocity, accelZ);

        // Log the current state
        switch (currentState) {
            case LANDED:
                Serial.println("State: LANDED");
                break;
            case TAKING_OFF:
                Serial.println("State: TAKING_OFF");
                break;
            case FLYING:
                Serial.println("State: FLYING");
                break;
            case LANDING:
                Serial.println("State: LANDING");
                break;
            case MOVING:
                Serial.println("State: MOVING");
                break;
            case STOPPED:
                Serial.println("State: STOPPED");
                break;
        }

        // Delay for a while
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
