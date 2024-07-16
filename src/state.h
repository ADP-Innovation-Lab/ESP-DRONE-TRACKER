#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include <math.h>

// Threshold values
#define ACCELERATION_THRESHOLD 10.0
#define ALTITUDE_THRESHOLD 2.0
#define SPEED_THRESHOLD 5.0
#define DISTANCE_THRESHOLD 30.0

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float altitude;
    float speed;
    double latitude;
    double longitude;
} SensorData;

typedef struct {
    double previous_latitude;
    double previous_longitude;
    float previous_altitude;
} StateData;

void state_init(StateData *state_data, double initial_latitude, double initial_longitude, float initial_altitude);
float calculate_total_acceleration(float accel_x, float accel_y, float accel_z);
double haversine(double lat1, double lon1, double lat2, double lon2);
void state_update(StateData *state_data, const SensorData *sensor_data, char *state);

#endif // STATE_H
