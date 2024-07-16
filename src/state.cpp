#include "state.h"
#include <math.h>
#include <stdio.h>
#include "sensors.h"

void state_init(StateData *state_data, double initial_latitude, double initial_longitude, float initial_altitude) {
    state_data->previous_latitude = initial_latitude;
    state_data->previous_longitude = initial_longitude;
    state_data->previous_altitude = initial_altitude;
}

float calculate_total_acceleration(float accel_x, float accel_y, float accel_z) {
    return sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000; // radius of Earth in meters
    double delta_lat = (lat2 - lat1) * M_PI / 180;
    double delta_lon = (lon2 - lon1) * M_PI / 180;
    double a = sin(delta_lat / 2) * sin(delta_lat / 2) +
               cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
               sin(delta_lon / 2) * sin(delta_lon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}



void state_update(StateData *state_data, const SensorData *sensor_data, char *state) {
    float total_acceleration = calculate_total_acceleration(sensor_data->accel_x, sensor_data->accel_y, sensor_data->accel_z);
    double distance_change = haversine(state_data->previous_latitude, state_data->previous_longitude, sensor_data->latitude, sensor_data->longitude);
    float altitude_change = fabs(sensor_data->altitude - state_data->previous_altitude);

    if (total_acceleration > ACCELERATION_THRESHOLD ||
        altitude_change > ALTITUDE_THRESHOLD ||
        sensor_data->speed > SPEED_THRESHOLD ||
        distance_change > DISTANCE_THRESHOLD) {
        *state = 'F'; // Flying
    } else {
        *state = 'S'; // Stopped
    }

    // Update previous values
    state_data->previous_latitude = sensor_data->latitude;
    state_data->previous_longitude = sensor_data->longitude;
    state_data->previous_altitude = sensor_data->altitude;
}

