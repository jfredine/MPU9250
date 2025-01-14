//
// Example use of the MPU9250 orientation sensor.
//
// Copyright 2025  John Fredine
//

#include <Arduino.h>
#include "MPU9250.h"

#ifdef TARGET_RASPBERRY_PI_PICO
auto &serial = Serial1;
#else
auto &serial = Serial;
#endif

MPU9250 mpu9250;
uint32_t timestamp;

void setup() {
    serial.begin(115200);
    while (!serial) {
      delay(10);
    }
    delay(1000);

    Wire.begin();
    Wire.setClock(400000);

    int retval = mpu9250.init();
    if (retval != 0) {
        if (retval == 1) {
            serial.println("Failed to locate MPU9250 I2C address");
            while (1) delay(10);
        } else if (retval == 2) {
            serial.println("Failed to identify MPU9250");
            while (1) delay(10);
        } else if (retval == 3) {
            serial.println("Failed to identify AK8963");
            while (1) delay(10);
        } else {
            serial.println("Unknown initialization error");
            while (1) delay(10);
        }
    }

    timestamp = millis();
}

#if defined(CALIBRATION)

// This is for doing calibration of the sensor using motion sensor calibration
// tool at https://github.com/PaulStoffregen/MotionCal.  See the write up
// from Adafruit at https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-motioncal
// or PJRC at https://www.pjrc.com/store/prop_shield.html.  Attempting AHRS
// without proper magnetic compensation will yield poor results.

void loop() {
    MPU9250::tuple<float> accel, gyro, mag;
    float temperature;
    mpu9250.read(&accel, &gyro, &mag, &temperature);

    serial.print("Raw:");
    serial.print(static_cast<int>(accel.x * 8192 / 9.8));
    serial.print(",");
    serial.print(static_cast<int>(accel.y * 8192 / 9.8));
    serial.print(",");
    serial.print(static_cast<int>(accel.z * 8192 / 9.8));
    serial.print(",");
    serial.print(static_cast<int>(gyro.x * MPU9250_RPS_TO_DPS * 16));
    serial.print(",");
    serial.print(static_cast<int>(gyro.y * MPU9250_RPS_TO_DPS * 16));
    serial.print(",");
    serial.print(static_cast<int>(gyro.z * MPU9250_RPS_TO_DPS * 16));
    serial.print(",");
    serial.print(static_cast<int>(mag.x * 10));
    serial.print(",");
    serial.print(static_cast<int>(mag.y * 10));
    serial.print(",");
    serial.print(static_cast<int>(mag.z * 10));
    serial.println("");

    delay(10);
}

#else

// Full orientation sensing using Madgwick/Mahony
//
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface


#ifdef MAHONY
#include <Mahony.h>
Mahony filter;    // faster but less accurate
#else
#include <Madgwick.h>
Madgwick filter;  // slower but more accurate
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

typedef struct {
    MPU9250::tuple<float> mag_hard_iron;  // MotionCal Magnetic Offset
    struct {
        MPU9250::tuple<float> x;
        MPU9250::tuple<float> y;
        MPU9250::tuple<float> z;
    } mag_soft_iron;                     // MotionCal Magnetic Mapping
    MPU9250::tuple<float> accel;
    MPU9250::tuple<float> gyro;
} sensor_adj_t;

// plug in the values found in MotionCal here.
#ifdef TARGET_RASPBERRY_PI_PICO
sensor_adj_t sensor_adj = {{13.60, 22.44, 29.51},
                           {{ 0.905, -0.015,  0.013},
                            {-0.015,  1.015,  0.008},
                            { 0.013,  0.008,  1.089}},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0}};
#else
sensor_adj_t sensor_adj = {{ 19.99,  18.61,  30.22},
                           {{ 0.935, -0.006,  0.028},
                            {-0.006,  0.963,  0.032},
                            { 0.028,  0.032,  1.113}},
                           {0.0, 0.0, 0.0},
                           {0.0, 0.0, 0.0}};
#endif


void loop() {
    float roll, pitch, heading;
    static uint8_t counter = 0;

    if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) {
        return;
    }
    timestamp = millis();

    // Read the motion sensors
    MPU9250::tuple<float> accel, gyro, mag;
    float temperature;
    mpu9250.read(&accel, &gyro, &mag, &temperature);

    float mag_x = mag.x - sensor_adj.mag_hard_iron.x;
    float mag_y = mag.y - sensor_adj.mag_hard_iron.y;
    float mag_z = mag.z - sensor_adj.mag_hard_iron.z;

    mag.x = mag_x * sensor_adj.mag_soft_iron.x.x
            + mag_y * sensor_adj.mag_soft_iron.x.y
            + mag_z * sensor_adj.mag_soft_iron.x.z;

    mag.y = mag_x * sensor_adj.mag_soft_iron.y.x
            + mag_y * sensor_adj.mag_soft_iron.y.y
            + mag_z * sensor_adj.mag_soft_iron.y.z;

    mag.z = mag_x * sensor_adj.mag_soft_iron.z.x
            + mag_y * sensor_adj.mag_soft_iron.z.y
            + mag_z * sensor_adj.mag_soft_iron.z.z;

    gyro.x -= sensor_adj.gyro.x;
    gyro.y -= sensor_adj.gyro.y;
    gyro.z -= sensor_adj.gyro.z;

    accel.x -= sensor_adj.accel.x;
    accel.y -= sensor_adj.accel.y;
    accel.z -= sensor_adj.accel.z;

    // Update the AHRS fusion filter
    filter.update(gyro.x, gyro.y, gyro.z,
                  accel.x, accel.y, accel.z,
                  mag.x, mag.y, mag.z,
                  1.0 / FILTER_UPDATE_RATE_HZ);

    // print the output less often to avoid bogging down the serial interface
    if (counter++ <= PRINT_EVERY_N_UPDATES) {
        return;
    }

    // print the orientation in euler angles and quaternions
    counter = 0;
    roll = filter.get_roll();
    pitch = filter.get_pitch();
    heading = filter.get_yaw();
    serial.print("Orientation: ");
    serial.print(heading);
    serial.print(", ");
    serial.print(pitch);
    serial.print(", ");
    serial.println(roll);

    float qw, qx, qy, qz;
    filter.get_quaternion(&qw, &qx, &qy, &qz);
    serial.print("Quaternion: ");
    serial.print(qw, 4);
    serial.print(", ");
    serial.print(qx, 4);
    serial.print(", ");
    serial.print(qy, 4);
    serial.print(", ");
    serial.println(qz, 4);
}
#endif
