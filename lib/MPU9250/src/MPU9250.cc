//
// MPU9250.cc
//
// Implementation of the MPU9250 class used for basic control of the MPU9250
// motion sensor.
//

#include "MPU9250.h"

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

MPU9250::MPU9250(void) {}

MPU9250::~MPU9250(void) {}

//
// MPU9250::init
//
// Arguments: mpu9250_i2c_addr -- Address of MPU9250 on the I2C bus
//            ak8963_i2c_addr -- Address of AK8963 on the I2C bus
//            wire -- pointer to TwoWire object used to access the I2C bus
//
// Returns: success -- 0
//          failure -- 2 - MPU9250 was not found on the I2C bus
//                     3 - AK8963  was not found on the I2C bus
//
// Initialize the MPU9250 for operation.  Must be called before anything
// useful can happen.  The main routine should initialize the I2C bus before
// calling this routine.  All arguments have reasonable defaults so are
// unlikely to be needed.
//

int MPU9250::init(uint8_t mpu9250_i2c_addr, uint8_t ak8963_i2c_addr,
                  TwoWire *wire) {
    delay(1000);

    mpu9250_i2c_addr_ = mpu9250_i2c_addr;
    wire_ = wire;

    if (mpu9250_i2c_read(MPU9250_WHO_AM_I) != MPU9250_DEVICE_ID) {
        return 2;
    }

    // Make the magnetometer visible on I2C bus by setting bypass mode
    set_I2C_bypass(true);

    ak8963_i2c_addr_ = ak8963_i2c_addr;
    if (ak8963_i2c_read(AK8963_WHO_AM_I) != AK8963_DEVICE_ID) {
        return 3;
    }

    reset();

    set_gyro_filter_bandwidth(MPU9250_GYRO_BAND_250_HZ);
    set_accel_filter_bandwidth(MPU9250_ACCEL_BAND_460_HZ);

    set_gyro_range(MPU9250_RANGE_500_DEG);

    set_accel_range(MPU9250_RANGE_2_G);

    mpu9250_i2c_write(MPU9250_PWR_MGMT_1,
                      0x01);  // set clock config to PLL with Gyro X reference

    // enter fuse access mode of mag which is required to read adjustment data
    set_mag_mode(AK8963_MODE_FUSE_ROM);

    // read three bytes of sensitivity adjustment data (one byte for each axis)
    uint8_t raw[3];
    ak8963_i2c_read(AK8963_ASAX, raw, 3);

    // store sensitivity adjustment per axis (see data sheet for calculation)
    asa_.x = (static_cast<float>(raw[0]) - 128) / 256 + 1;
    asa_.y = (static_cast<float>(raw[1]) - 128) / 256 + 1;
    asa_.z = (static_cast<float>(raw[2]) - 128) / 256 + 1;

    // Finish setting up magnetometer
    set_mag_mode(AK8963_MODE_100HZ);
    set_mag_sensitivity(AK8963_SENSITIVITY_16);
    delay(100);

    return 0;
}

//
// MPU9250::reset
//
// Arguments: None
//
// Returns: Nothing
//
// Reset the MPU9250.  Documentation is unclear about how this affects the
// AK8963 but presumably it is also reset.  A standalone AK8963 has a register
// for resetting it, but this register is marked "reserved" in the MPU9250.
//

void MPU9250::reset(void) {
    uint8_t power_mgmt_1 = mpu9250_i2c_read(MPU9250_PWR_MGMT_1);
    mpu9250_i2c_write(MPU9250_PWR_MGMT_1, power_mgmt_1 | 0x80);

    while (mpu9250_i2c_read(MPU9250_PWR_MGMT_1) &
           0x80) {  // Wait for reset bit to be cleared
        delay(1);
    }
    delay(100);

    mpu9250_i2c_write(MPU9250_SIGNAL_PATH_RESET, 0x7);
    delay(100);

    // Make the magnetometer visible on I2C bus by setting bypass mode
    set_I2C_bypass(true);

    delay(100);
}

//
//
// MPU9250::set_I2C_bypass
//
// Arguments: bypass -- true to enable bypass or false to disable
//
// Returns: Nothing
//
// Enable or disable the I2C bypass on the MPU9250.  When bypass is enabled
// the auxilliary I2C pins of the MPU9250 are electrically connected to the
// primary I2C pins and the AK8963 becomes visible from the primary I2C bus.
//

void MPU9250::set_I2C_bypass(bool bypass) {
    uint8_t u = mpu9250_i2c_read(MPU9250_INT_PIN_CFG);
    u &= ~(1 << 1);
    mpu9250_i2c_write(MPU9250_INT_PIN_CFG, u | (bypass << 1));

    u = mpu9250_i2c_read(MPU9250_USER_CTRL);
    u &= ~(1 << 5);
    mpu9250_i2c_write(MPU9250_USER_CTRL, u | (!bypass));
}

//
// MPU9250::get_clock
//
// Arguments: None
//
// Returns: The clock setting
//
// Get the currently programmed clock source as one of the enumerated values
// of type mpu9250_clock_select_t
//

mpu9250_clock_select_t MPU9250::get_clock(void) {
    uint8_t pwr_mgmt = mpu9250_i2c_read(MPU9250_PWR_MGMT_1);
    return (mpu9250_clock_select_t)(pwr_mgmt & 0x7);
}

//
// MPU9250::set_clock
//
// Arguments: new_clock -- New clock source
//
// Returns: Nothing
//
// Program the clock source for the sensors.  The new clock should be
// one of the enumerated values of type mpu9250_clock_select_t.
//

void MPU9250::set_clock(mpu9250_clock_select_t new_clock) {
    uint8_t pwr_mgmt = mpu9250_i2c_read(MPU9250_PWR_MGMT_1);
    pwr_mgmt &= ~0x7;
    pwr_mgmt |= new_clock & 0x7;
    mpu9250_i2c_write(MPU9250_PWR_MGMT_1, pwr_mgmt);
}

//
// MPU9250::get_accel_range
//
// Arguments: None
//
// Returns: The accelerometer range
//
// Get the currently programmed accelerometer range as one of the
// enumerated values of type mpu9250_accel_range_t
//

mpu9250_accel_range_t MPU9250::get_accel_range(void) {
    uint8_t accel_config = mpu9250_i2c_read(MPU9250_ACCEL_CONFIG);
    uint8_t accel_range = (accel_config >> 3) & 0x3;

    return (mpu9250_accel_range_t)accel_range;
}

//
// MPU9250::set_accel_range
//
// Arguments: new_range -- new range to which the MPU9250 should be programmed
//
// Returns: Nothing
//
// Program the MPU9250 to the user specified range.  The range should be
// of the enumberated values of type mpu9250_accel_range_t
//

void MPU9250::set_accel_range(mpu9250_accel_range_t new_range) {
    uint8_t accel_config = mpu9250_i2c_read(MPU9250_ACCEL_CONFIG);
    accel_config &= ~(0x3 << 3);
    accel_config |= (uint8_t)new_range << 3;

    mpu9250_i2c_write(MPU9250_ACCEL_CONFIG, accel_config);
}

//
// MPU9250::get_gyro_range
//
// Arguments: None
//
// Returns: The gyro range
//
// Get the currently programmed gyro range as one of the
// enumerated values of type mpu9250_gyro_range_t
//

mpu9250_gyro_range_t MPU9250::get_gyro_range(void) {
    uint8_t gyro_config = mpu9250_i2c_read(MPU9250_GYRO_CONFIG);
    uint8_t gyro_range = (gyro_config >> 3) & 0x3;

    return (mpu9250_gyro_range_t)gyro_range;
}

//
//
// MPU9250::set_gyro_range
//
// Arguments: new_range -- new range to which the MPU9250 should be programmed
//
// Returns: Nothing
//
// Program the MPU9250 to the user specified range.  The range should be
// of the enumberated values of type mpu9250_gyro_range_t
//

void MPU9250::set_gyro_range(mpu9250_gyro_range_t new_range) {
    uint8_t gyro_config = mpu9250_i2c_read(MPU9250_GYRO_CONFIG);
    gyro_config &= ~(0x3 << 3);
    gyro_config |= (uint8_t)new_range << 3;

    mpu9250_i2c_write(MPU9250_GYRO_CONFIG, gyro_config);
}

//
// MPU9250::get_accel_filter_bandwidth
//
// Arguments: None
//
// Returns: The accelerometer bandwidth setting
//
// Get the current accelerometer filter bandwidth as one of the enumerated
// values of type mpu9250_accel_bandwidth_t
//

mpu9250_accel_bandwidth_t MPU9250::get_accel_filter_bandwidth(void) {
    uint8_t u = mpu9250_i2c_read(MPU9250_ACCEL_CONFIG2);
    u &= 0xf;
    if (u & 0x8) {
        return MPU9250_ACCEL_BAND_1130_HZ;
    } else if (u == 0x7) {
        return MPU9250_ACCEL_BAND_460_HZ;
    } else {
        return (mpu9250_accel_bandwidth_t)u;
    }
}

//
// MPU9250::set_accel_filter_bandwidth
//
// Arguments: bandwidth - new bandwidth for the accelerometer
//
// Returns: Nothing
//
// Program the accelerometer filter bandwidth as one of the enumerated values
// of type mpu9250_accel_bandwidth_t
//

void MPU9250::set_accel_filter_bandwidth(mpu9250_accel_bandwidth_t bandwidth) {
    uint8_t u = mpu9250_i2c_read(MPU9250_ACCEL_CONFIG2);
    u &= 0xf;
    mpu9250_i2c_write(MPU9250_ACCEL_CONFIG2, u | bandwidth);
}

//
// MPU9250::get_gyro_filter_bandwidth
//
// Arguments: None
//
// Returns: The gyro bandwidth setting
//
// Get the current gyro filter bandwidth as one of the enumerated values
// of type mpu9250_gyro_bandwidth_t
//

mpu9250_gyro_bandwidth_t MPU9250::get_gyro_filter_bandwidth(void) {
    uint8_t u = mpu9250_i2c_read(MPU9250_CONFIG);
    u &= 0x7;

    uint8_t v = mpu9250_i2c_read(MPU9250_GYRO_CONFIG);
    v &= 0x3;

    if (v & 0x1) {
        return MPU9250_GYRO_BAND_8800_HZ;
    } else if (v == 0x2) {
        return MPU9250_GYRO_BAND_3600_HZ;
    } else {
        return (mpu9250_gyro_bandwidth_t)(u);
    }
}

//
// MPU9250::set_gyro_filter_bandwidth
//
// Arguments: bandwidth - new bandwidth for the gyro
//
// Returns: Nothing
//
// Program the gyro filter bandwidth as one of the enumerated values
// of type mpu9250_gyro_bandwidth_t
//

void MPU9250::set_gyro_filter_bandwidth(mpu9250_gyro_bandwidth_t bandwidth) {
    uint8_t u = mpu9250_i2c_read(MPU9250_GYRO_CONFIG);
    u &= ~0x3;
    mpu9250_i2c_write(MPU9250_GYRO_CONFIG, u | ((bandwidth >> 3) & 0x3));

    u = mpu9250_i2c_read(MPU9250_CONFIG);
    u &= ~0x7;
    mpu9250_i2c_write(MPU9250_CONFIG, u | (bandwidth & 0x7));
}

//
// MPU9250::get_mag_mode
//
// Arguments: None
//
// Returns: The mag mode
//
// Get the currently programmed mode of the AK8963 magnetometer as one
// of the enumerated values of type ak8963_mag_mode_t
//

ak8963_mag_mode_t MPU9250::get_mag_mode(void) {
    switch (mpu9250_i2c_read(AK8963_CNTL) & 0xf) {
        case 0:
            return AK8963_MODE_POWER_DOWN;
        case 1:
            return AK8963_MODE_SINGLE;
        case 2:
            return AK8963_MODE_8HZ;
        case 4:
            return AK8963_MODE_EXT_TRIG;
        case 6:
            return AK8963_MODE_100HZ;
        case 8:
            return AK8963_MODE_SELF_TEST;
        case 15:
            return AK8963_MODE_FUSE_ROM;
        default:
            return AK8963_MODE_POWER_DOWN;
    }
}

//
// MPU9250::set_mag_mode
//
// Arguments: new_mode -- New mode for the magnetometer
//
// Returns: Nothing
//
// Program the mode of the AK8963 magnetometer.  The mode should be one of the
// enumerated values of type ak8963_mag_mode_t.
//

void MPU9250::set_mag_mode(ak8963_mag_mode_t new_mode) {
    uint8_t mag_mode = ak8963_i2c_read(AK8963_CNTL);
    mag_mode &= ~0xf;
    ak8963_i2c_write(AK8963_CNTL, mag_mode | (uint8_t)AK8963_MODE_POWER_DOWN);
    delay(10);

    switch (new_mode) {
        case AK8963_MODE_SINGLE:
            mag_mode |= (uint8_t)AK8963_MODE_SINGLE;
            break;
        case AK8963_MODE_8HZ:
            mag_mode |= (uint8_t)AK8963_MODE_8HZ;
            break;
        case AK8963_MODE_EXT_TRIG:
            mag_mode |= (uint8_t)AK8963_MODE_EXT_TRIG;
            break;
        case AK8963_MODE_100HZ:
            mag_mode |= (uint8_t)AK8963_MODE_100HZ;
            break;
        case AK8963_MODE_SELF_TEST:
            mag_mode |= (uint8_t)AK8963_MODE_SELF_TEST;
            break;
        case AK8963_MODE_FUSE_ROM:
            mag_mode |= (uint8_t)AK8963_MODE_FUSE_ROM;
            break;
        default:
            mag_mode |= (uint8_t)AK8963_MODE_POWER_DOWN;
            break;
    }
    ak8963_i2c_write(AK8963_CNTL, mag_mode);
    delay(10);
}

//
// MPU9250::get_mag_sensitivity
//
// Arguments: None
//
// Returns: The mag sensitivity
//
// Get the currently programmed sensitivity of the AK8963 magnetometer as one
// of the enumerated values of type ak8963_mag_mode_t
//

ak8963_mag_sensitivity_t MPU9250::get_mag_sensitivity(void) {
    uint8_t cntl = ak8963_i2c_read(AK8963_CNTL);
    uint8_t mag_sensitivity = (cntl >> 4) & 0x1;
    return mag_sensitivity ? AK8963_SENSITIVITY_16 : AK8963_SENSITIVITY_14;
}

//
// MPU9250::set_mag_sensitivity
//
// Arguments: new_range -- New sensitivity for the magnetometer
//
// Returns: Nothing
//
// Program the sensitivity of the AK8963 magnetometer.  The sensitivity
// should be one of the enumerated values of type ak8963_mag_sensitivity_t.
//

void MPU9250::set_mag_sensitivity(ak8963_mag_sensitivity_t new_range) {
    uint8_t cntl = ak8963_i2c_read(AK8963_CNTL);
    cntl &= ~(1 << 4);
    ak8963_i2c_write(AK8963_CNTL, cntl | ((uint8_t)new_range << 4));
}

//
// MPU9250::read
//
// Arguments: accel -- User allocated structure to hold the latest values
//                     of X, Y, and Z from the accelerometer
//            gyro -- User allocated structure to hold the latest values
//                    of X, Y, and Z from the gyro
//            mag -- User allocated structure to hold the latest values
//                   of X, Y, and Z from the magnetometer
//            temperature -- pointer to location to hold the latest temperature
//                           reading
//
// Returns: success - zero
//          failure - non-zero error code
//
// Read the latest sensor data from the MPU9250 and write it to the user
// provided structures.  Any structure pointer which is NULL will be ignored.
//

int MPU9250::read(tuple<float> *accel, tuple<float> *gyro, tuple<float> *mag,
                  float *temperature) {
    // get raw readings of temp, accel, and gyro
    uint8_t buffer[14];
    mpu9250_i2c_read(MPU9250_ACCEL_XOUT_H, buffer, 14);

    int16_t rawAccX, rawAccY, rawAccZ;
    rawAccX = buffer[0] << 8 | buffer[1];
    rawAccY = buffer[2] << 8 | buffer[3];
    rawAccZ = buffer[4] << 8 | buffer[5];

    int16_t rawTemp;
    rawTemp = buffer[6] << 8 | buffer[7];

    int16_t rawGyroX, rawGyroY, rawGyroZ;
    rawGyroX = buffer[8] << 8 | buffer[9];
    rawGyroY = buffer[10] << 8 | buffer[11];
    rawGyroZ = buffer[12] << 8 | buffer[13];

    // adjust raw data for temp, accel, gyro
    temperature_ = (rawTemp / 340.0) + 36.53;

    mpu9250_accel_range_t accel_range = get_accel_range();

    float accel_scale = 1;
    if (accel_range == MPU9250_RANGE_16_G) {
        accel_scale = 2048;
    }
    if (accel_range == MPU9250_RANGE_8_G) {
        accel_scale = 4096;
    }
    if (accel_range == MPU9250_RANGE_4_G) {
        accel_scale = 8192;
    }
    if (accel_range == MPU9250_RANGE_2_G) {
        accel_scale = 16384;
    }

    accel_.x = static_cast<float>(rawAccX) / accel_scale * MPU9250_G_TO_MPS2;
    accel_.y = static_cast<float>(rawAccY) / accel_scale * MPU9250_G_TO_MPS2;
    accel_.z = static_cast<float>(rawAccZ) / accel_scale * MPU9250_G_TO_MPS2;

    mpu9250_gyro_range_t gyro_range = get_gyro_range();

    float gyro_scale = 1;
    if (gyro_range == MPU9250_RANGE_250_DEG) {
        gyro_scale = 131;
    }
    if (gyro_range == MPU9250_RANGE_500_DEG) {
        gyro_scale = 65.5;
    }
    if (gyro_range == MPU9250_RANGE_1000_DEG) {
        gyro_scale = 32.8;
    }
    if (gyro_range == MPU9250_RANGE_2000_DEG) {
        gyro_scale = 16.4;
    }

    gyro_.x = static_cast<float>(rawGyroX) / gyro_scale * MPU9250_DPS_TO_RPS;
    gyro_.y = static_cast<float>(rawGyroY) / gyro_scale * MPU9250_DPS_TO_RPS;
    gyro_.z = static_cast<float>(rawGyroZ) / gyro_scale * MPU9250_DPS_TO_RPS;

    // only update sensor data if the magnetometer has a new data
    if (ak8963_i2c_read(AK8963_ST1) & 0x1) {
        ak8963_i2c_read(AK8963_HXL, buffer, 7);

        //  ignore new data if there was overflow
        uint8_t st2 = buffer[6];
        if ((st2 & 0x8) == 0) {
            int16_t rawMagX, rawMagY, rawMagZ;
            rawMagX = buffer[1] << 8 | buffer[0];
            rawMagY = buffer[3] << 8 | buffer[2];
            rawMagZ = buffer[5] << 8 | buffer[4];

            ak8963_mag_sensitivity_t mag_sensitivity = get_mag_sensitivity();
            float mag_scale;
            if (mag_sensitivity == AK8963_SENSITIVITY_16) {
                // 16b sensitivity
                mag_scale = 0.15;
            } else {
                // 14b sensitivity
                mag_scale = 0.6;
            }

            // The magnetometer in the MPU9250 is not aligned the same way as
            // the accelerometer and gyro.  The x and y axes of the acceleromter
            // are swapped when aligned with the coordinate system of the
            // magnetometer and the z directions are reversed.  Adjust the
            // assignments of mag.x, mag.y, and mag.z so the coodinate systems
            // match
            mag_.x = asa_.y * rawMagY * mag_scale;
            mag_.y = asa_.x * rawMagX * mag_scale;
            mag_.z = -(asa_.z * rawMagZ * mag_scale);
        }
    }

    if (accel) {
        *accel = accel_;
    }
    if (gyro) {
        *gyro = gyro_;
    }
    if (mag) {
        *mag = mag_;
    }
    if (temperature) {
        *temperature = temperature_;
    }

    return true;
}

//
// MPU9250::mpu9250_i2c_read
//
// Arguments: reg_addr -- Address of MPU9250 register to read
//
// Returns: success - value of register
//          failure - 0 (may conflict with valid register value)
//
// Function to read a register from the MPU9250 using I2C.  The value of the
// register is returned with no reliable way to return an error.  Use the
// version of this function with user buffer and buffer size if error checking
// is desired.
//

uint8_t MPU9250::mpu9250_i2c_read(uint8_t reg_addr) {
    uint8_t data;
    if (!i2c_read(wire_, mpu9250_i2c_addr_, reg_addr, &data, 1)) {
        return 0;
    }

    return data;
}

//
// MPU9250::mpu9250_i2c_read
//
// Arguments: reg_addr -- Address of MPU9250 register to read
//            buffer -- User provided buffer for result of the read
//            len -- Number of bytes to read
//
// Returns: success -- true
//          failure -- false
//
// Function to read a register from the MPU9250 using I2C.  The value of the
// register is written to the user provided buffer which must be capable of
// holding at least "len" bytes.  On success true is returned and any error
// will result in false being returned.
//

bool MPU9250::mpu9250_i2c_read(uint8_t reg_addr, uint8_t *buffer, size_t len) {
    return i2c_read(wire_, mpu9250_i2c_addr_, reg_addr, buffer, len);
}

//
// MPU9250::ak8963_i2c_read
//
// Arguments: reg_addr -- Address of AK8963 register to read
//
// Returns: success - value of register
//          failure - 0 (may conflict with valid register value)
//
// Function to read a register from the AK8963 magnetometer within the MPU9250
// using I2C.  The value of the register is returned with no reliable way to
// return an error.  Use the version of this function with user buffer and
// buffer size if error checking is desired.
//

uint8_t MPU9250::ak8963_i2c_read(uint8_t reg_addr) {
    uint8_t data;
    if (!i2c_read(wire_, ak8963_i2c_addr_, reg_addr, &data, 1)) {
        return 0;
    }

    return data;
}

//
// MPU9250::ak8963_i2c_read
//
// Arguments: reg_addr -- Address of AK8963 register to read
//            buffer -- User provided buffer for result of the read
//            len -- Number of bytes to read
//
// Returns: success -- true
//          failure -- false
//
// Function to read a register from the AK8963 magnetometer within the MPU9250
// using I2C.  The value of the register is written to the user provided buffer
// which must be capable of holding at least "len" bytes.  On success true is
// returned and any error will result in false being returned.
//

bool MPU9250::ak8963_i2c_read(uint8_t reg_addr, uint8_t *buffer, size_t len) {
    return i2c_read(wire_, ak8963_i2c_addr_, reg_addr, buffer, len);
}

//
// MPU9250::mpu9250_i2c_write
//
// Arguments: reg_addr -- Address of MPU9250 register to write
//
// Returns: Nothing
//
// Function to write a register in the MPU9250 using I2C.  No value is
// returned.  Use the version of this function with user buffer and buffer size
// if error checking is desired.
//

void MPU9250::mpu9250_i2c_write(uint8_t reg_addr, uint8_t data) {
    i2c_write(wire_, mpu9250_i2c_addr_, reg_addr, &data, 1);
}

//
// MPU9250::mpu9250_i2c_write
//
// Arguments: reg_addr -- Address of MPU9250 register to write
//            buffer -- User provided buffer with data to write to the register
//            len -- Number of bytes to write
//
// Returns: success -- true
//          failure -- false
//
// Function to write a register in the MPU9250 using I2C.  The value written
// to the register is taken from the user provided buffer of size "len".  On
// success true is returned and any error will result in false being returned.
//

bool MPU9250::mpu9250_i2c_write(uint8_t reg_addr, const uint8_t *buffer,
                                size_t len) {
    return i2c_write(wire_, mpu9250_i2c_addr_, reg_addr, buffer, len);
}

//
// MPU9250::ak8963_i2c_write
//
// Arguments: reg_addr -- Address of AK8963 register to write
//
// Returns: Nothing
//
// Function to write a register in the AK8963 magnetometer in the MPU9250 using
// I2C.  No value is returned.  Use the version of this function with user
// buffer and buffer size if error checking is desired.
//

void MPU9250::ak8963_i2c_write(uint8_t reg_addr, uint8_t data) {
    i2c_write(wire_, ak8963_i2c_addr_, reg_addr, &data, 1);
}

//
// MPU9250::ak8963_i2c_write
//
// Arguments: reg_addr -- Address of AK8963 register to write
//            buffer -- User provided buffer with data to write to the register
//            len -- Number of bytes to write
//
// Returns: success -- true
//          failure -- false
//
// Function to write a register in the AK8963 magnetometer in the MPU9250 using
// I2C.  The value written to the register is taken from the user provided
// buffer of size "len".  On success true is returned and any error will
// result in false being returned.
//

bool MPU9250::ak8963_i2c_write(uint8_t reg_addr, const uint8_t *buffer,
                               size_t len) {
    return i2c_write(wire_, ak8963_i2c_addr_, reg_addr, buffer, len);
}

//
// MPU9250::i2c_read
//
// Arguments: wire -- TwoWire object controlling `the I2C bus
//            device_addr -- Address of the device on the I2C bus
//            reg_addr -- Address of register to read
//            buffer -- User provided buffer for result of the read
//            len -- Number of bytes to read
//
// Returns: success -- true
//          failure -- false
//
// Function to read a register on I2C.  It assumes a single byte of address
// must be written to the device with the register address and that subsequent
// reads will access the register.
//

bool MPU9250::i2c_read(TwoWire *wire, uint8_t device_addr, uint8_t reg_addr,
                       uint8_t *buffer, size_t len) {
    wire->beginTransmission(device_addr);

    // write register address
    if (wire->write(reg_addr) != 1) {
        wire->endTransmission(true);
        return false;
    }
    if (wire->endTransmission(false) != 0) {
        wire->endTransmission(true);
        return false;
    }

    // read data from register
    if (wire->requestFrom(device_addr, len) != len) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        buffer[i] = wire->read();
    }
    return true;
}

//
// MPU9250::i2c_write
//
// Arguments: wire -- TwoWire object controlling the I2C bus
//            device_addr -- Address of the device on the I2C bus
//            reg_addr -- Address of register to write
//            buffer -- Buffer of data bytes to write
//            len -- Number of bytes to write
//
// Returns: success -- true
//          failure -- false
//
// Function to write a register on I2C.
//

bool MPU9250::i2c_write(TwoWire *wire, uint8_t device_addr, uint8_t reg_addr,
                        const uint8_t *buffer, size_t len) {
    wire->beginTransmission(device_addr);

    // write register address
    if (wire->write(reg_addr) != 1) {
        wire->endTransmission(true);
        return false;
    }

    if (wire->write(buffer, len) != len) {
        wire->endTransmission(true);
        return false;
    }

    if (wire->endTransmission(true) != 0) {
        return false;
    }

    return true;
}
