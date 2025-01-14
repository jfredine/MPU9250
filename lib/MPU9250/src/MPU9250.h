//
// MPU9250.h
//
// Declaration of the MPU9250 class used for control of the MPU9250
// motion sensor.
//

#ifndef MPU9250_H
#define MPU9250_H

#include <Arduino.h>
#include <Wire.h>
#include <string.h>

// basic conversion constants
#define MPU9250_G_TO_MPS2  9.80665
#define MPU9250_DPS_TO_RPS 0.017453293
#define MPU9250_RPS_TO_DPS 57.29578

// MPU9250 accelerometer and gyro
#define MPU9250_I2CADDR_DEFAULT 0x68
#define MPU9250_DEVICE_ID 0x71

// see MPU9250 register map
#define MPU9250_SELF_TEST_X_GYRO  0x00
#define MPU9250_SELF_TEST_Y_GYRO  0x01
#define MPU9250_SELF_TEST_Z_GYRO  0x02
#define MPU9250_SELF_TEST_X_ACCEL 0x0D
#define MPU9250_SELF_TEST_Y_ACCEL 0x0E
#define MPU9250_SELF_TEST_Z_ACCEL 0x0F
#define MPU9250_SMPLRT_DIV        0x19
#define MPU9250_CONFIG            0x1A
#define MPU9250_GYRO_CONFIG       0x1B
#define MPU9250_ACCEL_CONFIG      0x1C
#define MPU9250_ACCEL_CONFIG2     0x1D
#define MPU9250_INT_PIN_CFG       0x37
#define MPU9250_WHO_AM_I          0x75
#define MPU9250_SIGNAL_PATH_RESET 0x68
#define MPU9250_USER_CTRL         0x6A
#define MPU9250_PWR_MGMT_1        0x6B
#define MPU9250_PWR_MGMT_2        0x6C
#define MPU9250_ACCEL_XOUT_H      0x3B

typedef enum clock_select {
    MPU9250_INTERNAL_20MHz,
    MPU9250_PLL,
    MPU9250_STOP = 7,
} mpu9250_clock_select_t;

typedef enum {
    MPU9250_RANGE_2_G = 0b00,
    MPU9250_RANGE_4_G = 0b01,
    MPU9250_RANGE_8_G = 0b10,
    MPU9250_RANGE_16_G = 0b11,
} mpu9250_accel_range_t;

typedef enum {
    MPU9250_RANGE_250_DEG,
    MPU9250_RANGE_500_DEG,
    MPU9250_RANGE_1000_DEG,
    MPU9250_RANGE_2000_DEG,
} mpu9250_gyro_range_t;

typedef enum {
    MPU9250_GYRO_BAND_250_HZ,
    MPU9250_GYRO_BAND_184_HZ,
    MPU9250_GYRO_BAND_92_HZ,
    MPU9250_GYRO_BAND_41_HZ,
    MPU9250_GYRO_BAND_20_HZ,
    MPU9250_GYRO_BAND_10_HZ,
    MPU9250_GYRO_BAND_5_HZ,
    MPU9250_GYRO_BAND_3600_HZ,
    MPU9250_GYRO_BAND_8800_HZ,
} mpu9250_gyro_bandwidth_t;

typedef enum {
    MPU9250_ACCEL_BAND_460_HZ,
    MPU9250_ACCEL_BAND_184_HZ,
    MPU9250_ACCEL_BAND_92_HZ,
    MPU9250_ACCEL_BAND_41_HZ,
    MPU9250_ACCEL_BAND_20_HZ,
    MPU9250_ACCEL_BAND_10_HZ,
    MPU9250_ACCEL_BAND_5_HZ,
    MPU9250_ACCEL_BAND_1130_HZ = 0b1000,
} mpu9250_accel_bandwidth_t;

// AK8963 magnetometer
#define AK8963_I2CADDR_DEFAULT 0x0c
#define AK8963_DEVICE_ID 0x48

// see AK8963 register map
#define AK8963_WHO_AM_I 0x00
#define AK8963_INFO     0x01
#define AK8963_ST1      0x02
#define AK8963_HXL      0x03
#define AK8963_ST2      0x09
#define AK8963_CNTL     0x0A
#define AK8963_ASTC     0x0C
#define AK8963_I2CDIS   0x0F
#define AK8963_ASAX     0x10

typedef enum {
    AK8963_MODE_POWER_DOWN,
    AK8963_MODE_SINGLE,
    AK8963_MODE_8HZ,
    AK8963_MODE_EXT_TRIG = 0x4,
    AK8963_MODE_100HZ = 0x6,
    AK8963_MODE_SELF_TEST = 0x8,
    AK8963_MODE_FUSE_ROM = 0xf
} ak8963_mag_mode_t;

typedef enum {
    AK8963_SENSITIVITY_14,
    AK8963_SENSITIVITY_16
} ak8963_mag_sensitivity_t;

class MPU9250 {
 public:
    MPU9250();
    ~MPU9250();

    template <typename T>
    struct tuple {
        T x;
        T y;
        T z;
    };

    int init(uint8_t mpu9250_i2c_addr = MPU9250_I2CADDR_DEFAULT,
             uint8_t ak8963_i2c_addr = AK8963_I2CADDR_DEFAULT,
             TwoWire *wire = &Wire);

    void reset(void);

    void set_I2C_bypass(bool bypass);

    mpu9250_clock_select_t get_clock(void);
    void set_clock(mpu9250_clock_select_t);

    mpu9250_accel_range_t get_accel_range(void);
    void set_accel_range(mpu9250_accel_range_t);

    mpu9250_gyro_range_t get_gyro_range(void);
    void set_gyro_range(mpu9250_gyro_range_t);

    mpu9250_accel_bandwidth_t get_accel_filter_bandwidth(void);
    void set_accel_filter_bandwidth(mpu9250_accel_bandwidth_t bandwidth);

    mpu9250_gyro_bandwidth_t get_gyro_filter_bandwidth(void);
    void set_gyro_filter_bandwidth(mpu9250_gyro_bandwidth_t bandwidth);

    ak8963_mag_mode_t get_mag_mode(void);
    void set_mag_mode(ak8963_mag_mode_t mag_mode);

    ak8963_mag_sensitivity_t get_mag_sensitivity(void);
    void set_mag_sensitivity(ak8963_mag_sensitivity_t mag_sensitivity);

    int read(tuple<float> *accel, tuple<float> *gyro, tuple<float> *mag,
             float *temperature);

 private:
    float temperature_;   // Last reading's temperature (C)
    tuple<float> accel_;  // Last reading's accelerometer X, Y, Z axis m/s^2
    tuple<float> gyro_;   // Last readings' gyro X, Y, Z in rad/s
    tuple<float> mag_;    // Last readings magnetometer X, Y, Z in uT
    tuple<float> asa_;    // mag sensitivity adjustment data for X, Y, and Z

    uint8_t mpu9250_i2c_addr_;
    uint8_t ak8963_i2c_addr_;
    TwoWire *wire_;

    uint8_t mpu9250_i2c_read(uint8_t reg_addr);
    bool mpu9250_i2c_read(uint8_t reg_addr, uint8_t *buffer, size_t len);
    void mpu9250_i2c_write(uint8_t reg_addr, uint8_t data);
    bool mpu9250_i2c_write(uint8_t reg_addr, const uint8_t *buffer, size_t len);

    uint8_t ak8963_i2c_read(uint8_t reg_addr);
    bool ak8963_i2c_read(uint8_t reg_addr, uint8_t *buffer, size_t len);
    void ak8963_i2c_write(uint8_t reg_addr, uint8_t data);
    bool ak8963_i2c_write(uint8_t reg_addr, const uint8_t *buffer, size_t len);

    static bool i2c_read(TwoWire *wire, uint8_t device_addr, uint8_t reg_addr,
                         uint8_t *buffer, size_t len);
    static bool i2c_write(TwoWire *wire, uint8_t device_addr, uint8_t reg_addr,
                          const uint8_t *buffer, size_t len);
};

#endif
