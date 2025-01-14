//
// Madgwick
//
// Object implementing the Madgwick algorthm for AHRS
//
// Derived heavily from work of Paul Stoffregen
// https://github.com/PaulStoffregen/MadgwickAHRS
//

#include "Madgwick.h"

#include <math.h>

#define sampleFreqDef 512.0f  // sample frequency in Hz
#define betaDef 0.1f          // 2 * proportional gain

Madgwick::Madgwick() {
    beta_ = betaDef;
    q0_ = 1.0f;
    q1_ = 0.0f;
    q2_ = 0.0f;
    q3_ = 0.0f;
    roll_valid_ = false;
    pitch_valid_ = false;
    yaw_valid_ = false;
}

// gx, gy, and gz must be in radians per second
void Madgwick::update(float gx, float gy, float gz, float ax, float ay,
                      float az, float mx, float my, float mz, float dt) {
    float recip_norm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
          _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
          q2q2, q2q3, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
    qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
    qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
    qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // Normalise magnetometer measurement
        recip_norm = inv_sqrt(mx * mx + my * my + mz * mz);
        mx *= recip_norm;
        my *= recip_norm;
        mz *= recip_norm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0_ * mx;
        _2q0my = 2.0f * q0_ * my;
        _2q0mz = 2.0f * q0_ * mz;
        _2q1mx = 2.0f * q1_ * mx;
        _2q0 = 2.0f * q0_;
        _2q1 = 2.0f * q1_;
        _2q2 = 2.0f * q2_;
        _2q3 = 2.0f * q3_;
        _2q0q2 = 2.0f * q0_ * q2_;
        _2q2q3 = 2.0f * q2_ * q3_;
        q0q0 = q0_ * q0_;
        q0q1 = q0_ * q1_;
        q0q2 = q0_ * q2_;
        q0q3 = q0_ * q3_;
        q1q1 = q1_ * q1_;
        q1q2 = q1_ * q2_;
        q1q3 = q1_ * q3_;
        q2q2 = q2_ * q2_;
        q2q3 = q2_ * q3_;
        q3q3 = q3_ * q3_;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3_ + _2q0mz * q2_ + mx * q1q1 +
             _2q1 * my * q2_ + _2q1 * mz * q3_ - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3_ + my * q0q0 - _2q0mz * q1_ + _2q1mx * q2_ -
             my * q1q1 + my * q2q2 + _2q2 * mz * q3_ - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2_ + _2q0my * q1_ + mz * q0q0 + _2q1mx * q3_ -
               mz * q1q1 + _2q2 * my * q3_ - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
             _2bz * q2_ *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q3_ + _2bz * q1_) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q2_ *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * q1_ * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             _2bz * q3_ *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q2_ + _2bz * q0_) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q3_ - _4bz * q1_) *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
             4.0f * q2_ * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
             (-_4bx * q2_ - _2bz * q0_) *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (_2bx * q1_ + _2bz * q3_) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             (_2bx * q0_ - _4bz * q2_) *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
             _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
             (-_4bx * q3_ + _2bz * q1_) *
                 (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
             (-_2bx * q0_ + _2bz * q2_) *
                 (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
             _2bx * q1_ *
                 (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        // normalise step magnitude
        recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recip_norm;
        s1 *= recip_norm;
        s2 *= recip_norm;
        s3 *= recip_norm;

        // Apply feedback step
        qDot1 -= beta_ * s0;
        qDot2 -= beta_ * s1;
        qDot3 -= beta_ * s2;
        qDot4 -= beta_ * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0_ += qDot1 * dt;
    q1_ += qDot2 * dt;
    q2_ += qDot3 * dt;
    q3_ += qDot4 * dt;

    // Normalise quaternion
    recip_norm = inv_sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recip_norm;
    q1_ *= recip_norm;
    q2_ *= recip_norm;
    q3_ *= recip_norm;

    roll_valid_ = false;
    pitch_valid_ = false;
    yaw_valid_ = false;
}
