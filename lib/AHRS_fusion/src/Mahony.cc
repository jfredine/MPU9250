//
// Mahony
//
// Object implementing the Mahony algorthm for AHRS
//
// Derived heavily from work of Paul Stoffregen
// https://github.com/PaulStoffregen/MahonyAHRS
//

#include "Mahony.h"

#include <math.h>

#define two_Kp_Def (2.0f * 0.5f)  // 2 * proportional gain
#define two_Ki_Def (2.0f * 0.0f)  // 2 * integral gain

Mahony::Mahony() {
    two_Kp_ = two_Kp_Def;  // 2 * proportional gain (Kp)
    two_Ki_ = two_Ki_Def;  // 2 * integral gain (Ki)
    q0_ = 1.0f;
    q1_ = 0.0f;
    q2_ = 0.0f;
    q3_ = 0.0f;
    integral_FBx_ = 0.0f;
    integral_FBy_ = 0.0f;
    integral_FBz_ = 0.0f;
    roll_valid_ = false;
    pitch_valid_ = false;
    yaw_valid_ = false;
}

// gx, gy, and gz must be in radians per second
void Mahony::update(float gx, float gy, float gz, float ax, float ay, float az,
                    float mx, float my, float mz, float dt) {
    float recip_norm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        return;
    }

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
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) +
                     mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) +
                     mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) +
                     mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction
        // and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if (two_Ki_ > 0.0f) {
            // integral error scaled by Ki
            integral_FBx_ += two_Ki_ * halfex * dt;
            integral_FBy_ += two_Ki_ * halfey * dt;
            integral_FBz_ += two_Ki_ * halfez * dt;
            gx += integral_FBx_;  // apply integral feedback
            gy += integral_FBy_;
            gz += integral_FBz_;
        } else {
            integral_FBx_ = 0.0f;  // prevent integral windup
            integral_FBy_ = 0.0f;
            integral_FBz_ = 0.0f;
        }

        // Apply proportional feedback
        gx += two_Kp_ * halfex;
        gy += two_Kp_ * halfey;
        gz += two_Kp_ * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);  // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0_;
    qb = q1_;
    qc = q2_;
    q0_ += (-qb * gx - qc * gy - q3_ * gz);
    q1_ += (qa * gx + qc * gz - q3_ * gy);
    q2_ += (qa * gy - qb * gz + q3_ * gx);
    q3_ += (qa * gz + qb * gy - qc * gx);

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
