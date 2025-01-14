//
// Mahony.h
//
// Header for Mahony object implementing the Mahony algorithm for AHRS.
//

#ifndef MAHONY_H
#define MAHONY_H

#include <math.h>

#include "AHRS_fusion.h"

class Mahony : public AHRS_fusion {
 private:
    float two_Kp_;  // 2 * proportional gain (Kp)
    float two_Ki_;  // 2 * integral gain (Ki)
    float q0_, q1_, q2_, q3_;
    float integral_FBx_;  // integral error terms scaled by Ki
    float integral_FBy_;
    float integral_FBz_;
    float roll_, pitch_, yaw_;
    bool roll_valid_, pitch_valid_, yaw_valid_;

 public:
    Mahony();
    void update(float gx, float gy, float gz, float ax, float ay, float az,
                float mx, float my, float mz, float dt);
    float get_Kp() { return two_Kp_ / 2.0f; }
    void set_Kp(float Kp) { two_Kp_ = 2.0f * Kp; }
    float get_Ki() { return two_Ki_ / 2.0f; }
    void set_Ki(float Ki) { two_Ki_ = 2.0f * Ki; }
    float get_roll() {
        if (!roll_valid_) {
            roll_ = atan2f(q0_ * q1_ + q2_ * q3_, 0.5f - q1_ * q1_ - q2_ * q2_);
            roll_valid_ = true;
        }
        return roll_ * 57.29578f;
    }
    float get_pitch() {
        if (!pitch_valid_) {
            pitch_ = asinf(-2.0f * (q1_ * q3_ - q0_ * q2_));
            pitch_valid_ = true;
        }
        return pitch_ * 57.29578f;
    }
    float get_yaw() {
        if (!yaw_valid_) {
            yaw_ = atan2f(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_);
            yaw_valid_ = true;
        }
        return yaw_ * 57.29578f + 180.0f;
    }
    void get_quaternion(float *w, float *x, float *y, float *z) {
        *w = q0_;
        *x = q1_;
        *y = q2_;
        *z = q3_;
    }
};

#endif
