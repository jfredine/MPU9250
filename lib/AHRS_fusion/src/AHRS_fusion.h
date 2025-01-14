//
// Pure virtual class from which AHRS filters are derived
//

#ifndef AHRS_FUSION_H
#define AHRS_FUSION_H

#include <stdint.h>

// pure virtual class used by concrete implementations
class AHRS_fusion{
 public:
    virtual void update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz, float dt) = 0;

    virtual float get_yaw() = 0;
    virtual float get_pitch() = 0;
    virtual float get_roll() = 0;

    virtual void get_quaternion(float *w, float *x, float *y, float *z) = 0;

    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    static float inv_sqrt(float x) {
        float halfx = 0.5f * x;
        union {
            float f;
            uint32_t i;
        } conv = {x};
        conv.i = 0x5f3759df - (conv.i >> 1);
        conv.f *= 1.5f - (halfx * conv.f * conv.f);
        conv.f *= 1.5f - (halfx * conv.f * conv.f);
        return conv.f;
    }
};

#endif
