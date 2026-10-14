#pragma once
#include <numbers>

namespace pe {

// Soft constraint coefficients (Box2D v3). A rigid constraint is treated as a very stiff,
// heavily damped spring so position error is removed without injecting energy.
struct Softness {
    float biasRate = 0.0f;
    float massScale = 1.0f;
    float impulseScale = 0.0f;

    static Softness make(float hertz, float zeta, float h) {
        if (hertz == 0.0f) return {};
        float omega = 2.0f * std::numbers::pi_v<float> * hertz;
        float a1 = 2.0f * zeta + h * omega;
        float a2 = h * omega * a1;
        float a3 = 1.0f / (1.0f + a2);
        return {omega / a1, a2 * a3, a3};
    }
};

} // namespace pe
