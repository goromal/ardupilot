#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#include "AC_CustomControl_Backend.h"

// Layer-A INDI attitude/rate backend (design-doc S3).
// Replaces only the inner attitude/rate loop: the stock guided-mode
// position->attitude outer loop still runs and hands us an attitude target.
// The control math is a C++ port of the tested S0 Python reference
// (indi_harness/{tilt_yaw,inner_loop,allocation}.py); see the gtest in
// tests/test_indi_math.cpp for the oracle values.
class AC_CustomControl_INDI : public AC_CustomControl_Backend {
public:
    AC_CustomControl_INDI(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    // run the INDI attitude/rate law, return roll/pitch/yaw actuator output
    Vector3f update(void) override;
    void reset(void) override;

    // Tilt-prioritized attitude -> desired body-rate reference (Task 2).
    // Pure function (no member state) so it is unit-testable directly. q and
    // q_ref are body->NED quaternions [w,x,y,z]; returns a body-frame rate
    // command [rad/s]. Direct port of indi_harness.tilt_yaw.attitude_rate_ref:
    // qe = conj(q)*q_ref (scalar part forced >= 0, no unwinding), split into a
    // reduced-tilt and a pure-yaw rotation, weighted by kp_tilt and kp_yaw
    // (kp_yaw < kp_tilt de-prioritizes yaw), plus the reference rate w_ff.
    static Vector3f attitude_rate_ref(const Quaternion &q, const Quaternion &q_ref,
                                      float kp_tilt, float kp_yaw, const Vector3f &w_ff);

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:
    // controller sample period (s)
    float _dt;

    // --- placeholder params (declared now, wired up in Tasks 2-3) ---
    // Tilt-prioritized attitude->rate reference gains (Task 2).
    AP_Float _kp_tilt;      // tilt (reduced attitude) P gain
    AP_Float _kp_yaw;       // yaw P gain (de-prioritized: keep < _kp_tilt)

    // Shared angular-accel / actuator-state low-pass cutoff (Hz) (Task 3).
    // A SINGLE cutoff drives BOTH the gyro-derivative filter and the
    // actuator-state filter so their group delays are phase-matched -- the
    // S2 synchronization lesson. Do not split this into two params.
    AP_Float _filt_hz;

    // G1 control-effectiveness diagonal (Task 3): roll/pitch and yaw.
    AP_Float _g1_rp;
    AP_Float _g1_yaw;
};

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
