#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>

#include "AC_CustomControl_Backend.h"
#include "AC_CustomControl_OuterLoop.h"

// Layer-A INDI rate loop: filtered angular-accel estimate + phase-matched
// actuator-state estimate + diagonal-G1 inversion, producing the INDI torque
// increment u = u_filt + G1^-1 (dw_cmd - domega_filt). The angular-accel
// filter and the actuator-state filter are built from a SINGLE cutoff so their
// group delays match (the phase-matching rule -- "the single most important
// INDI implementation detail"; a mismatch destabilizes the loop, the S2
// synchronization lesson, pinned by a regression test).
class AC_INDI_RateLoop {
public:
    AC_INDI_RateLoop() {}
    CLASS_NO_COPY(AC_INDI_RateLoop);

    // Production configure: ONE cutoff drives both filters (phase-matched).
    void configure(float cutoff_hz, float sample_freq,
                   const Vector3f &kw, const Vector3f &g1);

    // TEST-ONLY: independent cutoffs, used only to demonstrate that a filter
    // group-delay mismatch destabilizes the loop. Never called by flight code.
    void configure_split(float cutoff_domega_hz, float cutoff_uact_hz,
                         float sample_freq, const Vector3f &kw, const Vector3f &g1);

    // C1: select the angular-accel estimator. cutoff_hz > 0 selects
    // filter-then-differentiate (gyro_f = LPF(gyro); domega = d/dt gyro_f),
    // re-pointing the actuator-state filter to the SAME cutoff so the two
    // stay phase-matched. cutoff_hz == 0 restores the legacy
    // differentiate-then-filter path built by configure()/configure_split().
    void configure_estimator(float cutoff_hz, float sample_freq);

    void reset();

    // One INDI rate-loop iteration.
    //   gyro   : measured body rate [rad/s]
    //   u_act  : measured actuator state (stock-mixer torque roll/pitch/yaw)
    //   w_des  : desired body rate [rad/s] (from the attitude reference)
    //   dw_ff  : angular-accel feedforward [rad/s^2]
    //   u_limit: per-axis actuator magnitude limit (mixer range)
    // Returns u_cmd = u_filt + G1^-1(dw_cmd - domega_filt); on saturation the
    // yaw increment is shed first (prioritized allocation) then all axes clip.
    Vector3f step(float dt, const Vector3f &gyro, const Vector3f &u_act,
                  const Vector3f &w_des, const Vector3f &dw_ff, float u_limit,
                  Vector3f &domega_pred, Vector3f &domega_filt,
                  Vector3f &u_filt, bool &sat);

    // group-delay-match introspection (used by the phase-match regression test).
    // In filter-then-differentiate mode the gyro pre-filter is the one paired
    // with the actuator-state filter, so report ITS cutoff, not _f_domega's
    // (which sits unused in that mode).
    float domega_cutoff() const { return _use_ftd ? _f_gyro.get_cutoff_freq() : _f_domega.get_cutoff_freq(); }
    float uact_cutoff() const { return _f_uact.get_cutoff_freq(); }

private:
    LowPassFilter2pVector3f _f_domega;   // angular-accel estimate (legacy diff-then-filter)
    LowPassFilter2pVector3f _f_uact;     // actuator-state estimate (same cutoff as whichever estimate path is active)
    LowPassFilter2pVector3f _f_gyro;     // C1: gyro pre-filter (filter-then-differentiate)
    Vector3f _kw;                        // rate-error -> desired ang-accel gain
    Vector3f _g1_inv;                    // diagonal G1^-1 (rad/s^2 per unit u)
    Vector3f _prev_gyro;                 // legacy path: previous raw gyro
    Vector3f _prev_gyro_f;               // C1 path: previous filtered gyro
    bool _have_prev = false;
    bool _use_ftd = false;               // C1: filter-then-differentiate selected
};

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

    // Rate-error -> desired angular-accel gain kw (Task 3): roll/pitch and yaw.
    AP_Float _kw_rp;
    AP_Float _kw_yaw;

    // C1 (Layer C): angular-accel estimator pre-filter cutoff (Hz). >0 selects
    // filter-then-differentiate (gyro_f = LPF(gyro); domega = d/dt gyro_f) and
    // re-points the actuator-state filter to this SAME cutoff so the two stay
    // phase-matched. 0 = legacy differentiate-then-filter.
    AP_Float _omg_filt;

    // --- Layer-B outer loop params (Task B4) ---
    // Enable the in-firmware INDI outer loop. 0 = stock guided outer loop (C1
    // as-is); 1 = run AC_INDI_OuterLoop off a fresh DDS FlatSetpoint. Param
    // CC3_OUTER_EN.
    AP_Int8 _outer_en;
    // Position (kp) and velocity (kv) gains, roll/pitch (XY) and z (Z).
    // Params CC3_B_KP_XY/KP_Z/KV_XY/KV_Z.
    AP_Float _b_kp_xy;
    AP_Float _b_kp_z;
    AP_Float _b_kv_xy;
    AP_Float _b_kv_z;
    // Outer-loop INDI specific-force/thrust-state filter cutoff (Hz). The
    // phase-margin knob swept in Task B5. Param CC3_B_ACC_FILT.
    AP_Float _b_acc_filt;
    // Max FlatSetpoint age (ms) before the outer loop falls back to stock.
    // Param CC3_B_DDS_TMO.
    AP_Int16 _b_dds_tmo;
    // Drive the collective throttle from the outer loop's thrust command when
    // active (coordinates thrust with tilt). Param CC3_B_THR_EN.
    AP_Int8 _b_thr_en;

    // The INDI rate loop and a one-shot configure guard (params are only valid
    // after load_object_from_eeprom, which runs after construction).
    AC_INDI_RateLoop _rate_loop;
    bool _rate_loop_configured = false;

    // Layer-B outer loop + one-shot configure guard. Fed a fresh DDS flat
    // reference; produces (q_ref, w_ff, dw_ff) replacing the stock target.
    AC_INDI_OuterLoop _outer;
    bool _outer_configured = false;
    // previous-tick closed-form w_z for the inter-tick dw_z feedforward.
    float _prev_wz = 0.0f;
    bool _have_prev_wz = false;
};

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
