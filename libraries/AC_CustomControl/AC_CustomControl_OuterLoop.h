#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter2p.h>

// Layer-B INDI outer loop (design-doc S3). A C++ port of the tested S0 Python
// reference (indi_harness/{flatness,outer_loop}.py). Part 1 (Task B1) is the
// pure differential-flatness map FlatOutput -> RefState; Part 2 (Task B2) adds
// the linear-INDI position/velocity + thrust-vector-increment loop.
//
// Conventions: NED world frame, thrust along -z_b; scalar-first Hamilton
// quaternions [w,x,y,z] mapping body -> NED (matching the C1 inner-loop port).
// All intermediate math is done in double so the float32 I/O still reproduces
// the float64 Python oracle (see tests/test_indi_math.cpp for oracle values).
class AC_INDI_OuterLoop {
public:
    // Flat-output reference at one instant (NED). Mirrors
    // indi_harness.trajectory.FlatOutput.
    struct FlatOutput {
        Vector3f p, v, a, j, s;   // position and its 1st..4th derivatives
        float psi, dpsi, ddpsi;   // yaw and its 1st..2nd derivatives
    };

    // Differential-flatness reference. Mirrors indi_harness.flatness.RefState.
    // dw.z is the yaw angular-accel feedforward: the oracle obtains it by an
    // inter-tick central difference of the closed-form w_z, so the pure map
    // below leaves dw.z == 0 (the closed-form q, w (incl w_z), dw_x, dw_y, T
    // are exact). In flight the backend fills dw.z by finite-differencing the
    // returned w.z across successive setpoints.
    struct RefState {
        Quaternion q;   // body->NED reference attitude
        Vector3f w;     // body-rate feedforward [rad/s]
        Vector3f dw;    // body angular-accel feedforward [rad/s^2]
        float T;        // thrust magnitude [N], along -z_b
    };

    // Pure differential-flatness map (port of indi_harness.flatness._core).
    // Static/stateless: computes q, w (incl closed-form w_z), dw_x, dw_y, T;
    // dw.z is returned as 0 (see RefState). Near-parallel z_b/x_c (~90 deg
    // pitch along heading) is guarded to stay finite.
    static RefState flat_reference(const FlatOutput &fo, float m, float g);

    // One outer-loop iteration output.
    struct OuterState {
        Vector3f z_b_des;   // desired body-z direction in NED (unit)
        float T_cmd;        // collective thrust command [N]
    };

    AC_INDI_OuterLoop() {}
    CLASS_NO_COPY(AC_INDI_OuterLoop);

    // Configure the linear-INDI outer loop: position (kp) and velocity (kv)
    // gains, ONE low-pass cutoff shared by the two phase-matched INDI filters
    // (measured specific force + thrust-vector state -- the accel/thrust
    // analogue of the inner loop's angular-accel/actuator phase-matching), the
    // sample rate, and mass/gravity.
    void configure(const Vector3f &kp, const Vector3f &kv,
                   float cutoff_hz, float sample_freq, float m, float g);

    void reset();

    // One outer-loop iteration. Port of
    // indi_harness.outer_loop.OuterLoopINDI.update.
    //   p, v      : measured position / velocity (NED) [m], [m/s]
    //   q         : attitude, body -> NED
    //   f_b_meas  : measured body-frame specific force [m/s^2]
    //   T_state   : current thrust-vector-state magnitude / mass [m/s^2] --
    //               the firmware substitute for kf*sum(Omega^2)/m (no RPM in
    //               SITL; see Task B3/B4 for how it is derived from throttle)
    //   fo        : flat-output reference (uses p, v, a)
    // Returns the desired body-z direction and the collective thrust [N].
    OuterState update(const Vector3f &p, const Vector3f &v, const Quaternion &q,
                      const Vector3f &f_b_meas, float T_state, const FlatOutput &fo);

    // Reference quaternion from a desired body-z direction and yaw. Same triad
    // as the flatness map, with the degenerate-yaw guard. Pure/static. Port of
    // indi_harness.outer_loop.attitude_from_thrust_dir.
    static Quaternion attitude_from_thrust_dir(const Vector3f &z_b_des, float psi);

private:
    Vector3f _kp{6.0f, 6.0f, 6.0f};
    Vector3f _kv{4.0f, 4.0f, 4.0f};
    float _m = 1.0f;
    float _g = GRAVITY_MSS;
    LowPassFilter2pVector3f _f_accel;   // measured specific force (world)
    LowPassFilter2pVector3f _f_state;   // thrust-vector state (world)
};

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
