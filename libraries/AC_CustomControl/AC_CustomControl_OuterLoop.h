#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Math/AP_Math.h>

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
};

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
