#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include "AC_CustomControl_INDI.h"
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <cmath>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_INDI::var_info[] = {
    // @Param: ATT_TLT_P
    // @DisplayName: INDI tilt attitude P gain
    // @Description: Reduced-attitude (tilt) P gain converting the tilt error into a desired body rate. Weighted fully; keep >= ATT_YAW_P so yaw is de-prioritized.
    // @Range: 3.0 12.0
    // @User: Standard
    AP_GROUPINFO("ATT_TLT_P", 1, AC_CustomControl_INDI, _kp_tilt, 6.0f),

    // @Param: ATT_YAW_P
    // @DisplayName: INDI yaw attitude P gain
    // @Description: Yaw P gain converting the yaw error into a desired body rate. Kept below ATT_TLT_P to de-prioritize yaw versus tilt.
    // @Range: 1.0 6.0
    // @User: Standard
    AP_GROUPINFO("ATT_YAW_P", 2, AC_CustomControl_INDI, _kp_yaw, 3.0f),

    // @Param: FILT_HZ
    // @DisplayName: INDI shared filter cutoff
    // @Description: Low-pass cutoff (Hz) shared by BOTH the gyro-derivative (angular-accel) filter and the actuator-state filter so their group delays are phase-matched (the single most important INDI implementation detail).
    // @Range: 10 80
    // @Units: Hz
    // @User: Standard
    AP_GROUPINFO("FILT_HZ", 3, AC_CustomControl_INDI, _filt_hz, 40.0f),

    // @Param: G1_RP
    // @DisplayName: INDI roll/pitch control effectiveness
    // @Description: G1 diagonal control-effectiveness term for the roll and pitch axes (rad/s^2 per unit actuator increment).
    // @User: Advanced
    AP_GROUPINFO("G1_RP", 4, AC_CustomControl_INDI, _g1_rp, 1.0f),

    // @Param: G1_YAW
    // @DisplayName: INDI yaw control effectiveness
    // @Description: G1 diagonal control-effectiveness term for the yaw axis (rad/s^2 per unit actuator increment).
    // @User: Advanced
    AP_GROUPINFO("G1_YAW", 5, AC_CustomControl_INDI, _g1_yaw, 1.0f),

    AP_GROUPEND
};

AC_CustomControl_INDI::AC_CustomControl_INDI(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
}

// --- S0 tilt-prioritized attitude->rate reference (Task 2) -----------------
// Direct port of indi_harness.tilt_yaw.attitude_rate_ref (scalar-first Hamilton
// quaternions [w,x,y,z]). All intermediate math is done in double so the
// float32 Quaternion I/O still reproduces the float64 Python oracle to < 1e-5.
namespace {

// Hamilton product a (x) b (scalar-first), matching indi_harness.quat.qmul.
void qmul(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    out[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    out[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    out[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void qnormalize(double q[4])
{
    const double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (uint8_t i = 0; i < 4; i++) {
        q[i] /= n;
    }
}

// Quaternion -> rotation vector (rad). atan2 gives the full-range angle (not
// limited to the shortest arc), matching indi_harness.quat.qlog.
void qlog(const double q_in[4], double out[3])
{
    double q[4] = { q_in[0], q_in[1], q_in[2], q_in[3] };
    qnormalize(q);
    const double vn = sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (vn < 1e-12) {
        out[0] = 2.0*q[1]; out[1] = 2.0*q[2]; out[2] = 2.0*q[3];
        return;
    }
    const double s = (2.0 * atan2(vn, q[0])) / vn;
    out[0] = s*q[1]; out[1] = s*q[2]; out[2] = s*q[3];
}

// Split error quaternion qe into (q_red, q_yaw) with qe = q_red (x) q_yaw;
// q_red carries the tilt (zero z), q_yaw is a pure z rotation. Near 180 deg
// tilt (hypot(w,z) -> 0) yaw is undefined: all error goes to tilt.
// Mirrors indi_harness.tilt_yaw.tilt_yaw_decompose.
void tilt_yaw_decompose(const double qe[4], double q_red[4], double q_yaw[4])
{
    const double w = qe[0], x = qe[1], y = qe[2], z = qe[3];
    const double n = hypot(w, z);
    if (n < 1e-9) {
        q_red[0] = 0.0; q_red[1] = x; q_red[2] = y; q_red[3] = 0.0;
        qnormalize(q_red);
        q_yaw[0] = 1.0; q_yaw[1] = 0.0; q_yaw[2] = 0.0; q_yaw[3] = 0.0;
        return;
    }
    q_red[0] = (w*w + z*z) / n;
    q_red[1] = (w*x - y*z) / n;
    q_red[2] = (w*y + x*z) / n;
    q_red[3] = 0.0;
    q_yaw[0] = w / n; q_yaw[1] = 0.0; q_yaw[2] = 0.0; q_yaw[3] = z / n;
    qnormalize(q_red);
    qnormalize(q_yaw);
}

} // namespace

Vector3f AC_CustomControl_INDI::attitude_rate_ref(const Quaternion &q, const Quaternion &q_ref,
                                                  float kp_tilt, float kp_yaw, const Vector3f &w_ff)
{
    // Error quaternion qe = conj(q) (x) q_ref, scalar part forced >= 0 so the
    // controller always takes the short way round (no unwinding).
    const double qd[4]  = { q.q1, q.q2, q.q3, q.q4 };
    const double qrd[4] = { q_ref.q1, q_ref.q2, q_ref.q3, q_ref.q4 };
    const double qc[4]  = { qd[0], -qd[1], -qd[2], -qd[3] };
    double qe[4];
    qmul(qc, qrd, qe);
    if (qe[0] < 0.0) {
        for (uint8_t i = 0; i < 4; i++) {
            qe[i] = -qe[i];
        }
    }

    double q_red[4], q_yaw[4];
    tilt_yaw_decompose(qe, q_red, q_yaw);

    double lr[3], ly[3];
    qlog(q_red, lr);
    qlog(q_yaw, ly);

    const double kt = kp_tilt, ky = kp_yaw;
    return Vector3f(
        float(kt*lr[0] + ky*ly[0] + (double)w_ff.x),
        float(kt*lr[1] + ky*ly[1] + (double)w_ff.y),
        float(kt*lr[2] + ky*ly[2] + (double)w_ff.z));
}

Vector3f AC_CustomControl_INDI::update(void)
{
    // Behaviour-neutral until the INDI law lands (Tasks 2-3): hand back the
    // stock rate-controller output so CC_TYPE=3 changes nothing yet. This
    // proves the framework wiring + build in isolation.
    //
    // run_rate_controller_main runs immediately before run_custom_controller
    // (Copter.cpp scheduler), writing the stock output into the motors' roll/
    // pitch/yaw inputs. Reading those back and returning them means motor_set()
    // writes the identical values -- a genuine no-op.
    return Vector3f(_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw());
}

void AC_CustomControl_INDI::reset(void)
{
    // No integrator/filter state yet (behaviour-neutral). Filter reset lands
    // with the INDI rate loop in Task 3.
}

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
