#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include "AC_CustomControl_OuterLoop.h"
#include <cmath>

// --- Layer-B outer loop math (Task B1: differential-flatness map) -----------
// Direct port of indi_harness.flatness._core (NED, thrust along -z_b). All
// intermediate math in double so the float32 Quaternion/Vector3f I/O still
// reproduces the float64 Python oracle to < 1e-4 (same approach as the C1
// inner-loop port in AC_CustomControl_INDI.cpp).
namespace {

typedef double Vec3[3];

inline double dot3(const Vec3 a, const Vec3 b)
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

inline void cross3(const Vec3 a, const Vec3 b, Vec3 out)
{
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

inline double norm3(const Vec3 a)
{
    return sqrt(dot3(a, a));
}

// Rotation matrix (columns [x_b | y_b | z_b]) -> scalar-first quaternion,
// scalar part forced >= 0. Shepperd's method, matching indi_harness.quat.q_from_R.
// R is indexed R[row][col].
void q_from_R(const double R[3][3], double q[4])
{
    const double tr = R[0][0] + R[1][1] + R[2][2];
    if (tr > 0.0) {
        double s = sqrt(tr + 1.0) * 2.0;
        q[0] = 0.25 * s;
        q[1] = (R[2][1] - R[1][2]) / s;
        q[2] = (R[0][2] - R[2][0]) / s;
        q[3] = (R[1][0] - R[0][1]) / s;
    } else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
        double s = sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0;
        q[0] = (R[2][1] - R[1][2]) / s;
        q[1] = 0.25 * s;
        q[2] = (R[0][1] + R[1][0]) / s;
        q[3] = (R[0][2] + R[2][0]) / s;
    } else if (R[1][1] > R[2][2]) {
        double s = sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0;
        q[0] = (R[0][2] - R[2][0]) / s;
        q[1] = (R[0][1] + R[1][0]) / s;
        q[2] = 0.25 * s;
        q[3] = (R[1][2] + R[2][1]) / s;
    } else {
        double s = sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0;
        q[0] = (R[1][0] - R[0][1]) / s;
        q[1] = (R[0][2] + R[2][0]) / s;
        q[2] = (R[1][2] + R[2][1]) / s;
        q[3] = 0.25 * s;
    }
    const double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    for (uint8_t i = 0; i < 4; i++) {
        q[i] /= n;
    }
    if (q[0] < 0.0) {
        for (uint8_t i = 0; i < 4; i++) {
            q[i] = -q[i];
        }
    }
}

} // namespace

AC_INDI_OuterLoop::RefState
AC_INDI_OuterLoop::flat_reference(const FlatOutput &fo, float m, float g)
{
    const Vec3 a  = { fo.a.x, fo.a.y, fo.a.z };
    const Vec3 j  = { fo.j.x, fo.j.y, fo.j.z };
    const Vec3 s  = { fo.s.x, fo.s.y, fo.s.z };

    // alpha = g*e3 - a  (mass-normalized thrust vector, along +z_b); its
    // derivatives are -jerk, -snap.
    const Vec3 alpha   = { -a[0], -a[1], (double)g - a[2] };
    const Vec3 dalpha  = { -j[0], -j[1], -j[2] };
    const Vec3 ddalpha = { -s[0], -s[1], -s[2] };

    const double T_bar = norm3(alpha);
    Vec3 z_b = { alpha[0]/T_bar, alpha[1]/T_bar, alpha[2]/T_bar };

    const double dT = dot3(z_b, dalpha);
    Vec3 dz = { (dalpha[0] - dT*z_b[0]) / T_bar,
                (dalpha[1] - dT*z_b[1]) / T_bar,
                (dalpha[2] - dT*z_b[2]) / T_bar };
    const double ddT = dot3(dz, dalpha) + dot3(z_b, ddalpha);
    Vec3 ddz = { (ddalpha[0] - ddT*z_b[0] - 2.0*dT*dz[0]) / T_bar,
                 (ddalpha[1] - ddT*z_b[1] - 2.0*dT*dz[1]) / T_bar,
                 (ddalpha[2] - ddT*z_b[2] - 2.0*dT*dz[2]) / T_bar };

    const double cpsi = cos((double)fo.psi), spsi = sin((double)fo.psi);
    const Vec3 x_c = { cpsi, spsi, 0.0 };
    const Vec3 y_c = { -spsi, cpsi, 0.0 };

    Vec3 zxc; cross3(z_b, x_c, zxc);
    double n = norm3(zxc);
    Vec3 y_b;
    if (n < 1e-9) {
        // z_b ~ parallel to x_c (~90 deg pitch along heading): the yaw triad is
        // degenerate. Fall back to y_c to keep the frame (and quaternion)
        // finite -- mirrors indi_harness.outer_loop.attitude_from_thrust_dir.
        y_b[0] = y_c[0]; y_b[1] = y_c[1]; y_b[2] = y_c[2];
        n = 1e-9;
    } else {
        y_b[0] = zxc[0]/n; y_b[1] = zxc[1]/n; y_b[2] = zxc[2]/n;
    }
    Vec3 x_b; cross3(y_b, z_b, x_b);

    // R = column_stack([x_b, y_b, z_b]); R[row][col].
    const double R[3][3] = {
        { x_b[0], y_b[0], z_b[0] },
        { x_b[1], y_b[1], z_b[1] },
        { x_b[2], y_b[2], z_b[2] },
    };
    double q[4];
    q_from_R(R, q);

    const double w_x = -dot3(y_b, dz);
    const double w_y =  dot3(x_b, dz);
    const double w_z = (w_x * dot3(z_b, x_c) + (double)fo.dpsi * dot3(y_b, y_c)) / n;

    const double dw_x = -dot3(y_b, ddz) + w_y * w_z;
    const double dw_y =  dot3(x_b, ddz) - w_x * w_z;

    RefState r;
    r.q = Quaternion((float)q[0], (float)q[1], (float)q[2], (float)q[3]);
    r.w = Vector3f((float)w_x, (float)w_y, (float)w_z);
    // dw.z left 0: inter-tick central difference of w_z, filled in flight.
    r.dw = Vector3f((float)dw_x, (float)dw_y, 0.0f);
    r.T = (float)(m * T_bar);
    return r;
}

// --- Layer-B linear-INDI outer loop (Task B2) ------------------------------
// Port of indi_harness.outer_loop.OuterLoopINDI. The two INDI filters share a
// single cutoff so their group delays match (the accel/thrust analogue of the
// inner loop's phase-matching rule). Steady state is filter-family independent
// (both the firmware biquad and the Python Butter2 preserve DC), which is how
// the gtest bit-matches the Butter2 oracle after settle.

void AC_INDI_OuterLoop::configure(const Vector3f &kp, const Vector3f &kv,
                                  float cutoff_hz, float sample_freq,
                                  float m, float g)
{
    _kp = kp;
    _kv = kv;
    _m = m;
    _g = g;
    _f_accel.set_cutoff_frequency(sample_freq, cutoff_hz);
    _f_state.set_cutoff_frequency(sample_freq, cutoff_hz);
    reset();
}

void AC_INDI_OuterLoop::reset()
{
    _f_accel.reset();
    _f_state.reset();
}

AC_INDI_OuterLoop::OuterState
AC_INDI_OuterLoop::update(const Vector3f &p, const Vector3f &v, const Quaternion &q,
                          const Vector3f &f_b_meas, float T_state, const FlatOutput &fo)
{
    // position/velocity + flatness-accel command (element-wise gains).
    const Vector3f a_cmd(
        fo.a.x + _kp.x * (fo.p.x - p.x) + _kv.x * (fo.v.x - v.x),
        fo.a.y + _kp.y * (fo.p.y - p.y) + _kv.y * (fo.v.y - v.y),
        fo.a.z + _kp.z * (fo.p.z - p.z) + _kv.z * (fo.v.z - v.z));

    // measured specific force, body -> world, low-passed.
    const Vector3f f_meas_f = _f_accel.apply(q * f_b_meas);

    // thrust-vector state: current body -z scaled by T_state, body -> world,
    // low-passed (phase-matched with the accel filter).
    const Vector3f f_state_f = _f_state.apply(q * Vector3f(0.0f, 0.0f, -T_state));

    // INDI thrust-vector increment: replace the filtered measured accel with
    // the commanded accel about the filtered thrust-vector state. Drag never
    // enters a model -- it lives in f_meas.
    const Vector3f f_cmd = f_state_f + (a_cmd - Vector3f(0.0f, 0.0f, _g)) - f_meas_f;
    const float T_bar = f_cmd.length();

    OuterState os;
    os.z_b_des = -f_cmd / T_bar;
    os.T_cmd = _m * T_bar;
    return os;
}

Quaternion AC_INDI_OuterLoop::attitude_from_thrust_dir(const Vector3f &z_b_des, float psi)
{
    const double zb[3] = { z_b_des.x, z_b_des.y, z_b_des.z };
    const double cpsi = cos((double)psi), spsi = sin((double)psi);
    const Vec3 x_c = { cpsi, spsi, 0.0 };

    Vec3 y_b; cross3(zb, x_c, y_b);
    const double n = norm3(y_b);
    if (n < 1e-6) {
        // z_b_des ~ parallel to x_c (~90 deg pitch along heading): yaw triad is
        // degenerate; fall back to y_c to keep the quaternion finite.
        y_b[0] = -spsi; y_b[1] = cpsi; y_b[2] = 0.0;
    } else {
        y_b[0] /= n; y_b[1] /= n; y_b[2] /= n;
    }
    Vec3 x_b; cross3(y_b, zb, x_b);

    const double R[3][3] = {
        { x_b[0], y_b[0], zb[0] },
        { x_b[1], y_b[1], zb[1] },
        { x_b[2], y_b[2], zb[2] },
    };
    double q[4];
    q_from_R(R, q);
    return Quaternion((float)q[0], (float)q[1], (float)q[2], (float)q[3]);
}

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
