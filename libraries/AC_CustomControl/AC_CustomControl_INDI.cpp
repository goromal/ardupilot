#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include "AC_CustomControl_INDI.h"
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Logger/AP_Logger.h>
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
    // @Description: G1 diagonal control-effectiveness term for the roll and pitch axes (rad/s^2 per unit actuator increment). With a low-delay angular-accel estimate (OMG_FILT ~80 Hz) the rate loop is stable near the true effectiveness, so this is set to 500 (roll/pitch track cleanly, RATE-gyro buzz < 5 deg/s). The earlier Layer-A value of 1000 was a workaround for the higher-delay estimate (OMG_FILT ~30-40), which limit-cycled at true-effectiveness gains -- raise OMG_FILT before lowering this further. Tune per airframe.
    // @User: Advanced
    AP_GROUPINFO("G1_RP", 4, AC_CustomControl_INDI, _g1_rp, 500.0f),

    // @Param: G1_YAW
    // @DisplayName: INDI yaw control effectiveness
    // @Description: G1 diagonal control-effectiveness term for the yaw axis (rad/s^2 per unit actuator increment). See G1_RP.
    // @User: Advanced
    AP_GROUPINFO("G1_YAW", 5, AC_CustomControl_INDI, _g1_yaw, 1000.0f),

    // @Param: KW_RP
    // @DisplayName: INDI roll/pitch rate-error gain
    // @Description: Rate-error to desired-angular-acceleration gain kw for the roll and pitch axes (1/s).
    // @User: Standard
    AP_GROUPINFO("KW_RP", 6, AC_CustomControl_INDI, _kw_rp, 20.0f),

    // @Param: KW_YAW
    // @DisplayName: INDI yaw rate-error gain
    // @Description: Rate-error to desired-angular-acceleration gain kw for the yaw axis (1/s).
    // @User: Standard
    AP_GROUPINFO("KW_YAW", 7, AC_CustomControl_INDI, _kw_yaw, 10.0f),

    // @Param: OMG_FILT
    // @DisplayName: INDI angular-accel estimator pre-filter cutoff
    // @Description: Angular-accel estimator pre-filter cutoff; >0 selects filter-then-differentiate (cleaner ang-accel estimate). 0 = legacy differentiate-then-filter. The cutoff sets the feedback group delay: too low (<=40) and the INDI rate loop limit-cycles at true-effectiveness gains (delay-driven instability); ~80 Hz suppresses the roll/pitch limit cycle and lets the loop track. Default 80.
    // @Range: 0 120
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("OMG_FILT", 8, AC_CustomControl_INDI, _omg_filt, 80.0f),

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

// --- Layer-A INDI rate loop (Task 3) ---------------------------------------

void AC_INDI_RateLoop::configure(float cutoff_hz, float sample_freq,
                                 const Vector3f &kw, const Vector3f &g1)
{
    // One cutoff, both filters -> matched group delay (phase-matching rule).
    configure_split(cutoff_hz, cutoff_hz, sample_freq, kw, g1);
}

void AC_INDI_RateLoop::configure_split(float cutoff_domega_hz, float cutoff_uact_hz,
                                       float sample_freq, const Vector3f &kw,
                                       const Vector3f &g1)
{
    _f_domega.set_cutoff_frequency(sample_freq, cutoff_domega_hz);
    _f_uact.set_cutoff_frequency(sample_freq, cutoff_uact_hz);
    _kw = kw;
    _g1_inv = Vector3f(is_zero(g1.x) ? 0.0f : 1.0f / g1.x,
                       is_zero(g1.y) ? 0.0f : 1.0f / g1.y,
                       is_zero(g1.z) ? 0.0f : 1.0f / g1.z);
    reset();
}

void AC_INDI_RateLoop::reset()
{
    _f_domega.reset();
    _f_uact.reset();
    _f_gyro.reset();
    _have_prev = false;
}

// C1: select the angular-accel estimator. cutoff_hz > 0 switches step() to
// filter-then-differentiate and re-points the actuator-state filter to the
// SAME cutoff as the gyro pre-filter (the phase-matching rule -- see the
// class comment). cutoff_hz == 0 leaves the legacy diff-then-filter filters
// (built by configure()/configure_split()) untouched and active.
void AC_INDI_RateLoop::configure_estimator(float cutoff_hz, float sample_freq)
{
    _use_ftd = cutoff_hz > 0.0f;
    if (_use_ftd) {
        _f_gyro.set_cutoff_frequency(sample_freq, cutoff_hz);
        _f_uact.set_cutoff_frequency(sample_freq, cutoff_hz);
        _f_gyro.reset();
        _f_uact.reset();
        _have_prev = false;  // re-init _prev_gyro_f on the next settled sample
    }
}

Vector3f AC_INDI_RateLoop::step(float dt, const Vector3f &gyro, const Vector3f &u_act,
                                const Vector3f &w_des, const Vector3f &dw_ff, float u_limit,
                                Vector3f &domega_pred, Vector3f &domega_filt,
                                Vector3f &u_filt, bool &sat)
{
    // (1) angular-accel estimate.
    if (_use_ftd) {
        // C1: filter-then-differentiate -- filter the gyro first, then
        // difference the filtered signal. Cleaner than differentiating the
        // raw (noisy) gyro and filtering afterwards.
        const Vector3f gyro_f = _f_gyro.apply(gyro);
        if (!_have_prev) {
            _prev_gyro_f = gyro_f;
            _have_prev = true;
        }
        domega_filt = (gyro_f - _prev_gyro_f) / dt;
        _prev_gyro_f = gyro_f;
    } else {
        // Legacy: differentiate the raw gyro, then filter the derivative.
        if (!_have_prev) {
            _prev_gyro = gyro;
            _have_prev = true;
        }
        const Vector3f domega_raw = (gyro - _prev_gyro) / dt;
        _prev_gyro = gyro;
        domega_filt = _f_domega.apply(domega_raw);
    }

    // (2) actuator-state estimate: measured actuator torque through the SAME
    // filter config (phase-matched with the angular-accel estimate).
    u_filt = _f_uact.apply(u_act);

    // (3) rate error -> desired angular acceleration.
    domega_pred = Vector3f(_kw.x * (w_des.x - gyro.x) + dw_ff.x,
                           _kw.y * (w_des.y - gyro.y) + dw_ff.y,
                           _kw.z * (w_des.z - gyro.z) + dw_ff.z);

    // (4) INDI inversion: u = u_filt + G1^-1 (dw_cmd - domega_filt).
    Vector3f u_cmd(u_filt.x + (domega_pred.x - domega_filt.x) * _g1_inv.x,
                   u_filt.y + (domega_pred.y - domega_filt.y) * _g1_inv.y,
                   u_filt.z + (domega_pred.z - domega_filt.z) * _g1_inv.z);

    // (5) saturation: prioritized shed-yaw (drop the yaw increment first, keep
    // tilt), then clip all axes -- flagged, never silently wrapped.
    sat = (fabsf(u_cmd.x) > u_limit) || (fabsf(u_cmd.y) > u_limit) ||
          (fabsf(u_cmd.z) > u_limit);
    if (sat) {
        u_cmd.z = u_filt.z;
        u_cmd.x = constrain_float(u_cmd.x, -u_limit, u_limit);
        u_cmd.y = constrain_float(u_cmd.y, -u_limit, u_limit);
        u_cmd.z = constrain_float(u_cmd.z, -u_limit, u_limit);
    }
    return u_cmd;
}

Vector3f AC_CustomControl_INDI::update(void)
{
    // Reset filter/loop state while on the ground to avoid build-up, matching
    // the PID backend's spool-state handling.
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            reset();
            break;
        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            break;
    }

    // Params are only valid after load_object_from_eeprom (post-construction),
    // so configure the rate loop on first run.
    if (!_rate_loop_configured) {
        _rate_loop.configure(_filt_hz, 1.0f / _dt,
                             Vector3f(_kw_rp, _kw_rp, _kw_yaw),
                             Vector3f(_g1_rp, _g1_rp, _g1_yaw));
        _rate_loop.configure_estimator(_omg_filt, 1.0f / _dt);
        _rate_loop_configured = true;
    }

    // Outer loop (unchanged stock guided position->attitude): read the stock
    // attitude target and turn it into a desired body rate via the Task-2
    // tilt-prioritized reference, including the target angular-velocity
    // feedforward (rotated into the body frame, as the PID backend does).
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_target = _att_control->get_attitude_target_quat();
    const Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    const Vector3f w_ff = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();
    const Vector3f w_des = attitude_rate_ref(attitude_body, attitude_target,
                                             _kp_tilt, _kp_yaw, w_ff);

    // Inner loop: INDI rate law. Actuator state is the stock mixer's current
    // torque command (_motors->get_roll/pitch/yaw); outputs are torque-like in
    // the mixer's [-1, 1] range (S3 Layer-A accepts the stock-mixer limitation).
    const Vector3f gyro = _ahrs->get_gyro_latest();
    const Vector3f u_act(_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw());
    Vector3f domega_pred, domega_filt, u_filt;
    bool sat;
    const Vector3f u_cmd = _rate_loop.step(_dt, gyro, u_act, w_des, Vector3f(), 1.0f,
                                           domega_pred, domega_filt, u_filt, sat);

#if HAL_LOGGING_ENABLED
    // INDI health to the .BIN (design-doc L: the .BIN is source of truth).
    // Predicted vs measured/filtered angular accel is the tell for filter/G1
    // mismatch (they should track); the actuator-state estimate and the INDI
    // increment Du = u_cmd - u_filt plus the saturation flag round out the
    // per-loop health picture. Read back by indi_harness read_indi_health().
    // @LoggerMessage: INDI
    // @Description: Layer-A INDI attitude/rate backend health
    // @Field: TimeUS: Time since system startup
    // @Field: Px: predicted angular accel roll
    // @Field: Py: predicted angular accel pitch
    // @Field: Pz: predicted angular accel yaw
    // @Field: Mx: measured (filtered) angular accel roll
    // @Field: My: measured (filtered) angular accel pitch
    // @Field: Mz: measured (filtered) angular accel yaw
    // @Field: Ax: filtered actuator-state estimate roll
    // @Field: Ay: filtered actuator-state estimate pitch
    // @Field: Az: filtered actuator-state estimate yaw
    // @Field: Dx: INDI torque increment roll
    // @Field: Dy: INDI torque increment pitch
    // @Field: Dz: INDI torque increment yaw
    // @Field: S: saturation flag (1 if any axis saturated)
    AP::logger().Write(
        "INDI",
        "TimeUS,Px,Py,Pz,Mx,My,Mz,Ax,Ay,Az,Dx,Dy,Dz,S",
        "Qffffffffffffi",
        AP_HAL::micros64(),
        domega_pred.x, domega_pred.y, domega_pred.z,
        domega_filt.x, domega_filt.y, domega_filt.z,
        u_filt.x, u_filt.y, u_filt.z,
        u_cmd.x - u_filt.x, u_cmd.y - u_filt.y, u_cmd.z - u_filt.z,
        (int32_t)sat);
#endif

    return u_cmd;
}

void AC_CustomControl_INDI::reset(void)
{
    _rate_loop.reset();
    _rate_loop_configured = false;
}

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
