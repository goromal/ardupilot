#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include "AC_CustomControl_INDI.h"
#include <AP_Motors/AP_MotorsMulticopter.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_DDS/AP_DDS_config.h>
#if AP_DDS_ENABLED
#include <AP_DDS/AP_DDS_Client.h>
#endif
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

    // @Param: OUTER_EN
    // @DisplayName: INDI Layer-B outer loop enable
    // @Description: Enable the in-firmware INDI outer loop (design-doc S3 Layer B). 0 = stock guided position->attitude outer loop (C1 inner loop as-is). 1 = run the flatness + linear-INDI outer loop off a fresh DDS FlatSetpoint (rt/ap/flat_setpoint), replacing the stock attitude target with (q_ref, w_ff, dw_ff). Falls back to stock when the setpoint is stale/absent.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("OUTER_EN", 9, AC_CustomControl_INDI, _outer_en, 0),

    // @Param: B_KP_XY
    // @DisplayName: INDI outer-loop horizontal position P gain
    // @Description: Layer-B outer-loop position P gain for the horizontal (roll/pitch) axes: desired accel += kp*(p_ref - p).
    // @User: Advanced
    AP_GROUPINFO("B_KP_XY", 10, AC_CustomControl_INDI, _b_kp_xy, 6.0f),

    // @Param: B_KP_Z
    // @DisplayName: INDI outer-loop vertical position P gain
    // @Description: Layer-B outer-loop position P gain for the vertical (z) axis.
    // @User: Advanced
    AP_GROUPINFO("B_KP_Z", 11, AC_CustomControl_INDI, _b_kp_z, 6.0f),

    // @Param: B_KV_XY
    // @DisplayName: INDI outer-loop horizontal velocity P gain
    // @Description: Layer-B outer-loop velocity P gain for the horizontal (roll/pitch) axes: desired accel += kv*(v_ref - v).
    // @User: Advanced
    AP_GROUPINFO("B_KV_XY", 12, AC_CustomControl_INDI, _b_kv_xy, 4.0f),

    // @Param: B_KV_Z
    // @DisplayName: INDI outer-loop vertical velocity P gain
    // @Description: Layer-B outer-loop velocity P gain for the vertical (z) axis.
    // @User: Advanced
    AP_GROUPINFO("B_KV_Z", 13, AC_CustomControl_INDI, _b_kv_z, 4.0f),

    // @Param: B_ACC_FILT
    // @DisplayName: INDI outer-loop specific-force filter cutoff
    // @Description: Low-pass cutoff (Hz) shared by the outer loop's two phase-matched INDI filters (measured specific force + thrust-vector state). This sets the outer-loop feedback group delay -- the phase-margin knob. Too high and the thrust-vector INDI increment can become latency-unstable offboard; too low and it degenerates to PD+feedforward with no INDI action. Swept in S3 Layer-B bring-up (Task B5).
    // @Range: 2 40
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("B_ACC_FILT", 14, AC_CustomControl_INDI, _b_acc_filt, 8.0f),

    // @Param: B_DDS_TMO
    // @DisplayName: INDI outer-loop DDS setpoint timeout
    // @Description: Maximum age (ms) of the latest DDS FlatSetpoint before the outer loop treats it as stale and falls back to the stock guided target.
    // @Range: 50 1000
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("B_DDS_TMO", 15, AC_CustomControl_INDI, _b_dds_tmo, 200),

    // @Param: B_THR_EN
    // @DisplayName: INDI outer-loop collective thrust enable
    // @Description: When the Layer-B outer loop is active, drive the motor collective throttle from its thrust command (throttle = (T_bar/g)*hover_throttle) so thrust is coordinated with the commanded tilt. 0 = leave the collective to the stock/guided path (the outer loop then commands attitude only, which under-tracks because thrust is not coordinated with tilt).
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("B_THR_EN", 16, AC_CustomControl_INDI, _b_thr_en, 1),

    // @Param: USE_RPM
    // @DisplayName: INDI measured actuator state enable
    // @Description: 0=previous-command actuator estimate (Layer-A/C1). 1=measured actuator state reconstructed from bidi-eRPM (C2). Falls back to previous-command when RPM telemetry is unhealthy/stale.
    // @Values: 0:PrevCommand,1:MeasuredRPM
    // @User: Advanced
    AP_GROUPINFO("USE_RPM", 17, AC_CustomControl_INDI, _use_rpm, 0),

    // @Param: G2_YAW
    // @DisplayName: INDI rotor-inertia yaw-reaction coefficient
    // @Description: Normalized yaw correction subtracted from the measured actuator state: u_act.z -= G2_YAW * sum(d_i * OmegaDot_i). 0 disables. Only active with USE_RPM=1. Seed analytically (Ir-derived), confirm via sysid.
    // @User: Advanced
    AP_GROUPINFO("G2_YAW", 18, AC_CustomControl_INDI, _g2_yaw, 0.0f),

    // @Param: SIM_QNT
    // @DisplayName: INDI RPM shim quantization (SITL only)
    // @Description: SITL bidi-DShot shim eRPM quantization step (LSB). 0 disables quantization.
    // @User: Advanced
    AP_GROUPINFO("SIM_QNT", 19, AC_CustomControl_INDI, _sim_qnt, 0.0f),

    // @Param: SIM_DROP
    // @DisplayName: INDI RPM shim dropout probability (SITL only)
    // @Description: SITL bidi-DShot shim Bernoulli CRC-dropout probability per sample, in [0,1].
    // @User: Advanced
    AP_GROUPINFO("SIM_DROP", 20, AC_CustomControl_INDI, _sim_drop, 0.0f),

    // @Param: SIM_LAT
    // @DisplayName: INDI RPM shim latency (SITL only)
    // @Description: SITL bidi-DShot shim latency, in control-loop ticks.
    // @User: Advanced
    AP_GROUPINFO("SIM_LAT", 21, AC_CustomControl_INDI, _sim_lat, 0),

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

// --- Layer-C Task 4: measured actuator-state reconstruction (torque-space) -
// Recover the normalized roll/pitch/yaw actuator state (the SAME [-1,1] space
// as the stock mixer's get_roll/pitch/yaw) from the per-motor normalized rotor
// thrust omega2_norm[i] = Omega_i^2/Omega_max^2 (== the mixer's per-motor
// _thrust_rpyt_out[i]). The stock mixer's forward map is
//   thrust[i] = throttle + roll*rf[i] + pitch*pf[i] + yaw*yf[i]
// so each axis is recovered by projecting onto that axis's factor vector and
// normalizing by its squared norm:  axis = sum(f[i]*thrust[i]) / sum(f[i]^2).
// The factor vectors MUST be the mixer's own (rf/pf/yf passed in from
// AP_MotorsMatrix::get_roll_factor/get_pitch_factor), otherwise the recovered
// value is off by sum(f^2) and mis-scales the INDI operating point. (An earlier
// version hardcoded +/-0.5 without the /sum(f^2) normalization, which doubled
// the roll/pitch operating point and destabilized the loop.)
void AC_CustomControl_INDI::measured_actuator_torque(const float omega2_norm[4],
                                                     const float roll_f[4],
                                                     const float pitch_f[4],
                                                     const float yaw_f[4],
                                                     Vector3f &u_meas)
{
    float rx = 0, ry = 0, rz = 0, rf2 = 0, pf2 = 0, yf2 = 0;
    for (uint8_t i = 0; i < 4; i++) {
        rx += roll_f[i]  * omega2_norm[i];  rf2 += roll_f[i]  * roll_f[i];
        ry += pitch_f[i] * omega2_norm[i];  pf2 += pitch_f[i] * pitch_f[i];
        rz += yaw_f[i]   * omega2_norm[i];  yf2 += yaw_f[i]   * yaw_f[i];
    }
    u_meas.x = rf2 > 1e-6f ? rx / rf2 : 0.0f;
    u_meas.y = pf2 > 1e-6f ? ry / pf2 : 0.0f;
    u_meas.z = yf2 > 1e-6f ? rz / yf2 : 0.0f;
}

// --- Layer-C Task 5: G2 rotor-inertia yaw-reaction correction --------------
// -g2 * sum(d_i * Omega_dot_i), d=[+1,+1,-1,-1] (motor order [FR,BL,FL,BR],
// same spin convention as measured_actuator_torque's yaw_f*2).
float AC_CustomControl_INDI::g2_yaw_correction(const float omega_dot[4], float g2)
{
    static const float d[4] = { +1.0f, +1.0f, -1.0f, -1.0f };
    float s = 0.0f;
    for (uint8_t i = 0; i < 4; i++) {
        s += d[i] * omega_dot[i];
    }
    return -g2 * s;
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

        // aff_a/aff_b: shim FALLBACK affine model (omega_target = aff_a*thr + aff_b),
        // only used when telemetry drops (Task 8 dropout sub-case), NOT the clean path.
        // Seed aff_a so hover throttle maps ~hover rotor speed; aff_b=0. A rough
        // constant is fine here (calibrated at Task 8).
        const float aff_a = 490.0f / 0.3f;  // ~hover omega [rad/s] / hover throttle
        _rpm_shim.configure(_sim_qnt, _sim_drop, (uint8_t)_sim_lat, aff_a, 0.0f);

        // Task 5: per-motor Omega filter (filter-then-diff), same cutoff
        // (CC3_OMG_FILT) as the rate loop's estimator for consistency.
        for (uint8_t i = 0; i < 4; i++) {
            _f_omega[i].set_cutoff_frequency(1.0f / _dt, _omg_filt);
        }
    }

    // Outer loop -> desired body rate via the Task-2 tilt-prioritized reference.
    // Default: the stock guided position->attitude outer loop (C1 behaviour).
    // Layer B (CC3_OUTER_EN=1, fresh DDS FlatSetpoint): the in-firmware INDI
    // outer loop supplies (q_ref, w_ff, dw_ff) instead. dw_ff is passed to the
    // rate loop's angular-accel feedforward (5th step() arg).
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);

    bool outer_active = false;
    Vector3f w_ff, dw_ff;
    Vector3f log_ref_p, log_meas_p;   // INDB: reference vs measured position
    float log_tcmd = 0.0f;            // INDB: outer-loop collective thrust cmd
    // Clear the Layer-B attitude override each loop; set only if the outer
    // loop runs below (consumed by Copter::update_flight_mode this same loop).
    _ovr_valid = false;
#if AP_DDS_ENABLED
    AP_DDS_Client *dds = AP_DDS_Client::get_singleton();
    AP_DDS_Client::FlatRef ref;
    if (_outer_en && dds != nullptr &&
        dds->get_flat_setpoint(ref, (uint32_t)_b_dds_tmo.get() * 1000U)) {
        // one-shot configure (params are valid only after EEPROM load).
        if (!_outer_configured) {
            _outer.configure(Vector3f(_b_kp_xy, _b_kp_xy, _b_kp_z),
                             Vector3f(_b_kv_xy, _b_kv_xy, _b_kv_z),
                             _b_acc_filt, 1.0f / _dt, /*m*/ 1.0f, GRAVITY_MSS);
            _outer_configured = true;
            _have_prev_wz = false;
        }
        Vector3f p, v;
        if (_ahrs->get_relative_position_NED_origin_float(p) &&
            _ahrs->get_velocity_NED(v)) {
            // Body specific force, via the AHRS earth-frame accel (= R*body
            // specific force) rotated back to body so update() re-rotates it
            // consistently with attitude_body.
            const Vector3f f_b = attitude_body.inverse() * _ahrs->get_accel_ef();
            // Thrust-state estimate for the INDI increment. We ground it in the
            // flatness feedforward specific thrust |g*e3 - a_ref| (a function of
            // the reference ONLY), NOT the commanded throttle. Deriving T_state
            // from throttle closes a self-referential loop once we drive the
            // collective (throttle -> T_state -> f_state -> z_b_des/f_cmd ->
            // throttle) that winds up (observed altitude runaway). With T_state
            // grounded in the reference, z_b_des stays stable and the INDI
            // increment keeps its measured-accel disturbance-rejection term.
            const float T_state = (Vector3f(0.0f, 0.0f, GRAVITY_MSS) - ref.a).length();

            const AC_INDI_OuterLoop::FlatOutput fo{
                ref.p, ref.v, ref.a, ref.j, ref.s, ref.psi, ref.dpsi, ref.ddpsi};
            const AC_INDI_OuterLoop::OuterState os =
                _outer.update(p, v, attitude_body, f_b, T_state, fo);
            AC_INDI_OuterLoop::RefState rs =
                AC_INDI_OuterLoop::flat_reference(fo, /*m*/ 1.0f, GRAVITY_MSS);

            // Inter-tick dw_z: finite-difference the closed-form w_z (rs.w.z)
            // against the previous tick. The oracle's analytic central diff
            // needs the trajectory; successive dense DDS setpoints approximate
            // it. First tick after (re)engage: dw_z = 0.
            if (_have_prev_wz) {
                rs.dw.z = (rs.w.z - _prev_wz) / _dt;
            }
            _prev_wz = rs.w.z;
            _have_prev_wz = true;

            attitude_target = AC_INDI_OuterLoop::attitude_from_thrust_dir(os.z_b_des, ref.psi);
            // Rotate the reference body-rate / ang-accel FF into the current
            // body frame (the stock path rotates the target ang vel likewise).
            const Quaternion rot_ref_to_body = attitude_body.inverse() * attitude_target;
            w_ff = rot_ref_to_body * rs.w;
            dw_ff = rot_ref_to_body * rs.dw;
            // Collective thrust: the geometric-controller collective -- the
            // desired specific force (a_cmd - g*e3) PROJECTED onto the thrust
            // axis (-z_b_des), clamped >= 0. Projection (not magnitude) is
            // essential: a rotor can only push up, so a command to descend hard
            // (a_cmd.z large +down) must drive thrust toward 0 (free-fall), not
            // toward max -- taking |a_cmd - g*e3| flips the sign and drove an
            // altitude runaway. a_cmd depends only on measured p/v, so this is
            // stable (unlike the INDI increment os.T_cmd, which self-references
            // through throttle and winds up). throttle = (spec_thrust/g)*hover.
            //
            // Overrides the _throttle_in the flight mode set; run_custom_
            // controller runs just before motors_output_main, so this is the
            // last write. Left to the stock/guided path when B_THR_EN=0.
            if (_b_thr_en) {
                const float hover = _motors->get_throttle_hover();
                const Vector3f f_des = os.a_cmd - Vector3f(0.0f, 0.0f, GRAVITY_MSS);
                const float spec_thrust = MAX(0.0f, -(f_des * os.z_b_des));
                const float thr = (spec_thrust / GRAVITY_MSS) * hover;
                _motors->set_throttle(constrain_float(thr, 0.0f, 1.0f));
            }
            log_ref_p = ref.p;
            log_meas_p = p;
            log_tcmd = os.T_cmd;
            outer_active = true;
            // Publish the flatness target so Copter::update_flight_mode drives
            // it into AC_AttitudeControl this loop: the stock rate controller
            // then tracks the SAME target the INDI increment refines (otherwise
            // the guided target and the increment oppose -> ~0 roll/pitch
            // torque). rs.w is the reference body-rate feedforward.
            _ovr_q_ref = attitude_target;
            _ovr_w_ff = rs.w;
            _ovr_valid = true;
        }
    }
#endif // AP_DDS_ENABLED

    if (!outer_active) {
        // Stock guided position->attitude outer loop (C1 behaviour, unchanged):
        // read the stock attitude target + target angular-velocity feedforward.
        attitude_target = _att_control->get_attitude_target_quat();
        const Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
        w_ff = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();
        dw_ff.zero();
        _have_prev_wz = false;
    }

    const Vector3f w_des = attitude_rate_ref(attitude_body, attitude_target,
                                             _kp_tilt, _kp_yaw, w_ff);

    // Inner loop: INDI rate law. Actuator state is the stock mixer's current
    // torque command (_motors->get_roll/pitch/yaw); outputs are torque-like in
    // the mixer's [-1, 1] range (S3 Layer-A accepts the stock-mixer limitation).
    const Vector3f gyro = _ahrs->get_gyro_latest();
    Vector3f u_act(_motors->get_roll(), _motors->get_pitch(), _motors->get_yaw());
    // Layer-C Task 4: measured actuator state (CC3_USE_RPM=1). Reconstruct
    // u_act from per-motor rotor speed instead of the previous mixer command
    // -- the shipped previous-command estimate is wrong under actuator lag,
    // which is the limit-cycle root cause this task fixes. Falls back to the
    // previous-command u_act (above) when RPM telemetry is unhealthy/stale.
    // The shim's get() MUST be called exactly once per motor per tick (it
    // advances a latency ring + PRNG).
    _rpm_fallback = false;
    float omega_log[4] {};   // INDC: measured rotor speed, per motor (0 on non-RPM/fallback path)
    float odot_log[4] {};    // INDC: measured rotor angular accel, per motor (ditto)
    if (_use_rpm) {
        float o2n[4];
        bool healthy = true;
        for (uint8_t i = 0; i < 4; i++) {
            float erpm_esc;
            if (AP::esc_telem().get_rpm(i, erpm_esc)) {
                _rpm_shim.set_truth(i, erpm_esc);
            }
            _rpm_shim.set_throttle(i, _motors->get_throttle());   // collective 0..1 (base class)
            float omega;
            const bool ok = _rpm_shim.get(i, omega) && _rpm_shim.healthy(i);
            if (!ok) {
                healthy = false;
            }
            o2n[i] = constrain_float((omega * omega) / _omega2_max, 0.0f, 1.0f);

            // Task 5: Omega_dot_meas, filter-then-diff, computed in this SAME
            // single get() pass (a second shim.get() loop would double-advance
            // the shim's latency ring + PRNG).
            omega_log[i] = omega;
            const float of = _f_omega[i].apply(omega);
            odot_log[i] = _have_prev_omega ? (of - _prev_omega_f[i]) / _dt : 0.0f;
            _prev_omega_f[i] = of;
        }
        _have_prev_omega = true;
        if (healthy) {
            // Gather the stock mixer's own per-motor factors so the recovered
            // actuator state lands in the SAME normalized space as get_roll/
            // pitch/yaw. yaw has no accessor; use the normalized quad-X pattern.
            float roll_f[4], pitch_f[4];
            static const float yaw_f[4] = { +0.5f, +0.5f, -0.5f, -0.5f };
            for (uint8_t i = 0; i < 4; i++) {
                roll_f[i]  = _motors->get_roll_factor(i);
                pitch_f[i] = _motors->get_pitch_factor(i);
            }
            Vector3f u_meas;
            measured_actuator_torque(o2n, roll_f, pitch_f, yaw_f, u_meas);
            u_meas.z += g2_yaw_correction(odot_log, _g2_yaw);
            u_act = u_meas;
        } else {
            _rpm_fallback = true;   // keep previous-command u_act
        }
    }
    Vector3f domega_pred, domega_filt, u_filt;
    bool sat;
    const Vector3f u_cmd = _rate_loop.step(_dt, gyro, u_act, w_des, dw_ff, 1.0f,
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

    // Layer-C Task 5: per-motor measured-RPM health (design-doc L). Per-motor
    // Omega and Omega_dot (filter-then-diff, zero on the non-RPM path since
    // the whole CC3_USE_RPM block is skipped), the reconstructed u_act this
    // tick (previous-command estimate on the non-RPM/fallback path), and the
    // fallback flag. Read back by indi_harness read_indc_health().
    // @LoggerMessage: INDC
    // @Description: INDI Layer-C C2 measured-RPM health
    // @Field: TimeUS: Time since system startup
    // @Field: O0: measured rotor speed 0 [rad/s]
    // @Field: O1: measured rotor speed 1 [rad/s]
    // @Field: O2: measured rotor speed 2 [rad/s]
    // @Field: O3: measured rotor speed 3 [rad/s]
    // @Field: D0: measured rotor angular accel 0 [rad/s^2]
    // @Field: D1: measured rotor angular accel 1 [rad/s^2]
    // @Field: D2: measured rotor angular accel 2 [rad/s^2]
    // @Field: D3: measured rotor angular accel 3 [rad/s^2]
    // @Field: Ux: reconstructed normalized actuator roll
    // @Field: Uy: reconstructed normalized actuator pitch
    // @Field: Uz: reconstructed normalized actuator yaw (incl. G2)
    // @Field: FB: RPM fallback flag (1 = previous-command fallback)
    AP::logger().Write(
        "INDC", "TimeUS,O0,O1,O2,O3,D0,D1,D2,D3,Ux,Uy,Uz,FB",
        "Qfffffffffffi",
        AP_HAL::micros64(),
        omega_log[0], omega_log[1], omega_log[2], omega_log[3],
        odot_log[0], odot_log[1], odot_log[2], odot_log[3],
        u_act.x, u_act.y, u_act.z, (int32_t)_rpm_fallback);

    // Layer-B outer-loop health (design-doc L). Reference vs measured position,
    // the collective thrust command, and the fallback flag (1 = stock outer
    // loop this tick; the DDS ref was disabled/stale/absent). Read back by
    // indi_harness read_outer_health().
    // @LoggerMessage: INDB
    // @Description: Layer-B INDI outer-loop health
    // @Field: TimeUS: Time since system startup
    // @Field: RPx: reference position north
    // @Field: RPy: reference position east
    // @Field: RPz: reference position down
    // @Field: Px: measured position north
    // @Field: Py: measured position east
    // @Field: Pz: measured position down
    // @Field: Tc: outer-loop collective thrust command
    // @Field: FB: fallback flag (1 if the stock outer loop ran this tick)
    AP::logger().Write(
        "INDB",
        "TimeUS,RPx,RPy,RPz,Px,Py,Pz,Tc,FB",
        "Qfffffffi",
        AP_HAL::micros64(),
        log_ref_p.x, log_ref_p.y, log_ref_p.z,
        log_meas_p.x, log_meas_p.y, log_meas_p.z,
        log_tcmd,
        (int32_t)(!outer_active));
#endif

    return u_cmd;
}

void AC_CustomControl_INDI::reset(void)
{
    _rate_loop.reset();
    _rate_loop_configured = false;
    for (uint8_t i = 0; i < 4; i++) {
        _f_omega[i].reset();
    }
    _have_prev_omega = false;
}

bool AC_CustomControl_INDI::get_attitude_override(Quaternion &q_ref, Vector3f &ang_vel_body) const
{
    if (!_ovr_valid) {
        return false;
    }
    q_ref = _ovr_q_ref;
    ang_vel_body = _ovr_w_ff;
    return true;
}

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
