// Unit tests for the Layer-A INDI math (design-doc S3), validated against the
// tested S0 Python reference (indi_harness) used as the oracle. Reference
// vectors were dumped from indi_harness.tilt_yaw.attitude_rate_ref /
// indi_harness.inner_loop.InnerLoopINDI at build time and hard-coded here.
#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>
#include <AC_CustomControl/AC_CustomControl_INDI.h>
#include <AC_CustomControl/AC_CustomControl_OuterLoop.h>
#include <AC_CustomControl/AP_INDI_RpmSource.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_CUSTOMCONTROL_INDI_ENABLED

// ---- Task 2: tilt-prioritized attitude -> desired body rate ---------------
// Oracle: indi_harness.tilt_yaw.attitude_rate_ref (scalar-first [w,x,y,z]).

static void check_rate_ref(const char *name,
                           const float q[4], const float qr[4],
                           float kp_tilt, float kp_yaw, const float wff[3],
                           const float expect[3])
{
    const Quaternion Q(q[0], q[1], q[2], q[3]);
    const Quaternion QR(qr[0], qr[1], qr[2], qr[3]);
    const Vector3f w = AC_CustomControl_INDI::attitude_rate_ref(
        Q, QR, kp_tilt, kp_yaw, Vector3f(wff[0], wff[1], wff[2]));
    EXPECT_NEAR(w.x, expect[0], 1e-5f) << name << " x";
    EXPECT_NEAR(w.y, expect[1], 1e-5f) << name << " y";
    EXPECT_NEAR(w.z, expect[2], 1e-5f) << name << " z";
}

TEST(AC_CustomControl_INDI, tilt_yaw_ref)
{
    // level attitude, zero error, pure feedforward -> returns w_ff unchanged
    {
        const float q[4]  = {0.9690874237046979f, 0.1076763804116331f,
                             0.2153527608232662f, 0.05383819020581655f};
        const float wff[3] = {0.1f, -0.2f, 0.3f};
        const float exp[3] = {0.1f, -0.2f, 0.3f};
        check_rate_ref("level_zero_err_ff", q, q, 6.0f, 3.0f, wff, exp);
    }
    // pure 30 deg yaw error -> uses yaw gain only (3.0 * 0.5236 = 1.5708)
    {
        const float q[4]  = {1.0f, 0.0f, 0.0f, 0.0f};
        const float qr[4] = {0.9659258262890683f, 0.0f, 0.0f, 0.25881904510252074f};
        const float wff[3] = {0.0f, 0.0f, 0.0f};
        const float exp[3] = {0.0f, 0.0f, 1.5707963267948966f};
        check_rate_ref("pure_yaw_30", q, qr, 6.0f, 3.0f, wff, exp);
    }
    // pure 45 deg roll error -> uses tilt gain only (6.0 * 0.7854 = 4.7124)
    {
        const float q[4]  = {1.0f, 0.0f, 0.0f, 0.0f};
        const float qr[4] = {0.9238795325112867f, 0.3826834323650898f, 0.0f, 0.0f};
        const float wff[3] = {0.0f, 0.0f, 0.0f};
        const float exp[3] = {4.712388980384691f, 0.0f, 0.0f};
        check_rate_ref("pure_roll_45", q, qr, 6.0f, 3.0f, wff, exp);
    }
    // 170 deg tilt with scalar-NEGATIVE error quaternion -> must flip (no
    // unwinding); full-range qlog gives 6.0 * 2.9670 = 17.8024 rad, not the
    // short-arc complement.
    {
        const float q[4]  = {1.0f, 0.0f, 0.0f, 0.0f};
        const float qr[4] = {-0.08715574274765814f, -0.9961946980917455f, 0.0f, 0.0f};
        const float wff[3] = {0.0f, 0.0f, 0.0f};
        const float exp[3] = {17.802358370342162f, 0.0f, 0.0f};
        check_rate_ref("tilt_170_flip", q, qr, 6.0f, 3.0f, wff, exp);
    }
    // general mixed attitude + mixed error + feedforward
    {
        const float q[4]  = {0.8922685978385126f, 0.20994555243259122f,
                             -0.36740471675703457f, 0.1574591643244434f};
        const float qr[4] = {0.8235294117647058f, -0.11764705882352942f,
                             0.29411764705882354f, 0.4705882352941177f};
        const float wff[3] = {0.05f, 0.1f, -0.05f};
        const float exp[3] = {-4.032375340976597f, 8.179848894820342f, 2.242479639618012f};
        check_rate_ref("mixed_general", q, qr, 6.0f, 3.0f, wff, exp);
    }
}

// yaw is de-prioritized versus tilt: an equal-angle error about z produces a
// smaller rate command than about x, with the ratio equal to kp_yaw/kp_tilt.
TEST(AC_CustomControl_INDI, yaw_deprioritized)
{
    const Quaternion level(1.0f, 0.0f, 0.0f, 0.0f);
    const float ang = 0.3f;
    Quaternion roll_ref; roll_ref.from_axis_angle(Vector3f(ang, 0, 0));
    Quaternion yaw_ref;  yaw_ref.from_axis_angle(Vector3f(0, 0, ang));
    const Vector3f wr = AC_CustomControl_INDI::attitude_rate_ref(
        level, roll_ref, 6.0f, 3.0f, Vector3f());
    const Vector3f wy = AC_CustomControl_INDI::attitude_rate_ref(
        level, yaw_ref, 6.0f, 3.0f, Vector3f());
    EXPECT_NEAR(wr.x, 6.0f * ang, 1e-5f);
    EXPECT_NEAR(wy.z, 3.0f * ang, 1e-5f);
    EXPECT_LT(wy.z, wr.x);  // yaw correction weaker than tilt at equal error
}

// ---- Task 3: INDI rate loop (angular-accel + phase-matched actuator filter +
// diagonal-G1 inversion) --------------------------------------------------
// Oracle: /tmp/s3a_dump_inner_oracle.py — a Layer-A reduction of
// indi_harness.inner_loop.InnerLoopINDI stepped against a toy first-order-motor
// plant (mirroring tests/test_inner_loop.py), with a Python replica of
// ArduPilot's DigitalBiquadFilter so the trajectories match to < 1e-3.

namespace {
struct ToyOut { Vector3f u[250]; Vector3f w[250]; };

// Toy first-order-motor plant: the loop commands actuator torque u_cmd; the
// motor lags toward it (tau); the plant produces angular accel = G1 * u_act.
void run_toy(AC_INDI_RateLoop &loop, float tau, const Vector3f &g1,
             const Vector3f &wd, int n, ToyOut &out)
{
    const float dt = 1.0f / 500.0f;
    Vector3f om, u_act;
    for (int k = 0; k < n; k++) {
        Vector3f dwp, dwf, uf;
        bool sat;
        const Vector3f uc = loop.step(dt, om, u_act, wd, Vector3f(), 1e9f,
                                      dwp, dwf, uf, sat);
        u_act += (uc - u_act) * (dt / tau);
        om += Vector3f(g1.x * u_act.x, g1.y * u_act.y, g1.z * u_act.z) * dt;
        out.u[k] = uc;
        out.w[k] = om;
    }
}
struct Sample { int k; float v[3]; };
} // namespace

TEST(AC_CustomControl_INDI, indi_rate_step_matches_oracle)
{
    AC_INDI_RateLoop loop;
    loop.configure(25.0f, 500.0f, Vector3f(20, 20, 10), Vector3f(800, 800, 300));
    static ToyOut out;
    run_toy(loop, 0.02f, Vector3f(800, 800, 300), Vector3f(0.5f, -0.3f, 0.2f), 250, out);

    const Sample u_ref[] = {
        {  0, {0.012500000f, -0.007500000f, 0.006666667f}},
        { 25, {0.005580700f, -0.003348420f, 0.004642071f}},
        { 50, {0.000844823f, -0.000506894f, 0.002546248f}},
        { 75, {-0.000136898f, 0.000082139f, 0.001325922f}},
        {100, {-0.000103387f, 0.000062032f, 0.000681197f}},
        {125, {-0.000022448f, 0.000013469f, 0.000348691f}},
        {150, {0.000000414f, -0.000000249f, 0.000178309f}},
        {175, {0.000001741f, -0.000001045f, 0.000091157f}},
        {200, {0.000000514f, -0.000000308f, 0.000046598f}},
        {225, {0.000000035f, -0.000000021f, 0.000023820f}},
    };
    for (const auto &s : u_ref) {
        EXPECT_NEAR(out.u[s.k].x, s.v[0], 1e-3f) << "u k=" << s.k << " x";
        EXPECT_NEAR(out.u[s.k].y, s.v[1], 1e-3f) << "u k=" << s.k << " y";
        EXPECT_NEAR(out.u[s.k].z, s.v[2], 1e-3f) << "u k=" << s.k << " z";
    }
    const Sample w_ref[] = {
        {  0, {0.002000000f, -0.001200000f, 0.000400000f}},
        { 25, {0.288553601f, -0.173132160f, 0.063738461f}},
        { 50, {0.469741489f, -0.281844894f, 0.125539086f}},
        { 75, {0.505733522f, -0.303440113f, 0.161262288f}},
        {100, {0.503962157f, -0.302377294f, 0.180103433f}},
        {125, {0.500825307f, -0.300495184f, 0.189816057f}},
        {150, {0.499973204f, -0.299983922f, 0.194792353f}},
        {175, {0.499932386f, -0.299959432f, 0.197337719f}},
        {200, {0.499980816f, -0.299988489f, 0.198639072f}},
        {225, {0.499998865f, -0.299999319f, 0.199304322f}},
    };
    for (const auto &s : w_ref) {
        EXPECT_NEAR(out.w[s.k].x, s.v[0], 1e-3f) << "w k=" << s.k << " x";
        EXPECT_NEAR(out.w[s.k].y, s.v[1], 1e-3f) << "w k=" << s.k << " y";
        EXPECT_NEAR(out.w[s.k].z, s.v[2], 1e-3f) << "w k=" << s.k << " z";
    }
}

// The S2 synchronization lesson as a firmware regression: the angular-accel
// filter and the actuator-state filter MUST share one cutoff (matched group
// delay). Structural: the production configure() builds both filters with the
// same cutoff. Behavioural: a deliberately mismatched loop tracks far worse --
// mismatched steady-state rate error is many times the matched error.
TEST(AC_CustomControl_INDI, indi_phase_match)
{
    const Vector3f g1(800, 800, 300), kw(20, 20, 10), wd(0.5f, -0.3f, 0.2f);

    AC_INDI_RateLoop matched;
    matched.configure(25.0f, 500.0f, kw, g1);
    EXPECT_FLOAT_EQ(matched.domega_cutoff(), matched.uact_cutoff());  // structural

    AC_INDI_RateLoop mismatched;
    mismatched.configure_split(25.0f, 2.0f, 500.0f, kw, g1);
    EXPECT_GT(fabsf(mismatched.domega_cutoff() - mismatched.uact_cutoff()), 1.0f);

    static ToyOut om, ox;
    run_toy(matched, 0.02f, g1, wd, 250, om);
    run_toy(mismatched, 0.02f, g1, wd, 250, ox);

    float em = 0.0f, ex = 0.0f;
    for (int k = 230; k < 250; k++) {
        em += (om.w[k] - wd).length();
        ex += (ox.w[k] - wd).length();
    }
    em /= 20.0f;
    ex /= 20.0f;
    EXPECT_LT(em, 1e-3f);           // matched: tight rate tracking
    EXPECT_GT(ex, 10.0f * em);      // mismatched: error grows (>10x), the S2 tell
}

// update() returns the INDI increment, and saturation is flagged with
// prioritized yaw-shed (never silently wrapped).
TEST(AC_CustomControl_INDI, indi_saturation_sheds_yaw)
{
    AC_INDI_RateLoop loop;
    loop.configure(25.0f, 500.0f, Vector3f(20, 20, 10), Vector3f(800, 800, 300));
    Vector3f dwp, dwf, uf;
    bool sat;
    // gyro=0, u_act=0, large desired rate, tight limit: raw u would be
    // (0.125, -0.1, 0.1); shed yaw -> 0, clip roll/pitch -> (+0.1, -0.1, 0).
    const Vector3f uc = loop.step(1.0f / 500.0f, Vector3f(), Vector3f(),
                                  Vector3f(5.0f, -4.0f, 3.0f), Vector3f(), 0.1f,
                                  dwp, dwf, uf, sat);
    EXPECT_TRUE(sat);
    EXPECT_NEAR(uc.x, 0.1f, 1e-6f);
    EXPECT_NEAR(uc.y, -0.1f, 1e-6f);
    EXPECT_NEAR(uc.z, 0.0f, 1e-6f);
}

// ---- C1: filter-then-differentiate angular-accel estimator ----------------
// Filter-agnostic analytic properties (the C++ LowPassFilter2p biquad is a
// different filter family than the indi_harness Python Butter2 oracle, so we
// do not bit-match a per-sample trace -- see the Layer-C plan). Any
// DC-preserving low-pass satisfies these regardless of its exact coefficients.

// A constant-angular-accel ramp gyro.x = slope*t must, once the filter has
// settled, make filter-then-differentiate recover domega_filt.x == slope.
TEST(INDIRateLoop, FilterThenDiffRecoversRampSlope)
{
    const float fs = 400.0f, dt = 1.0f / fs, slope = 12.0f;  // rad/s^2
    AC_INDI_RateLoop loop;
    loop.configure(30.0f, fs, Vector3f(20, 20, 10), Vector3f(200, 200, 200));
    loop.configure_estimator(30.0f, fs);  // CC3_OMG_FILT > 0 -> filter-then-diff
    Vector3f dp, df, uf;
    bool sat;
    for (int i = 0; i < 400; i++) {
        const float t = i * dt;
        loop.step(dt, Vector3f(slope * t, 0, 0), Vector3f(), Vector3f(), Vector3f(),
                  1.0f, dp, df, uf, sat);
        if (i > 60) {
            EXPECT_NEAR(df.x, slope, 0.05f * slope) << "i=" << i;
        }
    }
    // phase-match invariant: production mode reports equal cutoffs for the
    // gyro pre-filter and the actuator-state filter.
    EXPECT_FLOAT_EQ(loop.domega_cutoff(), loop.uact_cutoff());
}

// Zero gyro input forever -> zero angular-accel estimate (no spurious output
// from a settled filter-then-differentiate pipeline fed silence).
TEST(INDIRateLoop, FilterThenDiffZeroInputZeroOutput)
{
    const float fs = 400.0f, dt = 1.0f / fs;
    AC_INDI_RateLoop loop;
    loop.configure(30.0f, fs, Vector3f(20, 20, 10), Vector3f(200, 200, 200));
    loop.configure_estimator(30.0f, fs);
    Vector3f dp, df, uf;
    bool sat;
    for (int i = 0; i < 100; i++) {
        loop.step(dt, Vector3f(), Vector3f(), Vector3f(), Vector3f(), 1.0f,
                  dp, df, uf, sat);
    }
    EXPECT_NEAR(df.x, 0.0f, 1e-6f);
    EXPECT_NEAR(df.y, 0.0f, 1e-6f);
    EXPECT_NEAR(df.z, 0.0f, 1e-6f);
}

// A constant (non-zero) gyro, once the filter settles, has zero derivative ->
// zero angular-accel estimate even though the rate itself is non-zero.
TEST(INDIRateLoop, FilterThenDiffConstantGyroZeroDomega)
{
    const float fs = 400.0f, dt = 1.0f / fs;
    AC_INDI_RateLoop loop;
    loop.configure(30.0f, fs, Vector3f(20, 20, 10), Vector3f(200, 200, 200));
    loop.configure_estimator(30.0f, fs);
    Vector3f dp, df, uf;
    bool sat;
    for (int i = 0; i < 200; i++) {
        loop.step(dt, Vector3f(0.7f, -0.4f, 0.2f), Vector3f(), Vector3f(), Vector3f(),
                  1.0f, dp, df, uf, sat);
        if (i > 60) {
            EXPECT_NEAR(df.x, 0.0f, 1e-3f) << "i=" << i;
            EXPECT_NEAR(df.y, 0.0f, 1e-3f) << "i=" << i;
            EXPECT_NEAR(df.z, 0.0f, 1e-3f) << "i=" << i;
        }
    }
}

// ---- Task 4: measured actuator-state reconstruction (torque-space) --------
// Oracle: params.py::mixer() quad-X normalized factors (see the
// measured_actuator_torque doc comment in AC_CustomControl_INDI.h).

// ArduPilot quad-X normalized mixer factors (from AP_MotorsMatrix after
// normalise_rpy_factors): roll/pitch +/-1, yaw +/-0.5. The reconstruction must
// project onto these and divide by sum(f^2) (4 for roll/pitch, 1 for yaw), so a
// full-authority differential recovers the SAME [-1,1] value as get_roll/pitch.
static const float RF_X[4] = { -1.0f, +1.0f, +1.0f, -1.0f };
static const float PF_X[4] = { +1.0f, -1.0f, +1.0f, -1.0f };
static const float YF_X[4] = { +0.5f, +0.5f, -0.5f, -0.5f };

TEST(AC_CustomControl_INDI, measured_actuator_torque_hover_is_zero)
{
    const float o2[4] = {0.5f, 0.5f, 0.5f, 0.5f};   // uniform -> no net torque
    Vector3f u;
    AC_CustomControl_INDI::measured_actuator_torque(o2, RF_X, PF_X, YF_X, u);
    EXPECT_NEAR(u.x, 0.0f, 1e-6f);
    EXPECT_NEAR(u.y, 0.0f, 1e-6f);
    EXPECT_NEAR(u.z, 0.0f, 1e-6f);
}
TEST(AC_CustomControl_INDI, measured_actuator_torque_roll_magnitude)
{
    // right side (idx0,3) up: sum(rf*o2) = -1.2, /sum(rf^2)=4 -> roll = -0.3
    // (the SAME value get_roll would report; the old un-normalized code gave
    // -0.6, so this magnitude assertion is what catches that 2x bug).
    const float o2[4] = {0.8f, 0.2f, 0.2f, 0.8f};
    Vector3f u; AC_CustomControl_INDI::measured_actuator_torque(o2, RF_X, PF_X, YF_X, u);
    EXPECT_NEAR(u.x, -0.3f, 1e-5f);
    EXPECT_NEAR(u.y,  0.0f, 1e-6f);   // balanced fore/aft -> no pitch
}
TEST(AC_CustomControl_INDI, measured_actuator_torque_pitch_magnitude)
{
    // front (idx0,2) up: sum(pf*o2) = 1.2, /4 -> pitch = +0.3
    const float o2[4] = {0.8f, 0.2f, 0.8f, 0.2f};
    Vector3f u; AC_CustomControl_INDI::measured_actuator_torque(o2, RF_X, PF_X, YF_X, u);
    EXPECT_NEAR(u.y, +0.3f, 1e-5f);
    EXPECT_NEAR(u.x,  0.0f, 1e-6f);   // balanced left/right -> no roll
}
TEST(AC_CustomControl_INDI, measured_actuator_torque_yaw_magnitude)
{
    // CCW pair (idx0,1) up: sum(yf*o2) = 0.6, /sum(yf^2)=1 -> yaw = +0.6
    const float o2[4] = {0.8f, 0.8f, 0.2f, 0.2f};
    Vector3f u; AC_CustomControl_INDI::measured_actuator_torque(o2, RF_X, PF_X, YF_X, u);
    EXPECT_NEAR(u.z, +0.6f, 1e-5f);
    EXPECT_NEAR(u.x,  0.0f, 1e-6f);
    EXPECT_NEAR(u.y,  0.0f, 1e-6f);
}

// ---- Task 5: G2 rotor-inertia yaw-reaction correction ---------------------
// Oracle: -g2 * sum(d_i * odot_i), d=[+1,+1,-1,-1] (motor order [FR,BL,FL,BR]).

TEST(AC_CustomControl_INDI, g2_yaw_correction_sign)
{
    const float odot_bal[4] = {10, 10, 10, 10};   // d-weighted sum = 0 -> no correction
    EXPECT_NEAR(AC_CustomControl_INDI::g2_yaw_correction(odot_bal, 0.01f), 0.0f, 1e-6f);
    const float odot_ccw[4] = {10, 10, 0, 0};      // CCW pair (d=+1) accel -> negative yaw
    EXPECT_LT(AC_CustomControl_INDI::g2_yaw_correction(odot_ccw, 0.01f), 0.0f);
}

// ---- C2/Task 4: RPM-source interface -- SITL shim + staleness fallback ----

// Bernoulli CRC-dropout probability = 1.0: every sample is dropped, so the
// shim must always report unhealthy and always substitute the first-order
// model estimate (never a raw zero/NaN passthrough). Under a rising
// throttle ramp the fallback estimate must track monotonically upward.
TEST(AP_INDI_RpmSource, ShimDropoutFallsBack)
{
    AP_INDI_RpmSource_Sim shim;
    shim.configure(/*qnt=*/50.0f, /*drop=*/1.0f, /*lat_ticks=*/2,
                   /*aff_a=*/4000.0f, /*aff_b=*/50.0f);
    shim.set_truth(0, 6000.0f);  // truth is present but must never get through

    float prev_om = -1.0f;
    for (int i = 0; i < 200; i++) {
        const float thr = constrain_float(i / 199.0f, 0.0f, 1.0f);  // 0 -> 1 ramp
        shim.set_throttle(0, thr);

        float om = 0.0f;
        const bool ok = shim.get(0, om);
        ASSERT_TRUE(ok);
        EXPECT_FALSE(shim.healthy(0)) << "i=" << i;
        EXPECT_TRUE(shim.used_fallback(0)) << "i=" << i;
        EXPECT_GT(om, 0.0f) << "i=" << i;          // model estimate, not zero
        EXPECT_TRUE(std::isfinite(om)) << "i=" << i; // never NaN/inf

        EXPECT_GE(om, prev_om) << "i=" << i;        // monotonic non-decreasing
        prev_om = om;
    }
}

// No dropout: quantization is applied and, once the (small) latency line
// has filled, the delivered eRPM->rad/s conversion is exact and the source
// reports healthy with no fallback.
TEST(AP_INDI_RpmSource, ShimNoDropoutQuantizedConversion)
{
    AP_INDI_RpmSource_Sim shim;
    shim.configure(/*qnt=*/100.0f, /*drop=*/0.0f, /*lat_ticks=*/0,
                   /*aff_a=*/4000.0f, /*aff_b=*/50.0f);
    // 6070 eRPM quantized to the nearest 100 -> 6100 eRPM.
    shim.set_truth(0, 6070.0f);

    float om = 0.0f;
    ASSERT_TRUE(shim.get(0, om));
    EXPECT_TRUE(shim.healthy(0));
    EXPECT_FALSE(shim.used_fallback(0));
    const float expect_omega = 6100.0f * (2.0f * static_cast<float>(M_PI) / 60.0f);
    EXPECT_NEAR(om, expect_omega, 1e-2f);
}

// ---- Task B1: differential-flatness map (outer loop, part 1) ---------------
// Oracle: indi_harness.flatness._core via tools/gen_flatness_oracle.py
// (lemniscate_slow samples + synthetic yaw-varying cases, m=1, g=9.81). The
// pure map reproduces the closed-form q, w (incl w_z), dw_x, dw_y, T; dw.z is
// an inter-tick central difference (0 in the pure map), validated in flight.
TEST(INDIOuterLoop, FlatReferenceMatchesOracle)
{
    struct Case {
        float p[3], v[3], a[3], j[3], s[3], psi, dpsi, ddpsi;
        float q[4], w[3], dwx, dwy, T;
    };
    static const Case C[] = {
    {{0.716735899f,0.669130606f,-10.000000000f},{0.977643136f,0.778219441f,0.000000000f},{-0.196497216f,-0.733783820f,0.000000000f},{-0.268026417f,-0.853413114f,0.000000000f},{0.053870828f,0.804684002f,0.000000000f},0.000000000f,0.000000000f,0.000000000f,
     {0.999253482f,-0.037319739f,0.009978798f,-0.000372684f},{-0.086492926f,0.027105574f,-0.001727653f},0.082720466f,-0.005881690f,9.839367350f},
    {{1.841009707f,0.719339800f,-10.000000000f},{0.409172681f,-0.727444544f,0.000000000f},{-0.504723264f,-0.788844362f,0.000000000f},{-0.112177014f,0.797732209f,0.000000000f},{0.138372749f,0.865064643f,0.000000000f},0.000000000f,0.000000000f,0.000000000f,
     {0.998867405f,-0.040095886f,0.025596307f,-0.001027470f},{0.080689791f,0.011700594f,0.004138122f},0.088500458f,-0.013858551f,9.854598977f},
    {{1.841009707f,-0.719339800f,-10.000000000f},{-0.409172681f,-0.727444544f,0.000000000f},{-0.504723264f,0.788844362f,0.000000000f},{0.112177014f,0.797732209f,0.000000000f},{0.138372749f,-0.865064643f,0.000000000f},0.000000000f,0.000000000f,0.000000000f,
     {0.998867405f,0.040095886f,0.025596307f,0.001027470f},{0.080689791f,-0.011700594f,0.004138122f},-0.088500458f,-0.013858551f,9.854598977f},
    {{0.716735899f,-0.669130606f,-10.000000000f},{-0.977643136f,0.778219441f,0.000000000f},{-0.196497216f,0.733783820f,0.000000000f},{0.268026417f,-0.853413114f,0.000000000f},{0.053870828f,-0.804684002f,0.000000000f},0.000000000f,0.000000000f,0.000000000f,
     {0.999253482f,0.037319739f,0.009978798f,0.000372684f},{-0.086492926f,-0.027105574f,-0.001727653f},-0.082720466f,-0.005881690f,9.839367350f},
    {{0.000000000f,0.000000000f,0.000000000f},{0.000000000f,0.000000000f,0.000000000f},{1.095693499f,-1.841706290f,-3.672211809f},{-5.801668374f,3.759242870f,4.953066927f},{1.706172412f,3.671944976f,0.697999863f},1.740289695f,0.631707108f,-0.994523000f,
     {0.644313109f,-0.074450933f,0.025614398f,0.760698087f},{0.351169630f,-0.283230567f,0.689574214f},-0.064795015f,-0.720836740f,13.651463715f},
    {{0.000000000f,0.000000000f,0.000000000f},{0.000000000f,0.000000000f,0.000000000f},{2.859234213f,-3.731315398f,1.837243571f},{-3.892132553f,4.358147068f,0.497534643f},{-3.204609751f,-1.237004461f,-7.546885262f},-1.502866894f,0.341248829f,0.294379023f,
     {0.681280796f,-0.034191645f,-0.261004688f,-0.683054875f},{-0.351839525f,0.379038951f,0.522420994f},-0.649369578f,0.643349830f,9.255418950f},
    {{0.000000000f,0.000000000f,0.000000000f},{0.000000000f,0.000000000f,0.000000000f},{0.923080892f,-0.930579566f,3.977679486f},{5.770024065f,2.226503814f,1.805511315f},{3.015147689f,-1.777257216f,-5.838455920f},0.885953361f,0.050708645f,-0.379516249f,
     {0.898503955f,-0.104006430f,-0.036748788f,0.424879785f},{-0.565550655f,-0.894617765f,0.036549521f},-0.614754515f,-0.457118574f,5.977793844f},
    };
    for (const auto &c : C) {
        AC_INDI_OuterLoop::FlatOutput fo{
            Vector3f(c.p[0],c.p[1],c.p[2]), Vector3f(c.v[0],c.v[1],c.v[2]),
            Vector3f(c.a[0],c.a[1],c.a[2]), Vector3f(c.j[0],c.j[1],c.j[2]),
            Vector3f(c.s[0],c.s[1],c.s[2]), c.psi, c.dpsi, c.ddpsi };
        const AC_INDI_OuterLoop::RefState r =
            AC_INDI_OuterLoop::flat_reference(fo, 1.0f, 9.81f);
        EXPECT_NEAR(r.q.q1, c.q[0], 1e-4f);
        EXPECT_NEAR(r.q.q2, c.q[1], 1e-4f);
        EXPECT_NEAR(r.q.q3, c.q[2], 1e-4f);
        EXPECT_NEAR(r.q.q4, c.q[3], 1e-4f);
        EXPECT_NEAR(r.w.x, c.w[0], 1e-4f);
        EXPECT_NEAR(r.w.y, c.w[1], 1e-4f);
        EXPECT_NEAR(r.w.z, c.w[2], 1e-4f);   // closed-form w_z (not inter-tick)
        EXPECT_NEAR(r.dw.x, c.dwx, 1e-4f);
        EXPECT_NEAR(r.dw.y, c.dwy, 1e-4f);
        EXPECT_NEAR(r.dw.z, 0.0f, 1e-9f);    // pure map leaves dw.z == 0
        EXPECT_NEAR(r.T, c.T, 1e-4f);
    }
}

// Degenerate-thrust guard: ~90 deg pitch along heading makes z_b nearly
// parallel to x_c (n -> 0). The map must stay finite (no NaN/inf).
TEST(INDIOuterLoop, FlatReferenceDegenerateStaysFinite)
{
    // a chosen so alpha = g*e3 - a points along +x (z_b ~ (1,0,0)); psi=0 =>
    // x_c=(1,0,0) parallel to z_b -> zxc ~ 0.
    AC_INDI_OuterLoop::FlatOutput fo{
        Vector3f(0,0,0), Vector3f(0,0,0), Vector3f(-20.0f, 0.0f, 9.81f),
        Vector3f(0,0,0), Vector3f(0,0,0), 0.0f, 0.0f, 0.0f };
    const AC_INDI_OuterLoop::RefState r =
        AC_INDI_OuterLoop::flat_reference(fo, 1.0f, 9.81f);
    EXPECT_TRUE(std::isfinite(r.q.q1) && std::isfinite(r.q.q2) &&
                std::isfinite(r.q.q3) && std::isfinite(r.q.q4));
    EXPECT_TRUE(std::isfinite(r.w.x) && std::isfinite(r.w.y) && std::isfinite(r.w.z));
    EXPECT_TRUE(std::isfinite(r.T));
}

// ---- Task B2: linear-INDI outer loop (outer loop, part 2) -----------------
// Oracle: indi_harness.outer_loop.OuterLoopINDI.update via
// tools/gen_flatness_oracle.py (kp=6, kv=4, cutoff=8 Hz, fs=500). The firmware
// LowPassFilter2p biquad is a different filter family than the Python Butter2
// (as the C1 inner-loop test notes), so we compare at STEADY STATE: drive
// constant inputs until both filters settle. Steady state is filter-family
// independent (both preserve DC), so z_b_des and T_cmd bit-match there. The
// firmware passes the scalar T_state (thrust/mass) directly in place of the
// oracle's kf*sum(Omega^2)/m (no RPM in SITL).
TEST(INDIOuterLoop, UpdateMatchesOracle)
{
    struct Case {
        float p[3], v[3], q[4], f_b[3], T_state;
        float fa[3], fp[3], fv[3];   // fo.a, fo.p, fo.v
        float z[3], T_cmd;
    };
    static const Case C[] = {
    {{0.000000000f,0.000000000f,0.000000000f},{0.000000000f,0.000000000f,0.000000000f},{1.000000000f,0.000000000f,0.000000000f,0.000000000f},{0.000000000f,0.000000000f,-9.810000000f},9.810000000f,
     {0.000000000f,0.000000000f,0.000000000f},{0.000000000f,0.000000000f,0.000000000f},{0.000000000f,0.000000000f,0.000000000f},
     {-0.000000000f,-0.000000000f,1.000000000f},9.810000000f},
    {{0.500000000f,-0.300000000f,-9.700000000f},{0.200000000f,0.100000000f,-0.050000000f},{1.000000000f,0.000000000f,0.000000000f,0.000000000f},{0.100000000f,-0.200000000f,-9.510000000f},9.810000000f,
     {0.600000000f,-0.400000000f,0.200000000f},{1.000000000f,0.500000000f,-10.000000000f},{0.400000000f,-0.200000000f,0.000000000f},
     {-0.337288516f,-0.266693245f,0.902835074f},12.748729348f},
    {{-0.400000000f,0.600000000f,-10.200000000f},{-0.100000000f,0.300000000f,0.080000000f},{0.974174432f,0.123922071f,-0.074353242f,0.173490899f},{0.500000000f,0.700000000f,-10.310000000f},11.022516000f,
     {-1.200000000f,0.900000000f,-0.500000000f},{0.000000000f,1.000000000f,-10.000000000f},{-0.300000000f,0.500000000f,-0.100000000f},
     {-0.022779511f,-0.308847912f,0.950838609f},11.313841594f},
    };
    for (const auto &c : C) {
        AC_INDI_OuterLoop ol;
        ol.configure(Vector3f(6,6,6), Vector3f(4,4,4), 8.0f, 500.0f, 1.0f, 9.81f);
        const Quaternion q(c.q[0], c.q[1], c.q[2], c.q[3]);
        AC_INDI_OuterLoop::FlatOutput fo{
            Vector3f(c.fp[0],c.fp[1],c.fp[2]), Vector3f(c.fv[0],c.fv[1],c.fv[2]),
            Vector3f(c.fa[0],c.fa[1],c.fa[2]), Vector3f(), Vector3f(), 0.0f, 0.0f, 0.0f };
        AC_INDI_OuterLoop::OuterState os{};
        for (int i = 0; i < 3000; i++) {
            os = ol.update(Vector3f(c.p[0],c.p[1],c.p[2]), Vector3f(c.v[0],c.v[1],c.v[2]),
                           q, Vector3f(c.f_b[0],c.f_b[1],c.f_b[2]), c.T_state, fo);
        }
        EXPECT_NEAR(os.z_b_des.x, c.z[0], 1e-3f);
        EXPECT_NEAR(os.z_b_des.y, c.z[1], 1e-3f);
        EXPECT_NEAR(os.z_b_des.z, c.z[2], 1e-3f);
        EXPECT_NEAR(os.T_cmd, c.T_cmd, 1e-3f);
    }
}

// attitude_from_thrust_dir: pure triad, incl the degenerate (~90 deg pitch
// along heading) fallback. Oracle: indi_harness.outer_loop.attitude_from_thrust_dir.
TEST(INDIOuterLoop, AttitudeFromThrustDirMatchesOracle)
{
    struct Case { float z[3], psi, q[4]; };
    static const Case C[] = {
    {{0.000000000f,0.000000000f,-1.000000000f},0.000000000f,{0.000000000f,1.000000000f,0.000000000f,0.000000000f}},
    {{0.200916258f,-0.100458129f,-0.974443852f},0.500000000f,{0.074510152f,0.961200500f,0.251625765f,0.085007712f}},
    {{-0.298970326f,0.398627101f,-0.867013944f},-1.200000000f,{0.078081324f,-0.786543212f,0.561120974f,0.245756658f}},
    {{0.999900590f,0.000000000f,-0.014100008f},0.000000000f,{0.000000000f,0.712074437f,0.000000000f,0.702103978f}},
    };
    for (const auto &c : C) {
        const Quaternion q = AC_INDI_OuterLoop::attitude_from_thrust_dir(
            Vector3f(c.z[0], c.z[1], c.z[2]), c.psi);
        EXPECT_NEAR(q.q1, c.q[0], 1e-4f);
        EXPECT_NEAR(q.q2, c.q[1], 1e-4f);
        EXPECT_NEAR(q.q3, c.q[2], 1e-4f);
        EXPECT_NEAR(q.q4, c.q[3], 1e-4f);
    }
}

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED

AP_GTEST_MAIN()
