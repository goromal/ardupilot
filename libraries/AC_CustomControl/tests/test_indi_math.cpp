// Unit tests for the Layer-A INDI math (design-doc S3), validated against the
// tested S0 Python reference (indi_harness) used as the oracle. Reference
// vectors were dumped from indi_harness.tilt_yaw.attitude_rate_ref /
// indi_harness.inner_loop.InnerLoopINDI at build time and hard-coded here.
#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>
#include <AC_CustomControl/AC_CustomControl_INDI.h>

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

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED

AP_GTEST_MAIN()
