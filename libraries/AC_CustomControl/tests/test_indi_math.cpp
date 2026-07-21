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

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED

AP_GTEST_MAIN()
