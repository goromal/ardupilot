#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include "AC_CustomControl_INDI.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

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
