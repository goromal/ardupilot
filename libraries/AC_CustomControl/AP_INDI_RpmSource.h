#pragma once

#include "AC_CustomControl_config.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors_Class.h>   // for AP_MOTORS_MAX_NUM_MOTORS

// Layer-C Task 4: measured per-motor rotor-speed source for the INDI
// increment (measured actuator state) and the G2 rotor-inertia term.
//
// AP_INDI_RpmSource is the abstract interface; two implementations:
//   - AP_INDI_RpmSource_ESC:  hardware backend, reads AP_ESC_Telem.
//   - AP_INDI_RpmSource_Sim:  SITL shim that degrades an injected "truth"
//     mechanical-RPM signal (quantization, Bernoulli dropout, latency) so
//     real bidi-DShot link would, so the staleness -> model-fallback logic
//     is exercised in sim before hardware. On drop/stale the shim falls
//     back to a first-order throttle-driven model estimate and flags
//     used_fallback() so callers (and gtests) can tell truth from estimate.
class AP_INDI_RpmSource {
public:
    virtual ~AP_INDI_RpmSource() {}

    // filtered rotor speed [rad/s] for motor i. Returns false if no value
    // (even a fallback estimate) is available.
    virtual bool get(uint8_t i, float &omega) = 0;

    // true if the last get() for motor i was backed by fresh, healthy
    // telemetry (i.e. NOT a fallback/model estimate).
    virtual bool healthy(uint8_t i) = 0;

    // true if the last get() for motor i returned a fallback/model
    // estimate rather than measured telemetry.
    virtual bool used_fallback(uint8_t i) { (void)i; return false; }

    // feed the current commanded throttle for motor i (0..1); used by the
    // fallback model to track a plausible rotor speed while telemetry is
    // stale/dropped. No-op for backends that don't need it.
    virtual void set_throttle(uint8_t i, float thr) { (void)i; (void)thr; }
};

// AP_ESC_Telem contains MECHANICAL RPM: DShot/BLHeli already divide by
// motor pole pairs before publishing. Do not divide a second time here.
class AP_INDI_RpmSource_ESC : public AP_INDI_RpmSource {
public:
    bool get(uint8_t i, float &omega) override;
    bool healthy(uint8_t i) override;

private:
    bool _healthy_last[AP_MOTORS_MAX_NUM_MOTORS] {};
};

// SITL shim: applies link degradation to an injected mechanical-RPM
// value, with a deterministic first-order fallback model when the
// (simulated) telemetry sample is dropped or stale.
class AP_INDI_RpmSource_Sim : public AP_INDI_RpmSource {
public:
    // qnt       : quantization step in mechanical RPM (post ESC pole conversion).
    // drop      : Bernoulli CRC-dropout probability per sample, in [0,1].
    // lat_ticks : latency, in control-loop ticks (ring-buffer depth).
    // aff_a,b   : fallback affine model omega_model_target = aff_a*throttle + aff_b
    //             (rad/s), driven through a first-order lag (tau) toward the
    //             affine target whenever telemetry is dropped/stale.
    void configure(float qnt, float drop, uint8_t lat_ticks, float aff_a, float aff_b);

    // Feed the upstream mechanical-RPM sample and its freshness every tick.
    // A sample is consumed once: omission must not replay cached truth as fresh.
    void set_truth(uint8_t i, float rpm, bool valid = true) {
        _truth[i] = rpm;
        _truth_valid[i] = valid && std::isfinite(rpm) && rpm >= 0.0f;
    }

    void set_throttle(uint8_t i, float thr) override { _thr[i] = thr; }

    bool get(uint8_t i, float &omega) override;
    bool healthy(uint8_t i) override { return _healthy_last[i]; }
    bool used_fallback(uint8_t i) override { return _fb[i]; }

private:
    static constexpr uint8_t MAX_LAT = 32;

    float rpm_to_omega(float rpm) const;
    float draw01();                      // deterministic xorshift U[0,1)

    // config
    float _qnt = 0.0f;
    float _drop = 0.0f;
    uint8_t _lat_ticks = 0;
    float _aff_a = 0.0f;
    float _aff_b = 0.0f;

    // per-motor state
    float _truth[AP_MOTORS_MAX_NUM_MOTORS] {};
    bool _truth_valid[AP_MOTORS_MAX_NUM_MOTORS] {};
    float _thr[AP_MOTORS_MAX_NUM_MOTORS] {};
    float _model[AP_MOTORS_MAX_NUM_MOTORS] {};
    bool _fb[AP_MOTORS_MAX_NUM_MOTORS] {};
    bool _healthy_last[AP_MOTORS_MAX_NUM_MOTORS] {};

    // latency ring buffer: per-motor circular delay line of quantized RPM
    // samples (dropout is decided fresh each tick, independent of latency --
    // see .cpp for the exact pipeline order). _ring_pushes counts writes
    // (saturating at buflen) so we know when the delay line has filled.
    float _ring[AP_MOTORS_MAX_NUM_MOTORS][MAX_LAT] {};
    bool _ring_valid[AP_MOTORS_MAX_NUM_MOTORS][MAX_LAT] {};
    uint8_t _ring_head[AP_MOTORS_MAX_NUM_MOTORS] {};
    uint8_t _ring_pushes[AP_MOTORS_MAX_NUM_MOTORS] {};

    // deterministic PRNG (xorshift32) so dropout is reproducible in tests.
    uint32_t _rng = 0x1234567u;

    static constexpr float DT = 1.0f / 400.0f;
    static constexpr float TAU = 0.03f;
};

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
