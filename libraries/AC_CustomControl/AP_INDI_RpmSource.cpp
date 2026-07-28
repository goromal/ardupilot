#include "AP_INDI_RpmSource.h"

#if AP_CUSTOMCONTROL_INDI_ENABLED

#include <AP_ESC_Telem/AP_ESC_Telem.h>

// ---------------------------------------------------------------------------
// AP_INDI_RpmSource_ESC -- hardware backend
// ---------------------------------------------------------------------------

bool AP_INDI_RpmSource_ESC::get(uint8_t i, float &omega)
{
    float erpm;
    if (!AP::esc_telem().get_rpm(i, erpm)) {
        _healthy_last[i] = false;
        return false;
    }
    _healthy_last[i] = true;
    // eRPM -> mechanical rad/s: rad/s = (erpm / pole_pairs) * 2*pi/60
    omega = (erpm / _pole_pairs) * (2.0f * M_PI / 60.0f);
    return true;
}

bool AP_INDI_RpmSource_ESC::healthy(uint8_t i)
{
    return _healthy_last[i];
}

// ---------------------------------------------------------------------------
// AP_INDI_RpmSource_Sim -- SITL shim: quantization + Bernoulli CRC-dropout +
// latency, with a deterministic first-order fallback model on drop/stale.
// ---------------------------------------------------------------------------

void AP_INDI_RpmSource_Sim::configure(float qnt, float drop, uint8_t lat_ticks,
                                       float aff_a, float aff_b)
{
    _qnt = qnt;
    _drop = constrain_float(drop, 0.0f, 1.0f);
    _lat_ticks = lat_ticks;
    _aff_a = aff_a;
    _aff_b = aff_b;
}

float AP_INDI_RpmSource_Sim::pole_conv(float erpm) const
{
    // shim assumes pole_pairs == 1: the signal path being modelled is the
    // eRPM->rad/s conversion itself, not motor-specific pole geometry.
    return erpm * (2.0f * M_PI / 60.0f);
}

float AP_INDI_RpmSource_Sim::draw01()
{
    // deterministic xorshift32, reproducible across test runs.
    _rng ^= _rng << 13;
    _rng ^= _rng >> 17;
    _rng ^= _rng << 5;
    return (_rng & 0xFFFFFFu) / float(0x1000000u);
}

bool AP_INDI_RpmSource_Sim::get(uint8_t i, float &omega)
{
    // 1) quantize this tick's truth sample to the configured LSB.
    float q = _truth[i];
    if (_qnt > 0.0f) {
        q = roundf(q / _qnt) * _qnt;
    }

    // 2) push into the per-motor latency line (delay line of depth
    // lat_ticks); reading the just-displaced slot after the write yields
    // the sample from exactly lat_ticks ago, once the line has filled.
    const uint8_t buflen = (uint8_t)MIN((uint16_t)_lat_ticks + 1, (uint16_t)MAX_LAT);
    _ring[i][_ring_head[i]] = q;
    _ring_head[i] = (uint8_t)((_ring_head[i] + 1) % buflen);
    const float delayed = _ring[i][_ring_head[i]];

    if (_ring_pushes[i] < buflen) {
        _ring_pushes[i]++;
    }
    const bool have_history = (_ring_pushes[i] >= buflen);

    // 3) Bernoulli CRC-dropout decision, fresh every tick.
    const bool dropped = (draw01() < _drop);

    const bool healthy_now = have_history && !dropped;
    _healthy_last[i] = healthy_now;

    if (healthy_now) {
        omega = pole_conv(delayed);
        _model[i] = omega;   // keep the model warm so fallback doesn't jump
        _fb[i] = false;
    } else {
        const float target = _aff_a * _thr[i] + _aff_b;
        _model[i] += (DT / TAU) * (target - _model[i]);
        omega = _model[i];
        _fb[i] = true;
    }
    return true;
}

#endif  // AP_CUSTOMCONTROL_INDI_ENABLED
