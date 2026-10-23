#ifndef MECHANISM_CONFIG_H
#define MECHANISM_CONFIG_H
#include <stdint.h>

/* The initializer below is the ONLY vehicle calibration entry point. Unknown
 * physical values stay zero; synthetic tests are never vehicle calibration. */
typedef struct {
    double units_per_rev; /* ARM degrees; LIFT/TRIM mm, per MOTOR revolution */
    uint32_t pulses_per_rev;
    float tolerance, max_rpm, move_speed;
    int8_t positive_sign; /* +1/-1 in signed protocol feedback coordinates */
    uint8_t calibrated, acceleration;
} MechanismAxisConfig;
typedef struct {
    float max_position, initial_release, fast_seek, backoff, slow_seek, clearance;
    float fast_rpm, slow_rpm;
    uint8_t wiring_confirmed, home_confirmed;
} MechanismHomeConfig; /* ARM/LIFT only. TRIM has NO travel boundary config. */
typedef struct {
    MechanismAxisConfig axis[3];
    MechanismHomeConfig home[2];
    float outside_angle, top_window, work_window, test_down_mm, test_trim_mm;
    uint32_t prepare_ms, stable_ms, between_ms;
} MechanismConfig;
/* axis entries: units/rev, pulses/rev, tolerance, max RPM, move speed in units/s,
   protocol sign, calibrated confirmation, acceleration field.
   home entries (5,6 ONLY): max position, initial release, fast seek, backoff,
   slow seek, clearance, fast RPM, slow RPM, wiring confirmed, home confirmed.
   Finally: measured outside angle/top/work windows, requested test amplitudes,
   prepare/stable/between milliseconds. 5/2 RPM and tolerances are candidates
   only; calibration + wiring + homing confirmations are mandatory. */
#define MECHANISM_CALIBRATION { \
    {{0.0,0u,1.0f,0.0f,0.0f,0,0u,100u}, \
     {0.0,0u,0.5f,0.0f,0.0f,0,0u,100u}, \
     {0.0,0u,0.5f,0.0f,0.0f,0,0u,100u}}, \
    {{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,5.0f,2.0f,0u,0u}, \
     {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,5.0f,2.0f,0u,0u}}, \
    0.0f,0.0f,0.0f,5.0f,5.0f,10000u,100u,500u }
enum { MECH_CFG_SCALE=1u, MECH_CFG_DIRECTION=2u, MECH_CFG_CONFIRM=4u,
       MECH_CFG_MOTION=8u, MECH_CFG_HOME=16u, MECH_CFG_POSE=32u };
void Mechanism_DefaultConfig(MechanismConfig *out);
uint32_t Mechanism_ConfigIssues(const MechanismConfig *config, unsigned axis);
#endif
