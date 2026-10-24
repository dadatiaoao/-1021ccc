#ifndef MECHANISM_CONFIG_H
#define MECHANISM_CONFIG_H
#include <stdint.h>

/* The initializers below are the ONLY vehicle calibration entry point.
 * User-requested TRIAL defaults permit a bounded bench run, NOT a calibration.
 * Set MECHANISM_TRIAL_RUN=0 to restore the uncalibrated/no-motion profile. */
#ifndef MECHANISM_TRIAL_RUN
#define MECHANISM_TRIAL_RUN 1
#endif
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
   prepare/stable/between milliseconds. */
/* Trial ASSUMPTIONS, not measurements: 3200 command pulses/motor revolution;
   protocol positive = ARM out / LIFT down / TRIM forward. Verify actual motion.
   1:3 ARM reduction -> 120 deg/motor revolution. LIFT and TRIM temporarily use
   2 mm/motor revolution; screw type alone does NOT establish its actual lead. */
#define MECHANISM_LIFT_PREVIEW_MM_PER_REV 2.0
#define MECHANISM_TRIAL_PULSES_PER_REV 3200u
#define MECHANISM_TRIAL_ARM_SIGN 1
#define MECHANISM_TRIAL_LIFT_SIGN 1
#define MECHANISM_TRIAL_TRIM_SIGN 1
#if MECHANISM_TRIAL_RUN
/* Confirmation bits below permit ONLY the trial explicitly requested by the
   user. They do not record measurements. Start close to the home switches:
   search bounds are ARM 10 deg / LIFT 2 mm (in these assumed units).
   Software ranges are trial command caps, not measured mechanical end stops. */
#define MECHANISM_CALIBRATION { \
    {{120.0,MECHANISM_TRIAL_PULSES_PER_REV,1.0f,3.0f,6.0f,MECHANISM_TRIAL_ARM_SIGN,1u,100u}, \
     {MECHANISM_LIFT_PREVIEW_MM_PER_REV,MECHANISM_TRIAL_PULSES_PER_REV,0.05f,15.0f,0.5f,MECHANISM_TRIAL_LIFT_SIGN,1u,100u}, \
     {2.0,MECHANISM_TRIAL_PULSES_PER_REV,0.05f,15.0f,0.5f,MECHANISM_TRIAL_TRIM_SIGN,1u,100u}}, \
    {{90.0f,3.0f,10.0f,2.0f,4.0f,2.0f,2.0f,1.0f,1u,1u}, \
     {10.0f,0.5f,2.0f,0.2f,0.4f,0.2f,5.0f,2.0f,1u,1u}}, \
    90.0f,0.1f,2.0f,5.0f,5.0f,10000u,100u,500u }
#else
#define MECHANISM_CALIBRATION { \
    {{120.0,0u,1.0f,0.0f,0.0f,0,0u,100u}, \
     {MECHANISM_LIFT_PREVIEW_MM_PER_REV,0u,0.5f,0.0f,0.0f,0,0u,100u}, \
     {0.0,0u,0.5f,0.0f,0.0f,0,0u,100u}}, \
    {{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,5.0f,2.0f,0u,0u}, \
     {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,5.0f,2.0f,0u,0u}}, \
    0.0f,0.0f,0.0f,5.0f,5.0f,10000u,100u,500u }
#endif
enum { MECH_CFG_SCALE=1u, MECH_CFG_DIRECTION=2u, MECH_CFG_CONFIRM=4u,
       MECH_CFG_MOTION=8u, MECH_CFG_HOME=16u, MECH_CFG_POSE=32u };
void Mechanism_DefaultConfig(MechanismConfig *out);
uint32_t Mechanism_ConfigIssues(const MechanismConfig *config, unsigned axis);
#endif
