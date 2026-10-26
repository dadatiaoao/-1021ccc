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
} MechanismHomeConfig; /* ARM/LIFT switch homing only. */
typedef struct {
    float travel_mm;
    uint16_t rpm, stall_rpm, stall_ma, stall_ms, backoff_tenths;
    uint32_t timeout_ms;
} MechanismTrimHomeConfig;
typedef struct {
    MechanismAxisConfig axis[3];
    MechanismHomeConfig home[2];
    MechanismTrimHomeConfig trim_home;
    float outside_angle, top_window, work_window, test_down_mm, test_trim_mm;
    uint32_t prepare_ms, stable_ms, between_ms;
} MechanismConfig;
/* axis entries: units/rev, pulses/rev, tolerance, max RPM, move speed in units/s,
   protocol sign, calibrated confirmation, acceleration field.
   home entries (5,6 ONLY): max position, initial release, fast seek, backoff,
   slow seek, clearance, fast RPM, slow RPM, wiring confirmed, home confirmed.
   trim_home: total mm, home/center RPM, collision detection RPM/mA/ms,
   fixed return angle in 0.1 degrees, native homing timeout in ms.
   Finally: measured outside angle/top/work windows, requested test amplitudes,
   prepare/stable/between milliseconds. */
/* User confirmed 2026-10-25: ARM timing-pulley reduction is 1:3, hence
   120 deg/motor revolution; home is parallel to the chassis side, work is
   perpendicular (90 deg). ARM inward and LIFT upward homing directions work.
   LIFT is a direct timing-belt drive: approximately 165 mm over an average
   264855 feedback counts (six full-stroke runs), 65536 counts/motor rev.
   User physically confirmed 2026-10-26: 20 teeth x 2 mm pitch,
   hence 40 mm/motor rev. The earlier MXL alternative is superseded.
   The 160 mm command cap is unchanged; remaining physical travel is unverified.
   User confirmed 2026-10-26: TRIM screw diameter 6 mm, pitch 1 mm,
   single-start screw and direct motor coupling, hence 1 mm/motor rev;
   screw diameter does not enter the position conversion.
   RPM caps include 0.1 RPM headroom for floating-point conversion roundoff;
   the next integer RPM remains rejected.
   TRIM TEST positive motion retracts (user observed); keep +1 = retract.
   TRIM has 50 mm total travel. Native collision home backs out 1 mm,
   then moves outward another 24 mm to establish the midpoint as software zero.
   180 RPM and manufacturer collision defaults (300 RPM/800 mA/60 ms)
   are trial values, not measured contact-force or repeatability limits.
   User physically confirmed 2026-10-26: all motors (addresses 1-7)
   use 3200 command pulses per motor revolution. This is distinct from
   the 65536 feedback counts per revolution. */
#define MECHANISM_LIFT_MM_PER_REV 40.0
#define MECHANISM_TRIM_MM_PER_REV 1.0
#define MECHANISM_TRIAL_PULSES_PER_REV 3200u
#define MECHANISM_TRIAL_ARM_SIGN 1
#define MECHANISM_TRIAL_LIFT_SIGN 1
#define MECHANISM_TRIAL_TRIM_SIGN 1
#if MECHANISM_TRIAL_RUN
/* Confirmation bits below permit ONLY the trial explicitly requested by the
   user. They do not record measurements. Full-range search bounds are
   ARM 360 deg / LIFT 170 mm; the raw switch requests an early STOP.
   Software ranges are trial command caps, not measured mechanical end stops. */
#define MECHANISM_CALIBRATION { \
    {{120.0,MECHANISM_TRIAL_PULSES_PER_REV,1.0f,24.1f,6.0f,MECHANISM_TRIAL_ARM_SIGN,1u,100u}, \
     {MECHANISM_LIFT_MM_PER_REV,MECHANISM_TRIAL_PULSES_PER_REV,0.05f,60.1f,10.0f,MECHANISM_TRIAL_LIFT_SIGN,1u,100u}, \
     {MECHANISM_TRIM_MM_PER_REV,MECHANISM_TRIAL_PULSES_PER_REV,0.05f,180.1f,0.5f,MECHANISM_TRIAL_TRIM_SIGN,1u,100u}}, \
    {{90.0f,6.0f,360.0f,4.0f,10.0f,4.0f,24.0f,6.0f,1u,1u}, \
     {160.0f,8.0f,170.0f,4.0f,10.0f,4.0f,60.0f,12.0f,1u,1u}}, \
    {50.0f,180u,300u,800u,60u,3600u,25000u}, \
    90.0f,0.1f,2.0f,5.0f,5.0f,10000u,100u,500u }
#else
#define MECHANISM_CALIBRATION { \
    {{120.0,0u,1.0f,0.0f,0.0f,0,0u,100u}, \
     {MECHANISM_LIFT_MM_PER_REV,0u,0.5f,0.0f,0.0f,0,0u,100u}, \
     {MECHANISM_TRIM_MM_PER_REV,0u,0.5f,0.0f,0.0f,0,0u,100u}}, \
    {{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,5.0f,2.0f,0u,0u}, \
     {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,5.0f,2.0f,0u,0u}}, \
    {50.0f,180u,300u,800u,60u,3600u,25000u}, \
    0.0f,0.0f,0.0f,5.0f,5.0f,10000u,100u,500u }
#endif
enum { MECH_CFG_SCALE=1u, MECH_CFG_DIRECTION=2u, MECH_CFG_CONFIRM=4u,
       MECH_CFG_MOTION=8u, MECH_CFG_HOME=16u, MECH_CFG_POSE=32u };
void Mechanism_DefaultConfig(MechanismConfig *out);
uint32_t Mechanism_ConfigIssues(const MechanismConfig *config, unsigned axis);
#endif
