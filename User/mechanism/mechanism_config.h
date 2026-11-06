#ifndef MECHANISM_CONFIG_H
#define MECHANISM_CONFIG_H
#include <stdint.h>

#ifndef MECHANISM_TRIAL_RUN
#define MECHANISM_TRIAL_RUN 1
#endif
typedef struct {
    double units_per_rev; /* ARM degrees; LIFT/TRIM mm, per motor revolution */
    uint32_t pulses_per_rev;
    float tolerance, max_rpm, move_speed;
    int8_t positive_sign;
    uint8_t calibrated, acceleration;
} MechanismAxisConfig;
typedef struct {
    float max_position; /* ordinary command cap, never a homing seek bound */
} MechanismHomeConfig;
typedef struct {
    uint16_t rpm, stall_rpm, stall_ma, stall_ms, backoff_tenths;
    uint32_t timeout_ms;
    float progress_units; /* independent of ordinary move tolerance */
} MechanismCollisionConfig;
typedef struct {
    float travel_mm;
} MechanismTrimHomeConfig;
typedef struct {
    MechanismAxisConfig axis[3];
    MechanismHomeConfig home[2];
    MechanismCollisionConfig collision[3];
    MechanismTrimHomeConfig trim_home;
    float outside_angle, top_window, work_window, observation_mm, observation_window, grab_mm;
    float test_down_mm, test_trim_mm;
    uint32_t prepare_ms, stable_ms, between_ms;
} MechanismConfig;

/* 65536 feedback counts/rev and 3200 command pulses/rev are distinct.
 * ARM: 120 degrees/motor rev. LIFT: 40 mm/rev. TRIM: 1 mm/rev.
 * PR #4 verified collision-homing principle and the LIFT 900-tenths
 * backoff (90 motor degrees = nominal 10 mm). ARM backoff is 120
 * tenths of MOTOR degrees (4 mechanical degrees), not 120 arm degrees.
 * TRIM backs off 3600 tenths (1 mm), then travels another 24 mm to
 * the 50 mm stroke midpoint. Contact is zero for ARM/LIFT; midpoint
 * is zero for TRIM. Observation is the PR's physical old-coordinate
 * 0 mm (= nominal 10 mm from contact); its 130 mm grab point becomes
 * nominal 140 mm. These nominal positions need physical acceptance.
 * User revised 2026-11-06: search speeds 15/40/150 RPM; ordinary
 * default speeds 20/60/200 RPM. move_speed stores degrees or mm/s.
 * ARM 40 deg/s = 20 RPM; LIFT 40 mm/s = 60 RPM;
 * TRIM 3.333333 mm/s = 200 RPM nominally. */
#define MECHANISM_NO_PROGRESS_MS 1500u
#define MECHANISM_MAX_PROGRESS_SAMPLE_GAP_MS 600u
#define MECHANISM_LIFT_MM_PER_REV 40.0
#define MECHANISM_TRIM_MM_PER_REV 1.0
#define MECHANISM_TRIAL_PULSES_PER_REV 3200u
#if MECHANISM_TRIAL_RUN
#define MECHANISM_CALIBRATION { \
    {{120.0,3200u,1.0f,20.1f,40.0f,1,1u,100u}, \
     {40.0,3200u,0.05f,60.1f,40.0f,1,1u,100u}, \
     {1.0,3200u,0.05f,200.1f,3.333333f,1,1u,100u}}, \
    {{180.0f},{150.0f}}, \
    {{15u,300u,800u,60u,120u,30000u,0.20f}, \
     {40u,300u,800u,60u,900u,30000u,0.05f}, \
     {150u,300u,800u,60u,3600u,25000u,0.02f}}, \
    {50.0f},90.0f,10.5f,2.0f,10.0f,0.5f,140.0f,5.0f,5.0f,10000u,100u,500u }
#else
#define MECHANISM_CALIBRATION { \
    {{120.0,0u,1.0f,0.0f,0.0f,0,0u,100u}, \
     {40.0,0u,0.05f,0.0f,0.0f,0,0u,100u}, \
     {1.0,0u,0.05f,0.0f,0.0f,0,0u,100u}}, \
    {{180.0f},{150.0f}}, \
    {{15u,300u,800u,60u,120u,30000u,0.20f}, \
     {40u,300u,800u,60u,900u,30000u,0.05f}, \
     {150u,300u,800u,60u,3600u,25000u,0.02f}}, \
    {50.0f},90.0f,10.5f,2.0f,10.0f,0.5f,140.0f,5.0f,5.0f,10000u,100u,500u }
#endif
enum { MECH_CFG_SCALE=1u, MECH_CFG_DIRECTION=2u, MECH_CFG_CONFIRM=4u,
       MECH_CFG_MOTION=8u, MECH_CFG_HOME=16u, MECH_CFG_POSE=32u };
void Mechanism_DefaultConfig(MechanismConfig *out);
uint32_t Mechanism_ConfigIssues(const MechanismConfig *config, unsigned axis);
#endif
