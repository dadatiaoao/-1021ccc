/* Motor bus gates and persistent STOP policy. Historical filename retained.
   Vehicle motion is configured separately in chassis/chassis_config.h. */
#ifndef MOTOR_TEST_CONFIG_H
#define MOTOR_TEST_CONFIG_H
/* Full motor power-cycle, no other bus writer and no leftover cache must be
   confirmed before setting this to 1 for a controlled synchronous test boot.
   MCU-only reset does NOT establish a clean motor cache. No runtime unlock. */
#define MOTOR_SYNC_CLEAN_BOOT_CONFIRMED 0u
#define MOTOR_POSITION_TOLERANCE_COUNTS 128u
#define MOTOR_STOP_TIMEOUT_MS 3000u
/* Bring-up bus budget, not a mechanical stopping-time guarantee. Initial
   submission plus two fast retries, then persistent low-rate verification. */
#define MOTOR_STOP_RETRY_MS 200u
#define MOTOR_STOP_FAST_ATTEMPTS 3u
#define MOTOR_STOP_BACKOFF_MS 1000u
/* CAL bench scope: all discovered/commanded axes remain relevant. Never-seen,
   never-commanded nodes may be absent; STOP intent cannot be excluded. */
#define MOTOR_CAL_AXIS_MASK 0x7Fu

/* 1 restores the legacy requirement for an explicit manual API session.
   The dashboard has no manual motor motion controls.
   UNKNOWN recovery still requires fresh zero-speed, fault-free feedback and
   no unconfirmed STOP. Set its window to 0 to require explicit STOP recovery. */
#ifndef MOTOR_DRIVER_REQUIRE_MANUAL
#define MOTOR_DRIVER_REQUIRE_MANUAL 0u
#endif
#ifndef MOTOR_UNKNOWN_AUTO_RECOVER_MS
#define MOTOR_UNKNOWN_AUTO_RECOVER_MS 500u
#endif
#endif
