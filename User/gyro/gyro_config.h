#ifndef GYRO_CONFIG_H
#define GYRO_CONFIG_H
/* Temporary communication experiment: 0=normal, 1=WHO_AM_I only,
 * 2=1000 flawless reads, bank-0 check, ONE reset, then WHO_AM_I only.
 * No automatic bus reinitialization or configuration writes in this test. */
#ifndef GYRO_COMM_TEST
#define GYRO_COMM_TEST 2
#endif
#if GYRO_COMM_TEST < 0 || GYRO_COMM_TEST > 2
#error Invalid GYRO_COMM_TEST mode
#endif
#define GYRO_COMM_TEST_READS 1000u
#define GYRO_COMM_TEST_PERIOD_MS 10u
/* WDS601M CHIP side only: PS open, CS=3V3, SA0=GND. I2C1 PB6/PB7. */
#ifndef GYRO_ADDRESS_7BIT
#define GYRO_ADDRESS_7BIT 0x68u
#endif
#ifndef GYRO_AXIS
#define GYRO_AXIS 2 /* 0=X, 1=Y, 2=Z; verify the installed vertical axis. */
#endif
#ifndef GYRO_SIGN
#define GYRO_SIGN 1
#endif
#if GYRO_AXIS < 0 || GYRO_AXIS > 2 || (GYRO_SIGN != 1 && GYRO_SIGN != -1)
#error Invalid gyro mounting configuration
#endif
/*
 * TEMPORARY BRING-UP WORKAROUND (2026-10-22, remove before release):
 * the module currently fitted shows a large, apparently stable stationary
 * zero-rate offset of about 15.86 deg/s on the mapped axis (value seen as RAW
 * while the module is at rest; GYRO_SIGN is already applied, so it is in the
 * same unit as RAW/RATE). Because the normal CAL motion gate rejects raw
 * samples above GYRO_CAL_MAX_ABS_DPS (5 deg/s), CAL can only report
 * CAL:MOVING for this module, so RATE/REL would stay unavailable until a
 * healthy module is fitted. Until then, let the service treat the measured
 * offset as the known bias so the rest of the chain can be exercised.
 *
 * This is NOT calibration data, NOT a measured specification and NOT a final
 * configuration. To restore normal CAL-only behaviour set
 * GYRO_TEMP_FIXED_BIAS_ENABLED to 0 (or delete this block), fit a healthy
 * module, then run a real CAL. If GYRO_AXIS/GYRO_SIGN change before removal,
 * GYRO_TEMP_FIXED_BIAS_DPS must be re-measured in the new mapped axis.
 */
#ifndef GYRO_TEMP_FIXED_BIAS_ENABLED
#define GYRO_TEMP_FIXED_BIAS_ENABLED 1
#endif
#ifndef GYRO_TEMP_FIXED_BIAS_DPS
#define GYRO_TEMP_FIXED_BIAS_DPS 15.86f
#endif
#if GYRO_ADDRESS_7BIT != 0x68u && GYRO_ADDRESS_7BIT != 0x69u
#error SA0 must select 0x68 or 0x69
#endif
#define GYRO_I2C_HZ 100000u
#define GYRO_BUS_TIMEOUT_MS 2u
/* Fixed matched FSR/ODR/filter tuple; change together and requalify timing. */
#define GYRO_CONFIG0_VALUE 0x48u /* +/-500 dps, 100 Hz */
#define GYRO_LSB_PER_DPS 65.5f
#define GYRO_PERIOD_MS 10u
#define GYRO_POLL_MS 2u
#define GYRO_MAX_SAMPLE_MS 14u
#define GYRO_STALE_MS 30u
#define GYRO_NO_DATA_RESTART_MS 500u
#define GYRO_CHECK_MS 250u
#define GYRO_RETRY_MIN_MS 250u
#define GYRO_RETRY_MAX_MS 4000u
#define GYRO_STARTUP_MS 50u
#define GYRO_CAL_MS 2000u
#define GYRO_CAL_SAMPLES 201u
#define GYRO_CAL_TIMEOUT_MS 3000u
/* Initial thresholds, to be qualified on the actual vehicle. No proof of rest. */
#define GYRO_CAL_MAX_ABS_DPS 5.0f
#define GYRO_CAL_MAX_MEAN_DPS 3.0f
#define GYRO_CAL_MAX_RANGE_DPS 1.0f
#define GYRO_CAL_MAX_VARIANCE 0.04f
#define GYRO_SATURATION_RAW 32760
#endif
