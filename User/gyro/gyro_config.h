#ifndef GYRO_CONFIG_H
#define GYRO_CONFIG_H
/* WDS601M CHIP side only: PS open, CS=3V3, SA0=GND. I2C1 PB6/PB7. */
#ifndef GYRO_ADDRESS_7BIT
#define GYRO_ADDRESS_7BIT 0x68u
#endif
#ifndef GYRO_AXIS
#define GYRO_AXIS 2 /* 0=X, 1=Y, 2=Z; verify the installed vertical axis. */
#endif
#ifndef GYRO_SIGN
#define GYRO_SIGN -1
#endif
#if GYRO_AXIS < 0 || GYRO_AXIS > 2 || (GYRO_SIGN != 1 && GYRO_SIGN != -1)
#error Invalid gyro mounting configuration
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
/* All three axes accept measured stationary offsets up to 25 dps.
 * These are acceptance bounds, never a bias to subtract. CAL measures it.
 * Range/variance and motor gates still apply; the operator must ensure rest.
 * A steady rotation cannot be distinguished from bias using gyro data alone.
 */
#define GYRO_CAL_MAX_ABS_DPS 25.0f
#define GYRO_CAL_MAX_MEAN_DPS 25.0f
#define GYRO_CAL_MAX_RANGE_DPS 1.0f
#define GYRO_CAL_MAX_VARIANCE 0.04f
#define GYRO_SATURATION_RAW 32760
#endif
