#ifndef GYRO_CONFIG_H
#define GYRO_CONFIG_H
/* MPU6050: AD0=GND, I2C1 PB6=SCL / PB7=SDA; XDA/XCL/INT unused. */
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
/* MPU6050 must use a real CAL; the old ICM module's 15.86 dps is not portable.
 * Keep the opt-in legacy test switch, but disable it in the shipped profile. */
#ifndef GYRO_TEMP_FIXED_BIAS_ENABLED
#define GYRO_TEMP_FIXED_BIAS_ENABLED 0
#endif
#ifndef GYRO_TEMP_FIXED_BIAS_DPS
#define GYRO_TEMP_FIXED_BIAS_DPS 15.86f
#endif
#if GYRO_ADDRESS_7BIT != 0x68u && GYRO_ADDRESS_7BIT != 0x69u
#error AD0 must select 0x68 or 0x69
#endif
#define GYRO_I2C_HZ 100000u
#define GYRO_BUS_TIMEOUT_MS 2u
/* Fixed matched FSR/ODR/filter tuple; change together and requalify timing. */
#define GYRO_CONFIG_VALUE 0x08u /* FS_SEL=1: +/-500 dps */
#define GYRO_DLPF_VALUE 0x04u /* 20 Hz gyro bandwidth, internal 1 kHz */
#define GYRO_SMPLRT_DIV 9u /* 1000 / (1+9) = 100 Hz */
#define GYRO_RESET_WAIT_MS 100u
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
