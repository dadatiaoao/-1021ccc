#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>
#include <stddef.h>
typedef enum { GYRO_BUS_OK, GYRO_BUS_NACK, GYRO_BUS_TIMEOUT,
    GYRO_BUS_BUSY, GYRO_BUS_IO } GyroBusError;
typedef struct {
    void *ctx;
    GyroBusError (*init)(void *ctx);
    GyroBusError (*read)(void *ctx, uint8_t reg, uint8_t *data, size_t size);
    GyroBusError (*write)(void *ctx, uint8_t reg, uint8_t value);
    uint32_t (*millis)(void *ctx);
    uint32_t (*cycles)(void *ctx);
    uint32_t cycles_per_us;
} GyroIo;
typedef struct {
    uint32_t due;
    uint8_t phase, index, verify, who, check_index;
    GyroBusError bus_error;
} Mpu6050;
enum { MPU_NO_DATA=0, MPU_READY=1, MPU_BUS_ERROR=-1, MPU_ID_ERROR=-2,
    MPU_RESET_ERROR=-3, MPU_CONFIG_ERROR=-4, MPU_INVALID=-5,
    MPU_SATURATED=-6, MPU_RESET_DETECTED=-7 };
void Mpu_Begin(Mpu6050 *p, uint32_t now);
int Mpu_Step(Mpu6050 *p, const GyroIo *io, uint32_t now);
int Mpu_Fetch(Mpu6050 *p, const GyroIo *io, int16_t raw[3], uint32_t *sample_ms);
int Mpu_Check(Mpu6050 *p, const GyroIo *io);
#endif
