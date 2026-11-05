#ifndef ICM42688_H
#define ICM42688_H
#include <stdint.h>
#include <stddef.h>
typedef enum { GYRO_BUS_OK, GYRO_BUS_NACK, GYRO_BUS_TIMEOUT,
    GYRO_BUS_BUSY, GYRO_BUS_IO,
    GYRO_BUS_SCL_LOW, GYRO_BUS_SDA_LOW, GYRO_BUS_BOTH_LOW,
    GYRO_BUS_HW_BUSY } GyroBusError;
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
} Icm42688;
enum { ICM_NO_DATA=0, ICM_READY=1, ICM_BUS_ERROR=-1, ICM_ID_ERROR=-2,
    ICM_RESET_ERROR=-3, ICM_CONFIG_ERROR=-4, ICM_INVALID=-5,
    ICM_SATURATED=-6, ICM_RESET_DETECTED=-7 };
void Icm_Begin(Icm42688 *p, uint32_t now);
int Icm_Step(Icm42688 *p, const GyroIo *io, uint32_t now);
int Icm_Fetch(Icm42688 *p, const GyroIo *io, int16_t raw[3], uint32_t *sample_ms);
int Icm_Check(Icm42688 *p, const GyroIo *io);
#endif
