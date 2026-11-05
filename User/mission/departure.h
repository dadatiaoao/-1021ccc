#ifndef DEPARTURE_H
#define DEPARTURE_H
#include "../chassis/chassis.h"
#include "../qr/qr_service.h"
#include "departure_config.h"

typedef enum { DEPARTURE_IDLE, DEPARTURE_PREPARE, DEPARTURE_MOVE,
    DEPARTURE_SETTLE, DEPARTURE_SCAN, DEPARTURE_DONE,
    DEPARTURE_ABORTED, DEPARTURE_FAULT } DepartureState;
typedef enum { DEPARTURE_OK, DEPARTURE_BUSY, DEPARTURE_ARM,
    DEPARTURE_SENSOR, DEPARTURE_MOTOR, DEPARTURE_SCANNER,
    DEPARTURE_CHASSIS, DEPARTURE_TIMEOUT, DEPARTURE_QR_TIMEOUT,
    DEPARTURE_STOPPED, DEPARTURE_CONFIG } DepartureError;
typedef struct {
    void *context;
    int (*arm_ready)(void *context, uint32_t now);
    /* Must discard UART/DMA backlog, then start a new QrService session. */
    void (*new_scan)(void *context, uint32_t now);
} DeparturePort;
typedef struct {
    Chassis *chassis;
    MotorBus *motor;
    QrService *qr;
    DeparturePort port;
    DepartureState state;
    DepartureError error;
    QrTaskCode task;
    GyroSnapshot start_gyro;
    uint32_t started_ms, phase_ms, request_id, generation;
    uint32_t scan_session, scan_seq, scan_uart_errors, scan_overflows;
    uint8_t started, task_valid;
} Departure;
void Departure_Init(Departure *d, Chassis *c, MotorBus *m, QrService *q, DeparturePort port);
int Departure_Start(Departure *d, uint32_t now);
void Departure_Process(Departure *d, uint32_t now);
void Departure_Stop(Departure *d, uint32_t now);
int Departure_IsRunning(const Departure *d);
const char *Departure_StateName(DepartureState state);
const char *Departure_ErrorName(DepartureError error);
void Departure_Line(char out[43], unsigned row, const Departure *d);
#endif
