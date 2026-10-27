#include "chassis_kinematics.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
int Chassis_Finite(double v) { return v == v && v <= DBL_MAX && v >= -DBL_MAX; }
static int positive(float v) { return Chassis_Finite(v) && v > 0.0f; }
void Chassis_DefaultConfig(ChassisConfig *c)
{
    static const int8_t signs[4] = CHASSIS_MOTOR_SIGNS;
    memset(c, 0, sizeof(*c));
    c->mechanical_confirmed=CHASSIS_MECHANICAL_CONFIRMED;
    c->feedback_confirmed=CHASSIS_FEEDBACK_CONFIRMED;
    c->wheel_diameter_mm=CHASSIS_WHEEL_DIAMETER_MM;
    c->length_mm=CHASSIS_LENGTH_MM; c->width_mm=CHASSIS_WIDTH_MM;
    c->gear_ratio=CHASSIS_GEAR_RATIO; memcpy(c->motor_sign, signs, sizeof(signs));
    c->max_rpm=60.0f; c->max_speed_mm_s=100.0f; c->max_rate_deg_s=30.0f;
    c->acceleration_mm_s2=200.0f; c->acceleration_deg_s2=60.0f;
    c->position_kp=1.5f; c->heading_kp=2.0f; c->rate_damping=0.2f;
    c->position_window_mm=10.0f; c->angle_window_deg=2.0f; c->rest_rate_deg_s=3.0f;
    c->control_ms=20u; c->publish_ms=100u; c->input_lease_ms=300u;
    c->stable_ms=200u; c->between_ms=1000u; c->driver_acceleration=100u;
}
uint32_t Chassis_ConfigIssues(const ChassisConfig *c, uint8_t closed)
{
    uint32_t bad=0u; unsigned i;
    if (!c) return 31u;
    if (c->mechanical_confirmed!=1u) bad|=CHASSIS_CFG_MECHANICAL;
    /* Bounds avoid overflowing otherwise finite geometry during calculation. */
    if (!positive(c->wheel_diameter_mm) || c->wheel_diameter_mm>10000.0f ||
        !positive(c->length_mm) || c->length_mm>10000.0f ||
        !positive(c->width_mm) || c->width_mm>10000.0f ||
        !positive(c->gear_ratio) || c->gear_ratio>10000.0f) bad|=CHASSIS_CFG_GEOMETRY;
    for (i=0u;i<4u;++i) if (c->motor_sign[i]!=1 && c->motor_sign[i]!=-1) bad|=CHASSIS_CFG_DIRECTION;
    if (!positive(c->max_rpm) || c->max_rpm>3000.0f ||
        !positive(c->max_speed_mm_s) || c->max_speed_mm_s>10000.0f ||
        !positive(c->max_rate_deg_s) || c->max_rate_deg_s>360.0f ||
        !positive(c->acceleration_mm_s2) || c->acceleration_mm_s2>10000.0f ||
        !positive(c->acceleration_deg_s2) || c->acceleration_deg_s2>10000.0f ||
        !positive(c->position_kp) || c->position_kp>100.0f ||
        !positive(c->heading_kp) || c->heading_kp>100.0f ||
        !Chassis_Finite(c->rate_damping) || c->rate_damping<0.0f || c->rate_damping>100.0f ||
        !positive(c->position_window_mm) || !positive(c->angle_window_deg) ||
        !positive(c->rest_rate_deg_s) || !c->control_ms || c->control_ms>100u ||
        c->publish_ms<100u || c->publish_ms>200u || c->input_lease_ms<200u ||
        c->input_lease_ms>300u || c->stable_ms<200u || c->stable_ms>5000u ||
        c->between_ms<1000u || c->between_ms>10000u) bad|=CHASSIS_CFG_LIMITS;
    if (closed && c->feedback_confirmed!=1u) bad|=CHASSIS_CFG_FEEDBACK;
    return bad;
}
int Chassis_Solve(const ChassisConfig *c, ChassisVelocity v, ChassisWheels *o)
{
    double length, scale, k, w, f, r[4], maximum=0.0; unsigned i;
    if (!o) return 0;
    memset(o,0,sizeof(*o));
    if (Chassis_ConfigIssues(c,0u) || !Chassis_Finite(v.vx) ||
        !Chassis_Finite(v.vy) || !Chassis_Finite(v.omega)) return 0;
    length=sqrt((double)v.vx*v.vx+(double)v.vy*v.vy);
    if (length>c->max_speed_mm_s) {
        scale=c->max_speed_mm_s/length; v.vx=(float)(v.vx*scale); v.vy=(float)(v.vy*scale);
    }
    if (v.omega>c->max_rate_deg_s) v.omega=c->max_rate_deg_s;
    if (v.omega< -c->max_rate_deg_s) v.omega= -c->max_rate_deg_s;
    k=((double)c->length_mm+c->width_mm)*0.5; w=(double)v.omega*PI/180.0;
    f=60.0*(double)c->gear_ratio/(PI*(double)c->wheel_diameter_mm);
    r[0]=(v.vx-v.vy-k*w)*f; r[1]=(v.vx+v.vy+k*w)*f;
    r[2]=(v.vx+v.vy-k*w)*f; r[3]=(v.vx-v.vy+k*w)*f;
    for (i=0u;i<4u;++i) if (fabs(r[i])>maximum) maximum=fabs(r[i]);
    scale=maximum>c->max_rpm ? c->max_rpm/maximum : 1.0;
    o->scale=(float)scale; o->limited.vx=(float)(v.vx*scale);
    o->limited.vy=(float)(v.vy*scale); o->limited.omega=(float)(v.omega*scale);
    for (i=0u;i<4u;++i) {
        double rpm=r[i]*scale*c->motor_sign[i];
        o->rpm[i]=(float)rpm; o->quantized[i]=(int32_t)(rpm+(rpm<0.0 ? -0.5 : 0.5));
    }
    return 1;
}
