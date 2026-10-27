#include "mission_config.h"
#include "../chassis/chassis_kinematics.h"
#include <math.h>
#include <string.h>
void Mission_DefaultConfig(MissionConfig *c)
{
    memset(c,0,sizeof(*c));
    c->speed_mm_s=80.0f; c->turn_deg_s=20.0f;
    /* Fill measured waypoints here. count=0 / confirmed=0 deliberately rejects
       START; map dimensions cannot substitute for this vehicle's clearances,
       camera observation poses, initial heading or random obstacles. */
}
int Mission_ConfigValid(const MissionConfig *c)
{
    unsigned i,j;
    if (!c || c->initial_pose_confirmed!=1u || !Chassis_Finite(c->speed_mm_s) ||
        c->speed_mm_s<=0.0f || c->speed_mm_s>1000.0f ||
        !Chassis_Finite(c->turn_deg_s) || c->turn_deg_s<=0.0f || c->turn_deg_s>90.0f) return 0;
    for (i=0u;i<MISSION_ROUTE_COUNT;++i) {
        const MissionRoute *r=&c->routes[i];
        if (r->confirmed!=1u || !r->count || r->count>MISSION_WAYPOINTS_MAX) return 0;
        for (j=0u;j<r->count;++j) {
            const MissionWaypoint *p=&r->points[j];
            if (!Chassis_Finite(p->x_mm) || !Chassis_Finite(p->y_mm) ||
                !Chassis_Finite(p->heading_deg) || fabs(p->x_mm)>4000.0 ||
                fabs(p->y_mm)>4000.0 || fabs(p->heading_deg)>180.0) return 0;
        }
    }
    /* Home means the same reference point AND orientation as departure. */
    {
        const MissionRoute *r=&c->routes[MISSION_ROUTE_STORAGE_HOME];
        const MissionWaypoint *p=&r->points[r->count-1u];
        if (p->x_mm!=0.0f || p->y_mm!=0.0f || p->heading_deg!=0.0f) return 0;
    }
    return 1;
}
