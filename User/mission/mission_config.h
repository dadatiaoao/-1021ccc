#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H
#include <stdint.h>
#define MISSION_WAYPOINTS_MAX 8u
typedef enum { MISSION_ROUTE_HOME_QR, MISSION_ROUTE_QR_SOURCE,
    MISSION_ROUTE_SOURCE_ROUGH, MISSION_ROUTE_ROUGH_STORAGE,
    MISSION_ROUTE_STORAGE_SOURCE, MISSION_ROUTE_STORAGE_HOME,
    MISSION_ROUTE_COUNT } MissionRouteId;
/* Fixed zone-2 frame: origin = measured chassis reference point at START;
   +X = initial body forward, +Y = body left, positive heading = CCW.
   Waypoints are absolute in THIS frame, never distances from the previous one.
   Drive to each point holding current heading, then turn at that point. */
typedef struct { float x_mm, y_mm, heading_deg; } MissionWaypoint;
typedef struct {
    uint8_t confirmed, count;
    MissionWaypoint points[MISSION_WAYPOINTS_MAX];
} MissionRoute;
typedef struct {
    MissionRoute routes[MISSION_ROUTE_COUNT];
    float speed_mm_s, turn_deg_s;
    uint8_t initial_pose_confirmed;
} MissionConfig;
void Mission_DefaultConfig(MissionConfig *config);
int Mission_ConfigValid(const MissionConfig *config);
#endif
