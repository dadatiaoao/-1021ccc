#ifndef MISSION_PREVIEW_PARAMETERS_H
#define MISSION_PREVIEW_PARAMETERS_H
/* REFERENCE-ONLY arena frame, millimetres: origin upper-right, X left, Y down.
 * These are not calibrated for this robot and are never motor commands.
 * QR point is an example only: the actual QR location varies.
 * Heading, obstacle clearance and transitions still require measurement.
 */
typedef struct { const char *name; int x_mm, y_mm; } PreviewPoint;
static const PreviewPoint PREVIEW_ZONE1 = {"ZONE1",150,150};
static const PreviewPoint PREVIEW_ZONE2 = {"ZONE2",150,2250};
static const PreviewPoint PREVIEW_QR = {"QR_EXAMPLE",300,1240};
static const PreviewPoint PREVIEW_SOURCE = {"SOURCE",1230,340};
static const PreviewPoint PREVIEW_CENTER = {"CENTER",1200,1200};
static const PreviewPoint PREVIEW_PROCESS = {"PROCESS",1200,2030};
static const PreviewPoint PREVIEW_STORAGE = {"STORAGE",2000,1200};
/* User-measured image point; validity depends on camera pose and resolution. */
#define PREVIEW_GRASP_X 617
#define PREVIEW_GRASP_Y 338
#define PREVIEW_LIFT_DOWN_MM 100
#define PREVIEW_GRASP_TOLERANCE_PX 20 /* provisional */
#endif
