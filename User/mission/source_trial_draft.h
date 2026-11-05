#ifndef SOURCE_TRIAL_DRAFT_H
#define SOURCE_TRIAL_DRAFT_H
/* Draft map only, millimetres: origin at upper-right, X left, Y down.
   These points are illustrative, NOT collision checked or measured.
   QR position is variable and must be observed, not assumed to be at the sample point.
   Do not use these coordinates for movement until geometry, heading and sensing are confirmed. */
#define SOURCE_DRAFT_CONFIRMED 0u
#define SOURCE_DRAFT_SPEED_MM_S 60.0f
#define SOURCE_DRAFT_ZONE1_X_MM 150.0f
#define SOURCE_DRAFT_ZONE1_Y_MM 150.0f
#define SOURCE_DRAFT_ZONE2_X_MM 150.0f
#define SOURCE_DRAFT_ZONE2_Y_MM 2250.0f
#define SOURCE_DRAFT_QR_EXAMPLE_X_MM 300.0f
#define SOURCE_DRAFT_QR_EXAMPLE_Y_MM 1240.0f
#define SOURCE_DRAFT_RIGHT_UPPER_X_MM 275.0f
#define SOURCE_DRAFT_RIGHT_UPPER_Y_MM 300.0f
#define SOURCE_DRAFT_DOCK_X_MM 1230.0f
#define SOURCE_DRAFT_DOCK_Y_MM 340.0f
/* Reference project docking node, source (1230,340) to processing (1200,2030).
   These are reference robot coordinates, not measured/calibrated for our robot. */
#define PROCESS_DRAFT_DOCK_X_MM 1200.0f
#define PROCESS_DRAFT_DOCK_Y_MM 2030.0f
#define PROCESS_DRAFT_CONFIRMED 0u
/* Additional nodes copied from reference initialize_graph(), mm. */
#define REFERENCE_CENTER_X_MM 1200.0f
#define REFERENCE_CENTER_Y_MM 1200.0f
#define REFERENCE_STORAGE_X_MM 2000.0f
#define REFERENCE_STORAGE_Y_MM 1200.0f
#define REFERENCE_RIGHT_LOWER_X_MM 275.0f
#define REFERENCE_RIGHT_LOWER_Y_MM 2060.0f
/* Rulebook source tabletop height is 80-100mm. This is NOT an axis command. */
#define SOURCE_DRAFT_TABLE_HEIGHT_MM 90.0f
#define SOURCE_DRAFT_GRIP_POSITION_CONFIRMED 0u
#endif
