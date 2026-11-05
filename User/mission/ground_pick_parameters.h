#ifndef GROUND_PICK_PARAMETERS_H
#define GROUND_PICK_PARAMETERS_H
/* Trial observation 2026-11-05, ground object, after lift collision home
   and nominal 10 mm backoff. Revalidate after changing home or camera mount.
   Pulse output and successful positioning do not prove an object is held. */
#define GROUND_PICK_LIFT_MM 130.0f
#define GROUND_PICK_JAW_CLOSE_US 2450
#define GROUND_PICK_CENTER_X 646u
#define GROUND_PICK_CENTER_Y 318u
/* Preliminary visual alignment observations, 2026-11-05.
   ARM +5 deg: (652,247) -> (706,248), delta (+54,+1) pixels.
   TRIM +2 mm: T=8 -> 10 mm, (648,279) -> (647,288), delta (-1,+9).
   Screen showed AXIS BUSY in the TRIM starting frame; repeat after settling.
   Local estimates only; do not treat as calibrated global mapping or enable
   automatic movement from these single samples. */
#define GROUND_PICK_ARM_TEST_DEG 5.0f
#define GROUND_PICK_ARM_DX_PX 54.0f
#define GROUND_PICK_ARM_DY_PX 1.0f
#define GROUND_PICK_TRIM_TEST_MM 2.0f
#define GROUND_PICK_TRIM_DX_PX (-1.0f)
#define GROUND_PICK_TRIM_DY_PX 9.0f
#endif
