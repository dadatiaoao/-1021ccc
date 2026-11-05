#ifndef DEPARTURE_CONFIG_H
#define DEPARTURE_CONFIG_H
/* User route, 2026-11-01: zone 2, body-forward 100 cm, no turn.
 * These are command targets, not a claim of calibrated ground accuracy.
 * This isolated trial does not implement lidar obstacle detection. */
#define DEPARTURE_FORWARD_MM 1000.0f
#define DEPARTURE_SPEED_MM_S 60.0f
#define DEPARTURE_SETTLE_MS 500u
#define DEPARTURE_NAV_TIMEOUT_MS 45000u
#define DEPARTURE_SCAN_TIMEOUT_MS 15000u
#endif
