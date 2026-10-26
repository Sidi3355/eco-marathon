// Race and vehicle parameters. Everything here is an assumption until the team
// confirms it; see "Information needed for integration" in the proposal.
#pragma once
#include <stdint.h>

// Wheel: loaded rolling circumference with the driver aboard, measured by
// pushing the car over 10 m and counting pulses.
constexpr float WHEEL_CIRC_M     = 1.60f;   // ~20 inch wheel, to be measured
constexpr float MAGNETS_PER_REV  = 4.0f;

// Track (placeholder values, set per event)
constexpr double START_LAT       = 51.5000;
constexpr double START_LON       = -0.1200;
constexpr float  START_RADIUS_M  = 12.0f;   // geofence radius for lap detection
constexpr float  LAP_M           = 1500.0f;
constexpr uint8_t TOTAL_LAPS     = 10;
constexpr uint32_t TIME_LIMIT_S  = 36 * 60; // attempt time limit
constexpr float  MIN_AVG_KMH     = (TOTAL_LAPS * LAP_M / 1000.0f) / (TIME_LIMIT_S / 3600.0f);
constexpr uint32_t TARGET_LAP_MS = (uint32_t)(TIME_LIMIT_S * 1000.0f / TOTAL_LAPS * 0.97f); // 3 % safety
constexpr uint32_t MIN_LAP_MS    = 60 * 1000;  // ignore lap triggers closer than this

// Pace cue thresholds, in seconds of slack on the projected finish time
constexpr float PACE_HIGH_S = 60.0f;   // more slack than this: coast
constexpr float PACE_LOW_S  = 20.0f;   // less slack than this: burn
constexpr float PACE_HYST_S = 10.0f;

// Strategy plan: where on the lap each burn or coast starts, in metres from
// the start line. Flat-track placeholder from the simulator; the real plan
// comes from practice-run logs on the actual circuit (hills move the burns).
struct PlanStep { float fromM; bool burn; };
constexpr PlanStep PLAN[] = {
  {   0.0f, true  },   // burn out of the start line
  { 100.0f, false },   // long glide
  { 750.0f, true  },   // second burn
  { 850.0f, false },   // glide to the line
};
constexpr int PLAN_STEPS = sizeof(PLAN) / sizeof(PLAN[0]);
