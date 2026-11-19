#pragma once

// Rectilinear (pinhole) camera model shared by the overlay projection and the
// camera motion detector, so radar bearings and visual bearings agree on where
// a given direction lands in the image.
//
// A linear "pixels per degree" mapping is only correct at the image centre: at
// 25 deg off-axis with a 66 deg HFOV on a 320 px frame it misplaces a contact by
// ~6 px, enough to put a radar blip on the wrong side of a door frame.
//
// Image x is continuous: 0 is the left edge, width is the right edge, and the
// optical axis sits at width / 2. Positive bearings are to the right.

#include <math.h>
#include <stdbool.h>

#define CAMERA_MODEL_PI 3.14159265358979323846f
#define CAMERA_MODEL_DEG_TO_RAD (CAMERA_MODEL_PI / 180.0f)
#define CAMERA_MODEL_RAD_TO_DEG (180.0f / CAMERA_MODEL_PI)

// Bearings at or beyond this angle are behind/alongside the image plane and
// can never be on screen.
#define CAMERA_MODEL_MAX_BEARING_DEG 89.0f

static inline float camera_model_focal_px(int width, float hfov_deg) {
    return 0.5f * (float)width / tanf(0.5f * hfov_deg * CAMERA_MODEL_DEG_TO_RAD);
}

// Project a horizontal bearing into image x. Returns false when the bearing is
// too far off-axis to project; *x_px is then pushed well outside the frame on
// the matching side so callers can still pick an edge.
static inline bool camera_model_bearing_to_x(
    float bearing_deg, int width, float hfov_deg, float *x_px) {
    const float centre = 0.5f * (float)width;
    if (fabsf(bearing_deg) >= CAMERA_MODEL_MAX_BEARING_DEG) {
        *x_px = bearing_deg > 0.0f ? 4.0f * (float)width : -3.0f * (float)width;
        return false;
    }
    *x_px = centre +
        camera_model_focal_px(width, hfov_deg) * tanf(bearing_deg * CAMERA_MODEL_DEG_TO_RAD);
    return true;
}

static inline float camera_model_x_to_bearing(float x_px, int width, float hfov_deg) {
    const float centre = 0.5f * (float)width;
    return atanf((x_px - centre) / camera_model_focal_px(width, hfov_deg)) *
        CAMERA_MODEL_RAD_TO_DEG;
}
