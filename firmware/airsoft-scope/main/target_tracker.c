#include <math.h>
#include <string.h>

#include "target_tracker.h"

#define AIRSOFT_PI 3.14159265358979323846f
#define RAD_TO_DEG (180.0f / AIRSOFT_PI)
#define DEG_TO_RAD (AIRSOFT_PI / 180.0f)

typedef struct {
    uint8_t track;
    uint8_t observation;
    float cost;
} association_pair_t;

typedef struct {
    float x_m;
    float y_m;
    float range_m;
    float bearing_deg;
    bool valid;
} radar_observation_t;

static float clamp_float(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float normalize_angle(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

static uint64_t elapsed_us(uint64_t now_us, uint64_t then_us) {
    return now_us >= then_us ? now_us - then_us : 0;
}

static float elapsed_seconds(uint64_t now_us, uint64_t then_us) {
    return (float)elapsed_us(now_us, then_us) / 1000000.0f;
}

static void sort_pairs(association_pair_t *pairs, int count) {
    for (int i = 1; i < count; ++i) {
        const association_pair_t value = pairs[i];
        int j = i - 1;
        while (j >= 0 && pairs[j].cost > value.cost) {
            pairs[j + 1] = pairs[j];
            --j;
        }
        pairs[j + 1] = value;
    }
}

static bool id_is_active(const target_tracker_t *tracker, uint8_t id) {
    for (int i = 0; i < AIRSOFT_MAX_TRACKS; ++i) {
        if (tracker->slots[i].active && tracker->slots[i].id == id) {
            return true;
        }
    }
    return false;
}

static uint8_t allocate_id(target_tracker_t *tracker) {
    for (int attempt = 0; attempt < UINT8_MAX; ++attempt) {
        uint8_t id = tracker->next_id++;
        if (id == 0) {
            id = tracker->next_id++;
        }
        if (tracker->next_id == 0) {
            tracker->next_id = 1;
        }
        if (!id_is_active(tracker, id)) {
            return id;
        }
    }
    return 0;
}

static int allocate_slot(target_tracker_t *tracker, uint64_t now_us, bool radar_priority) {
    for (int i = 0; i < AIRSOFT_MAX_TRACKS; ++i) {
        if (!tracker->slots[i].active) {
            return i;
        }
    }

    // Visual noise must never evict a real radar-backed track. A new radar
    // contact may replace the stalest/coasting slot when the table is full.
    if (!radar_priority) {
        return -1;
    }
    int candidate = -1;
    float lowest_score = 1000.0f;
    for (int i = 0; i < AIRSOFT_MAX_TRACKS; ++i) {
        target_tracker_slot_t *slot = &tracker->slots[i];
        const bool radar_fresh = slot->radar_seen_us != 0 &&
            elapsed_us(now_us, slot->radar_seen_us) <=
                (uint64_t)tracker->config.radar_hold_ms * 1000ULL;
        const float age_penalty = clamp_float(
            elapsed_seconds(now_us, slot->last_seen_us) /
                ((float)tracker->config.coast_ms / 1000.0f),
            0.0f,
            1.0f);
        const float score = slot->confidence - age_penalty + (radar_fresh ? 2.0f : 0.0f);
        if (score < lowest_score) {
            lowest_score = score;
            candidate = i;
        }
    }
    return candidate;
}

static void start_radar_track(
    target_tracker_t *tracker,
    int index,
    const radar_observation_t *observation,
    uint64_t now_us) {
    target_tracker_slot_t *slot = &tracker->slots[index];
    memset(slot, 0, sizeof(*slot));
    slot->active = true;
    slot->range_valid = true;
    slot->id = allocate_id(tracker);
    slot->radar_hits = 1;
    slot->x_m = observation->x_m;
    slot->y_m = observation->y_m;
    slot->bearing_deg = observation->bearing_deg;
    slot->confidence = 0.55f;
    slot->created_us = now_us;
    slot->last_predict_us = now_us;
    slot->last_seen_us = now_us;
    slot->radar_seen_us = now_us;
}

static void start_camera_track(
    target_tracker_t *tracker,
    int index,
    const airsoft_camera_detection_t *observation,
    uint64_t now_us) {
    target_tracker_slot_t *slot = &tracker->slots[index];
    memset(slot, 0, sizeof(*slot));
    slot->active = true;
    slot->range_valid = false;
    slot->id = allocate_id(tracker);
    slot->camera_hits = 1;
    slot->bearing_deg = observation->bearing_deg;
    slot->confidence = 0.25f + observation->confidence * 0.25f;
    slot->image_x = observation->center_x;
    slot->image_y = observation->center_y;
    slot->image_width = observation->width;
    slot->image_height = observation->height;
    slot->created_us = now_us;
    slot->last_predict_us = now_us;
    slot->last_seen_us = now_us;
    slot->camera_seen_us = now_us;
}

static void predict_tracks(target_tracker_t *tracker, uint64_t now_us) {
    const uint64_t coast_us = (uint64_t)tracker->config.coast_ms * 1000ULL;
    for (int i = 0; i < AIRSOFT_MAX_TRACKS; ++i) {
        target_tracker_slot_t *slot = &tracker->slots[i];
        if (!slot->active) continue;
        if (elapsed_us(now_us, slot->last_seen_us) > coast_us) {
            memset(slot, 0, sizeof(*slot));
            continue;
        }

        const float dt = clamp_float(elapsed_seconds(now_us, slot->last_predict_us), 0.0f, 0.5f);
        if (slot->range_valid) {
            slot->x_m += slot->velocity_x_mps * dt;
            slot->y_m += slot->velocity_y_mps * dt;
            slot->bearing_deg = atan2f(slot->x_m, slot->y_m) * RAD_TO_DEG;
        } else {
            slot->bearing_deg = normalize_angle(
                slot->bearing_deg + slot->bearing_velocity_dps * dt);
        }
        slot->last_predict_us = now_us;
    }
}

static int build_radar_observations(
    const target_tracker_t *tracker,
    const airsoft_radar_frame_t *frame,
    radar_observation_t *out) {
    int count = 0;
    if (!frame) return 0;

    for (uint8_t i = 0; i < frame->count && count < AIRSOFT_MAX_TARGETS; ++i) {
        if (!frame->targets[i].valid) continue;
        const float raw_x = frame->targets[i].x_m;
        const float raw_y = frame->targets[i].y_m;
        const float range = hypotf(raw_x, raw_y);
        float bearing = atan2f(raw_x, raw_y) * RAD_TO_DEG;
        if (range < tracker->config.min_range_m ||
            range > tracker->config.max_range_m ||
            fabsf(bearing) > tracker->config.radar_half_fov_deg) {
            continue;
        }
        bearing = normalize_angle(bearing + tracker->config.radar_boresight_deg);
        out[count].range_m = range;
        out[count].bearing_deg = bearing;
        out[count].x_m = sinf(bearing * DEG_TO_RAD) * range;
        out[count].y_m = cosf(bearing * DEG_TO_RAD) * range;
        out[count].valid = true;
        ++count;
    }
    return count;
}

static void apply_radar_observation(
    target_tracker_slot_t *slot,
    const radar_observation_t *observation,
    uint64_t now_us) {
    if (!slot->range_valid) {
        slot->x_m = observation->x_m;
        slot->y_m = observation->y_m;
        slot->velocity_x_mps = 0.0f;
        slot->velocity_y_mps = 0.0f;
        slot->range_valid = true;
    } else {
        const float residual_x = observation->x_m - slot->x_m;
        const float residual_y = observation->y_m - slot->y_m;
        const float dt = clamp_float(elapsed_seconds(now_us, slot->radar_seen_us), 0.05f, 0.5f);
        slot->x_m += residual_x * 0.72f;
        slot->y_m += residual_y * 0.72f;
        slot->velocity_x_mps = clamp_float(
            slot->velocity_x_mps + residual_x * 0.16f / dt, -8.0f, 8.0f);
        slot->velocity_y_mps = clamp_float(
            slot->velocity_y_mps + residual_y * 0.16f / dt, -8.0f, 8.0f);
    }
    slot->bearing_deg = atan2f(slot->x_m, slot->y_m) * RAD_TO_DEG;
    slot->confidence = clamp_float(slot->confidence + 0.22f, 0.0f, 1.0f);
    ++slot->radar_hits;
    slot->last_seen_us = now_us;
    slot->radar_seen_us = now_us;
}

static void associate_radar(
    target_tracker_t *tracker,
    const airsoft_radar_frame_t *frame,
    uint64_t now_us) {
    radar_observation_t observations[AIRSOFT_MAX_TARGETS] = {0};
    const int observation_count = build_radar_observations(tracker, frame, observations);
    association_pair_t pairs[AIRSOFT_MAX_TRACKS * AIRSOFT_MAX_TARGETS];
    int pair_count = 0;

    for (int t = 0; t < AIRSOFT_MAX_TRACKS; ++t) {
        const target_tracker_slot_t *slot = &tracker->slots[t];
        if (!slot->active) continue;
        for (int o = 0; o < observation_count; ++o) {
            float cost;
            if (slot->range_valid) {
                cost = hypotf(
                    observations[o].x_m - slot->x_m,
                    observations[o].y_m - slot->y_m);
                if (cost > tracker->config.radar_position_gate_m) continue;
            } else {
                const float angle_error = fabsf(normalize_angle(
                    observations[o].bearing_deg - slot->bearing_deg));
                if (angle_error > tracker->config.camera_bearing_gate_deg) continue;
                cost = tracker->config.radar_position_gate_m *
                    (0.35f + 0.65f * angle_error /
                        tracker->config.camera_bearing_gate_deg);
            }
            pairs[pair_count++] = (association_pair_t){
                .track = (uint8_t)t,
                .observation = (uint8_t)o,
                .cost = cost,
            };
        }
    }

    sort_pairs(pairs, pair_count);
    bool track_used[AIRSOFT_MAX_TRACKS] = {0};
    bool observation_used[AIRSOFT_MAX_TARGETS] = {0};
    for (int i = 0; i < pair_count; ++i) {
        const int t = pairs[i].track;
        const int o = pairs[i].observation;
        if (track_used[t] || observation_used[o]) continue;
        apply_radar_observation(&tracker->slots[t], &observations[o], now_us);
        track_used[t] = true;
        observation_used[o] = true;
    }

    for (int o = 0; o < observation_count; ++o) {
        if (observation_used[o]) continue;
        const int slot = allocate_slot(tracker, now_us, true);
        if (slot >= 0) {
            start_radar_track(tracker, slot, &observations[o], now_us);
        }
    }
}

static void apply_camera_observation(
    target_tracker_slot_t *slot,
    const airsoft_camera_detection_t *observation,
    uint64_t now_us) {
    const float residual = normalize_angle(observation->bearing_deg - slot->bearing_deg);
    const float dt = clamp_float(elapsed_seconds(now_us, slot->camera_seen_us), 0.05f, 0.5f);
    if (slot->range_valid) {
        const float range = hypotf(slot->x_m, slot->y_m);
        slot->bearing_deg = normalize_angle(slot->bearing_deg + residual * 0.45f);
        slot->x_m = sinf(slot->bearing_deg * DEG_TO_RAD) * range;
        slot->y_m = cosf(slot->bearing_deg * DEG_TO_RAD) * range;
    } else {
        slot->bearing_deg = normalize_angle(slot->bearing_deg + residual * 0.65f);
        slot->bearing_velocity_dps = clamp_float(
            slot->bearing_velocity_dps + residual * 0.15f / dt, -90.0f, 90.0f);
    }
    slot->image_x = observation->center_x;
    slot->image_y = observation->center_y;
    slot->image_width = observation->width;
    slot->image_height = observation->height;
    slot->confidence = clamp_float(
        slot->confidence + 0.12f + observation->confidence * 0.08f,
        0.0f,
        1.0f);
    ++slot->camera_hits;
    slot->last_seen_us = now_us;
    slot->camera_seen_us = now_us;
}

static void associate_camera(
    target_tracker_t *tracker,
    const airsoft_camera_frame_t *frame,
    uint64_t now_us) {
    if (!frame || frame->global_motion) return;
    association_pair_t pairs[AIRSOFT_MAX_TRACKS * AIRSOFT_MAX_CAMERA_DETECTIONS];
    int pair_count = 0;
    const uint8_t observation_count = frame->count < AIRSOFT_MAX_CAMERA_DETECTIONS
        ? frame->count
        : AIRSOFT_MAX_CAMERA_DETECTIONS;

    for (int t = 0; t < AIRSOFT_MAX_TRACKS; ++t) {
        const target_tracker_slot_t *slot = &tracker->slots[t];
        if (!slot->active) continue;
        for (uint8_t o = 0; o < observation_count; ++o) {
            if (!frame->detections[o].valid) continue;
            const float error = fabsf(normalize_angle(
                frame->detections[o].bearing_deg - slot->bearing_deg));
            if (error > tracker->config.camera_bearing_gate_deg) continue;
            pairs[pair_count++] = (association_pair_t){
                .track = (uint8_t)t,
                .observation = o,
                // Prefer adding camera confirmation to a range-bearing radar
                // track over extending a nearby visual-only duplicate.
                .cost = error + (slot->range_valid
                    ? 0.0f
                    : tracker->config.camera_bearing_gate_deg * 0.2f),
            };
        }
    }

    sort_pairs(pairs, pair_count);
    bool track_used[AIRSOFT_MAX_TRACKS] = {0};
    bool observation_used[AIRSOFT_MAX_CAMERA_DETECTIONS] = {0};
    for (int i = 0; i < pair_count; ++i) {
        const int t = pairs[i].track;
        const int o = pairs[i].observation;
        if (track_used[t] || observation_used[o]) continue;
        apply_camera_observation(&tracker->slots[t], &frame->detections[o], now_us);
        track_used[t] = true;
        observation_used[o] = true;
    }

    // Frame differencing can split the two edges of one moving player into
    // nearby blobs. Absorb those near an already matched track instead of
    // minting duplicate IDs.
    const float duplicate_gate = fminf(
        4.0f, tracker->config.camera_bearing_gate_deg * 0.4f);
    for (uint8_t o = 0; o < observation_count; ++o) {
        if (observation_used[o] || !frame->detections[o].valid) continue;
        for (int t = 0; t < AIRSOFT_MAX_TRACKS; ++t) {
            if (!track_used[t]) continue;
            const float error = fabsf(normalize_angle(
                frame->detections[o].bearing_deg - tracker->slots[t].bearing_deg));
            if (error <= duplicate_gate) {
                observation_used[o] = true;
                break;
            }
        }
    }

    for (uint8_t o = 0; o < observation_count; ++o) {
        if (observation_used[o] || !frame->detections[o].valid) continue;
        const int slot = allocate_slot(tracker, now_us, false);
        if (slot >= 0) {
            start_camera_track(tracker, slot, &frame->detections[o], now_us);
        }
    }
}

static void make_snapshot(
    const target_tracker_t *tracker,
    uint64_t now_us,
    airsoft_track_frame_t *out) {
    memset(out, 0, sizeof(*out));
    out->updated_us = now_us;
    const uint64_t radar_hold_us = (uint64_t)tracker->config.radar_hold_ms * 1000ULL;
    const uint64_t camera_hold_us = (uint64_t)tracker->config.camera_hold_ms * 1000ULL;
    const float coast_seconds = (float)tracker->config.coast_ms / 1000.0f;

    for (int i = 0; i < AIRSOFT_MAX_TRACKS && out->count < AIRSOFT_MAX_TRACKS; ++i) {
        const target_tracker_slot_t *slot = &tracker->slots[i];
        const bool confirmed = slot->radar_hits > 0 || slot->camera_hits >= 2;
        if (!slot->active || !confirmed) continue;

        airsoft_track_t *track = &out->tracks[out->count++];
        const bool radar_fresh = slot->radar_seen_us != 0 &&
            elapsed_us(now_us, slot->radar_seen_us) <= radar_hold_us;
        const bool camera_fresh = slot->camera_seen_us != 0 &&
            elapsed_us(now_us, slot->camera_seen_us) <= camera_hold_us;
        if (radar_fresh) track->source_flags |= AIRSOFT_TRACK_SOURCE_RADAR;
        if (camera_fresh) track->source_flags |= AIRSOFT_TRACK_SOURCE_CAMERA;
        if (!radar_fresh && !camera_fresh) {
            track->source_flags = AIRSOFT_TRACK_SOURCE_PREDICTED;
        }

        const float since_seen = elapsed_seconds(now_us, slot->last_seen_us);
        const float coast_factor = 1.0f - 0.7f * clamp_float(
            since_seen / coast_seconds, 0.0f, 1.0f);
        track->id = slot->id;
        track->x_m = slot->x_m;
        track->y_m = slot->y_m;
        track->velocity_x_mps = slot->velocity_x_mps;
        track->velocity_y_mps = slot->velocity_y_mps;
        track->range_m = slot->range_valid ? hypotf(slot->x_m, slot->y_m) : 0.0f;
        track->bearing_deg = slot->bearing_deg;
        track->confidence = slot->confidence * coast_factor;
        track->image_x = slot->image_x;
        track->image_y = slot->image_y;
        track->image_width = slot->image_width;
        track->image_height = slot->image_height;
        track->age_ms = (uint32_t)(elapsed_us(now_us, slot->created_us) / 1000ULL);
        track->range_valid = slot->range_valid;
        track->valid = true;
    }
}

void target_tracker_init(
    target_tracker_t *tracker,
    const target_tracker_config_t *config) {
    if (!tracker) return;
    memset(tracker, 0, sizeof(*tracker));
    if (config) {
        tracker->config = *config;
    }
    tracker->next_id = 1;
}

void target_tracker_update(
    target_tracker_t *tracker,
    const airsoft_radar_frame_t *radar,
    const airsoft_camera_frame_t *camera,
    uint64_t now_us,
    airsoft_track_frame_t *out) {
    if (!tracker || !out) return;
    predict_tracks(tracker, now_us);
    if (radar) {
        associate_radar(tracker, radar, now_us);
    }
    if (camera) {
        associate_camera(tracker, camera, now_us);
    }
    make_snapshot(tracker, now_us, out);
}
