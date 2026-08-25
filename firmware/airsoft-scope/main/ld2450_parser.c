// Pure LD2450 frame parser -- no ESP-IDF dependencies (host-testable).
#include <string.h>

#include "ld2450_parser.h"

static const uint8_t LD2450_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t LD2450_FOOTER[2] = {0x55, 0xCC};

// LD2450 encodes X/Y/speed as sign-magnitude where bit15 is the sign flag
// (1 = positive, 0 = negative). Returns the signed value (mm, or cm/s).
static int16_t ld2450_signed(uint16_t raw) {
    const int16_t magnitude = (int16_t)(raw & 0x7FFF);
    return (raw & 0x8000) ? magnitude : (int16_t)(-magnitude);
}

static void emit_frame(const ld2450_parser_t *parser, const uint8_t *targets) {
    airsoft_radar_frame_t frame = {0};

    uint8_t count = 0;
    for (int i = 0; i < LD2450_TARGETS && count < AIRSOFT_MAX_TARGETS; ++i) {
        const uint8_t *t = targets + i * 8;
        const uint16_t x_raw = (uint16_t)(t[0] | (t[1] << 8));
        const uint16_t y_raw = (uint16_t)(t[2] | (t[3] << 8));
        // bytes 4-5 speed (cm/s), bytes 6-7 distance resolution -- unused for now.
        if (x_raw == 0 && y_raw == 0) {
            continue;  // empty target slot
        }
        frame.targets[count].x_m = (float)ld2450_signed(x_raw) / 1000.0f;
        frame.targets[count].y_m = (float)ld2450_signed(y_raw) / 1000.0f;
        frame.targets[count].valid = true;
        ++count;
    }
    frame.count = count;

    if (parser->callback) {
        parser->callback(&frame, parser->context);
    }
}

static bool header_at(const uint8_t *buf, size_t len, size_t offset) {
    return offset + 4 <= len &&
           buf[offset] == LD2450_HEADER[0] &&
           buf[offset + 1] == LD2450_HEADER[1] &&
           buf[offset + 2] == LD2450_HEADER[2] &&
           buf[offset + 3] == LD2450_HEADER[3];
}

static void shift_parser(ld2450_parser_t *parser, size_t count) {
    if (count >= parser->len) {
        parser->len = 0;
        return;
    }
    memmove(parser->buf, parser->buf + count, parser->len - count);
    parser->len -= count;
}

void ld2450_parser_init(
    ld2450_parser_t *parser,
    ld2450_parser_frame_cb_t callback,
    void *context) {
    if (!parser) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->context = context;
}

void ld2450_parser_feed(ld2450_parser_t *parser, const uint8_t *data, size_t length) {
    if (!parser || !data) {
        return;
    }
    for (size_t i = 0; i < length; ++i) {
        if (parser->len >= LD2450_PARSER_BUF_SIZE) {
            shift_parser(parser, 1);
        }
        parser->buf[parser->len++] = data[i];

        while (parser->len >= LD2450_FRAME_LEN) {
            size_t start = 0;
            bool found = false;
            for (; start + 4 <= parser->len; ++start) {
                if (header_at(parser->buf, parser->len, start)) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                // No header yet; keep only a possible partial header tail.
                if (parser->len > 3) {
                    shift_parser(parser, parser->len - 3);
                }
                break;
            }
            if (start > 0) {
                shift_parser(parser, start);
            }
            if (parser->len < LD2450_FRAME_LEN) {
                break;  // wait for the rest of the frame
            }
            if (parser->buf[LD2450_FRAME_LEN - 2] != LD2450_FOOTER[0] ||
                parser->buf[LD2450_FRAME_LEN - 1] != LD2450_FOOTER[1]) {
                shift_parser(parser, 4);  // false header, skip past it
                continue;
            }
            emit_frame(parser, parser->buf + 4);
            shift_parser(parser, LD2450_FRAME_LEN);
        }
    }
}
