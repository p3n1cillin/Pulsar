#pragma once

#include <stddef.h>
#include <stdint.h>

#include "scope_types.h"

// Pure LD2450 byte-stream parser. Free of ESP-IDF dependencies so it can be
// unit-tested on the host; ld2450.c owns the UART reader and timestamps.
#define LD2450_PARSER_BUF_SIZE 256
#define LD2450_FRAME_LEN 30  // AA FF 03 00 | 3x8 target bytes | 55 CC
#define LD2450_TARGETS 3

typedef void (*ld2450_parser_frame_cb_t)(const airsoft_radar_frame_t *frame, void *context);

typedef struct {
    ld2450_parser_frame_cb_t callback;
    void *context;
    uint8_t buf[LD2450_PARSER_BUF_SIZE];
    size_t len;
} ld2450_parser_t;

void ld2450_parser_init(
    ld2450_parser_t *parser,
    ld2450_parser_frame_cb_t callback,
    void *context);

// Feed raw UART bytes into the parser. The callback fires once per complete,
// checksum-free-validated LD2450 frame found in the stream; decoded frames are
// delivered with updated_us == 0 (the caller stamps the receive time).
void ld2450_parser_feed(ld2450_parser_t *parser, const uint8_t *data, size_t length);
