/*
 * binary_uart_protocol.h - Binary UART Protocol Definition
 * 
 * Custom binary payload format for high-speed TWR distance reporting
 * 15 bytes per TWR message vs 110+ bytes for JSON
 * 
 * TWR frame format:
 * [0xAA][TAG_ID:2LE][ANCHOR_ID:2LE][DISTANCE:2LE][RANGE:1][FLAGS:1][X_CM:2LE][Y_CM:2LE][CHK:1][0x55]
 *
 * New-tag frame format:
 * [0xAB][TAG_ADDR64:8LE][CHK:1][0x55]
 *
 * EUI request frame format:
 * [0xAC][CMD:1][RESERVED:7][CHK:1][0x55]
 *
 * EUI response frame format:
 * [0xAD][EUI64:8LE][CHK:1][0x55]
 *
 * Tag-location frame format (mirrors send_to_pc_tag_location()'s JSON "TAG" report):
 * [0xAF][TAG_ADDR:2LE][NODE_ADDR:2LE][RANGE_NUM:1][X_CM:2LE][Y_CM:2LE][CLK_OFFSET_PPHM:2LE][CHK:1][0x55]
 */

#ifndef BINARY_UART_PROTOCOL_H
#define BINARY_UART_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Frame markers */
#define BIN_FRAME_START         0xAA
#define BIN_NEWTAG_FRAME_START  0xAB
#define BIN_EUI_REQ_FRAME_START 0xAC
#define BIN_EUI_RESP_FRAME_START 0xAD
#define BIN_TAG_LOC_FRAME_START 0xAF
#define BIN_FRAME_END           0x55
#define BIN_FRAME_SIZE          15
#define BIN_TWR_PAYLOAD_SIZE    12
#define BIN_NEWTAG_FRAME_SIZE   11
#define BIN_EUI_FRAME_SIZE      11
#define BIN_TAG_LOC_FRAME_SIZE     14
#define BIN_TAG_LOC_PAYLOAD_SIZE   11

/* EUI request commands */
#define BIN_EUI_CMD_GET         0x01

/* Flags (byte 8) */
#define BIN_FLAG_ESCORT     0x01    /* Bit 0: Escort mode active */

/*
 * Binary TWR frame structure (packed, 15 bytes)
 * All multi-byte values are little-endian
 */
typedef struct {
    uint8_t  start;             /* [0] = 0xAA */
    uint16_t tag_id;            /* [1-2] 16-bit tag short address (LE) */
    uint16_t anchor_id;         /* [3-4] 16-bit anchor short address (LE) */
    uint16_t distance_cm;       /* [5-6] Distance in cm, 0-65535 (LE) */
    uint8_t  range_num;         /* [7] Range/sequence number */
    uint8_t  flags;             /* [8] Flags (bit 0: escort) */
    int16_t  x_cm;              /* [9-10] X coordinate in cm (LE) */
    int16_t  y_cm;              /* [11-12] Y coordinate in cm (LE) */
    uint8_t  checksum;          /* [13] XOR checksum of bytes 1-12 */
    uint8_t  end;               /* [14] = 0x55 */
} __attribute__((packed)) binary_twr_frame_t;

/*
 * Binary new-tag frame structure (packed, 11 bytes)
 * The 64-bit tag address is little-endian.
 */
typedef struct {
    uint8_t  start;             /* [0] = 0xAB */
    uint64_t tag_addr64;        /* [1-8] 64-bit tag address (LE) */
    uint8_t  checksum;          /* [9] XOR checksum of bytes 1-8 */
    uint8_t  end;               /* [10] = 0x55 */
} __attribute__((packed)) binary_newtag_frame_t;

/*
 * Binary EUI response frame structure (packed, 11 bytes)
 * The 64-bit EUI is little-endian.
 */
typedef struct {
    uint8_t  start;             /* [0] = 0xAD */
    uint64_t eui64;             /* [1-8] 64-bit EUI (LE) */
    uint8_t  checksum;          /* [9] XOR checksum of bytes 1-8 */
    uint8_t  end;               /* [10] = 0x55 */
} __attribute__((packed)) binary_eui_frame_t;

/*
 * Binary tag-location frame structure (packed, 14 bytes)
 * Sent by the TAG's own report path (send_to_pc_tag_location()) - mirrors the
 * fields of its "TAG" JSON object: tag/node address, range number, X/Y
 * position and clock offset. All multi-byte values are little-endian.
 */
typedef struct {
    uint8_t  start;             /* [0] = 0xAF */
    uint16_t tag_addr;          /* [1-2] this Tag's 16-bit address (LE) */
    uint16_t node_addr;         /* [3-4] reporting Node's 16-bit address (LE) */
    uint8_t  range_num;         /* [5] Range/sequence number */
    int16_t  x_cm;               /* [6-7] X coordinate in cm (LE) */
    int16_t  y_cm;               /* [8-9] Y coordinate in cm (LE) */
    int16_t  clk_offset_pphm;    /* [10-11] Clock offset, hundredths of ppm (LE) */
    uint8_t  checksum;           /* [12] XOR checksum of bytes 1-11 */
    uint8_t  end;                /* [13] = 0x55 */
} __attribute__((packed)) binary_tag_loc_frame_t;

/*
 * Calculate XOR checksum over data buffer
 * @param data: pointer to data bytes
 * @param len: number of bytes to checksum
 * @return: XOR checksum value
 */
static inline uint8_t binary_twr_calc_checksum(const uint8_t *data, int len)
{
    uint8_t sum = 0;
    int i;
    for (i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}

/*
 * Validate received frame
 * @param frame: pointer to 15-byte frame buffer
 * @return: 1 if valid, 0 if invalid
 */
static inline int binary_twr_validate_frame(const uint8_t *frame)
{
    uint8_t checksum;
    
    /* Check frame markers */
    if (frame[0] != BIN_FRAME_START || frame[14] != BIN_FRAME_END) {
        return 0;
    }
    
    /* Check checksum (bytes 1-12) */
    checksum = binary_twr_calc_checksum(&frame[1], BIN_TWR_PAYLOAD_SIZE);
    if (checksum != frame[13]) {
        return 0;
    }
    
    return 1;
}

/*
 * Validate received new-tag frame
 * @param frame: pointer to 11-byte frame buffer
 * @return: 1 if valid, 0 if invalid
 */
static inline int binary_newtag_validate_frame(const uint8_t *frame)
{
    uint8_t checksum;

    if (frame[0] != BIN_NEWTAG_FRAME_START || frame[10] != BIN_FRAME_END) {
        return 0;
    }

    checksum = binary_twr_calc_checksum(&frame[1], 8);
    if (checksum != frame[9]) {
        return 0;
    }

    return 1;
}

/*
 * Validate received EUI request frame.
 * @param frame: pointer to 11-byte frame buffer
 * @return: 1 if valid, 0 if invalid
 */
static inline int binary_eui_validate_request_frame(const uint8_t *frame)
{
    uint8_t checksum;

    if (frame[0] != BIN_EUI_REQ_FRAME_START || frame[10] != BIN_FRAME_END) {
        return 0;
    }

    if (frame[1] != BIN_EUI_CMD_GET) {
        return 0;
    }

    checksum = binary_twr_calc_checksum(&frame[1], 8);
    if (checksum != frame[9]) {
        return 0;
    }

    return 1;
}

/*
 * Extract fields from binary frame
 * Note: All multi-byte values are little-endian
 */
static inline void binary_twr_extract(const uint8_t *frame,
                                      uint16_t *tag_id,
                                      uint16_t *anchor_id,
                                      uint16_t *distance_cm,
                                      uint8_t *range_num,
                                      uint8_t *flags,
                                      int16_t *x_cm,
                                      int16_t *y_cm)
{
    *tag_id     = frame[1] | (frame[2] << 8);
    *anchor_id  = frame[3] | (frame[4] << 8);
    *distance_cm = frame[5] | (frame[6] << 8);
    *range_num   = frame[7];
    *flags       = frame[8];
    *x_cm        = (int16_t)(frame[9] | (frame[10] << 8));
    *y_cm        = (int16_t)(frame[11] | (frame[12] << 8));
}

/*
 * Extract the 64-bit tag address from a new-tag frame.
 */
static inline uint64_t binary_newtag_extract_addr64(const uint8_t *frame)
{
    return ((uint64_t)frame[1]) |
           ((uint64_t)frame[2] << 8) |
           ((uint64_t)frame[3] << 16) |
           ((uint64_t)frame[4] << 24) |
           ((uint64_t)frame[5] << 32) |
           ((uint64_t)frame[6] << 40) |
           ((uint64_t)frame[7] << 48) |
           ((uint64_t)frame[8] << 56);
}

#ifdef __cplusplus
}
#endif

#endif /* BINARY_UART_PROTOCOL_H */
