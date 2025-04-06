#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>  // for STM32 compatibility

// Protocol markers
constexpr uint8_t START_MARKER = '<';
constexpr uint8_t END_MARKER = '>';

// Message size (including start and end markers)
constexpr int MESSAGE_SIZE = 7;

// Serial settings
constexpr uint32_t SERIAL_BAUD_RATE = 115200;
constexpr uint8_t SERIAL_DATA_BITS = 8;
constexpr uint8_t SERIAL_STOP_BITS = 1;
constexpr uint8_t SERIAL_PARITY_NONE = 0; // define values to share meaning

// Buffer size for incoming data (desktop only)
constexpr int HOLDING_BUFFER_SIZE = 1000;

// Byte layout:
// [0] = START_MARKER
// [1..4] = float as 4 bytes (little-endian)
// [5] = checksum (sum of [1] to [4])
// [6] = END_MARKER

// Optional helper to calculate checksum
inline uint8_t calculateChecksum(const uint8_t *data, int length) {
    uint8_t sum = 0;
    for (int i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum;
}

#endif // SERIAL_PROTOCOL_H
