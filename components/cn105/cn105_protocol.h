/// cn105_protocol.h — Pure protocol functions for the Mitsubishi CN105 UART protocol.
/// Role: Decoupled, testable logic for checksum, temperature encoding/decoding, and byte-map lookups.
/// Deps: <cstdint>, <cmath>, <cstring>, <optional> (no ESPHome dependency)
#pragma once

#include <cstdint>
#include <cctype>
#include <cmath>
#include <cstring>
#include <optional>

namespace cn105_protocol {

inline int ascii_casecmp(const char* lhs, const char* rhs) {
    if (lhs == nullptr || rhs == nullptr) {
        return lhs == rhs ? 0 : (lhs == nullptr ? -1 : 1);
    }
    while (*lhs != '\0' && *rhs != '\0') {
        const auto left = static_cast<unsigned char>(*lhs);
        const auto right = static_cast<unsigned char>(*rhs);
        const int difference = std::tolower(left) - std::tolower(right);
        if (difference != 0) {
            return difference;
        }
        ++lhs;
        ++rhs;
    }
    return static_cast<unsigned char>(*lhs) - static_cast<unsigned char>(*rhs);
}

// ════════════════════════════════════════════════════════════════
// Checksum
// ════════════════════════════════════════════════════════════════

/// Compute the CN105 packet checksum.
/// The checksum is: (0xFC - sum_of_all_bytes) & 0xFF.
/// @param bytes  Pointer to the full packet (header + payload), excluding the checksum byte itself.
/// @param len    Number of bytes to sum (packet length minus the final checksum byte).
/// @return       The expected checksum byte.
inline uint8_t checksum(const uint8_t* bytes, int len) {
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += bytes[i];
    }
    return (0xfc - sum) & 0xff;
}

/// Return the actual error code after removing the CN105 status flag.
inline uint8_t decode_error_code(uint8_t raw_status) {
    return raw_status & 0x7f;
}

/// Identify the JP ZW-series AI AUTO signature in a settings response.
inline bool is_jp_ai_auto(bool power_on, bool auto_mode, uint8_t data_13, uint8_t data_14) {
    return power_on && auto_mode && data_13 == 0x0a && data_14 == 0x02;
}

/// Encode a wide-vane position. Airflow-control mode always requires bit 7.
inline uint8_t encode_wide_vane(uint8_t position, bool adjustment, bool airflow_control) {
    return position | ((adjustment || airflow_control) ? 0x80 : 0x00);
}

/// Apply the shared vertical-vane control flag and preserve both vane bytes.
inline bool apply_vertical_vane_control(
    uint8_t* packet,
    std::optional<uint8_t> right_vane,
    std::optional<uint8_t> left_vane
) {
    if (!right_vane && !left_vane) {
        return false;
    }
    if (right_vane) {
        packet[12] = *right_vane;
    }
    if (left_vane) {
        packet[20] = *left_vane;
    }
    packet[6] |= 0x10;
    return true;
}

// ════════════════════════════════════════════════════════════════
// Temperature decoding / encoding
// ════════════════════════════════════════════════════════════════

/// Decode a temperature from the two encoding variants used by the CN105 protocol.
///
/// Encoding B (half-degree precision):
///   When enc_b != 0, temperature = (enc_b - 128) / 2.0  (range: 10.0..31.5°C typically)
///
/// Encoding A (integer precision, legacy):
///   When enc_b == 0, temperature = enc_a + offset  (offset is typically 10 for room temp)
///
/// This function unifies both branches into a single call, eliminating the need
/// for callers to manually check which encoding variant is in use.
///
/// @param enc_a   Encoding A raw byte (e.g., data[3] for room temp, data[5] for settings)
/// @param enc_b   Encoding B raw byte (e.g., data[6] for room temp, data[11] for settings)
/// @param offset  Additive offset for encoding A (10 for room temp, use TEMP_MAP for settings)
/// @return        Decoded temperature in °C as a float.
inline float decode_temperature(uint8_t enc_a, uint8_t enc_b, int offset = 10) {
    if (enc_b != 0) {
        return static_cast<float>(enc_b - 128) / 2.0f;
    }
    return static_cast<float>(enc_a) + static_cast<float>(offset);
}

/// Encode a target temperature into the encoding B format (half-degree precision).
/// Formula: byte = round(temp * 2) + 128
///
/// @param temperature  Target temperature in °C.
/// @return             The encoded byte value for encoding B.
inline uint8_t encode_temperature_b(float temperature) {
    return static_cast<uint8_t>(std::round(temperature * 2.0f) + 128);
}

/// Encode a remote temperature into the two-byte format used by SET remote temp packets.
/// Byte 1 (encoding A legacy): round(temp * 2) - 16
/// Byte 2 (encoding B):        round(temp * 2) + 128
///
/// @param temperature  Remote temperature in °C.
/// @param[out] enc_a   Output: encoding A byte for the remote temp packet.
/// @param[out] enc_b   Output: encoding B byte for the remote temp packet.
inline void encode_remote_temperature(float temperature, uint8_t& enc_a, uint8_t& enc_b) {
    float rounded = std::round(temperature * 2.0f);
    enc_a = static_cast<uint8_t>(rounded - 16);
    enc_b = static_cast<uint8_t>(rounded + 128);
}

// ════════════════════════════════════════════════════════════════
// Byte-map lookups — fallback variant (legacy compatibility)
// ════════════════════════════════════════════════════════════════

/// Look up a mapped value from a parallel byte-map / value-map pair.
/// Scans the byteMap for a matching byteValue and returns the corresponding entry in valuesMap.
/// Returns valuesMap[0] if no match is found (safe fallback for protocol continuity).
///
/// @tparam T         Value type (typically const char* or int).
/// @param valuesMap  Array of mapped values.
/// @param byteMap    Array of protocol byte codes (same length as valuesMap).
/// @param len        Number of entries in both arrays.
/// @param byteValue  The raw protocol byte to look up.
/// @return           The corresponding value, or valuesMap[0] if not found.
template <typename T>
inline T lookup_value(const T valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue) {
    for (int i = 0; i < len; i++) {
        if (byteMap[i] == byteValue) {
            return valuesMap[i];
        }
    }
    return valuesMap[0];
}

/// Look up the index of a value in a value-map.
/// Returns the index (usable as an offset into the parallel byte-map), or -1 if not found.
///
/// @param valuesMap   Array of mapped values (int variant).
/// @param len         Number of entries.
/// @param lookupValue The value to search for.
/// @return            Index of the match, or -1 if not found.
inline int lookup_index(const int valuesMap[], int len, int lookupValue) {
    for (int i = 0; i < len; i++) {
        if (valuesMap[i] == lookupValue) {
            return i;
        }
    }
    return -1;
}

/// Look up the index of a string value in a value-map (case-insensitive).
///
/// @param valuesMap   Array of mapped string values.
/// @param len         Number of entries.
/// @param lookupValue The string to search for.
/// @return            Index of the match, or -1 if not found.
inline int lookup_index(const char* valuesMap[], int len, const char* lookupValue) {
    if (lookupValue == nullptr) {
        return -1;
    }
    for (int i = 0; i < len; i++) {
        if (ascii_casecmp(valuesMap[i], lookupValue) == 0) {
            return i;
        }
    }
    return -1;
}

// ════════════════════════════════════════════════════════════════
// Byte-map lookups — std::optional variant (graceful degradation)
// ════════════════════════════════════════════════════════════════

/// Look up a mapped value, returning std::nullopt on miss.
/// Unlike lookup_value(), this does NOT silently fall back to index 0.
/// Callers can decide how to handle unknown bytes (keep previous value, log, etc.).
///
/// @tparam T         Value type (typically const char* or int).
/// @param valuesMap  Array of mapped values.
/// @param byteMap    Array of protocol byte codes (same length as valuesMap).
/// @param len        Number of entries in both arrays.
/// @param byteValue  The raw protocol byte to look up.
/// @return           The corresponding value, or std::nullopt if not found.
template <typename T>
inline std::optional<T> lookup_value_opt(const T valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue) {
    for (int i = 0; i < len; i++) {
        if (byteMap[i] == byteValue) {
            return valuesMap[i];
        }
    }
    return std::nullopt;
}

/// Look up the index of a value in a value-map, returning std::nullopt on miss.
///
/// @param valuesMap   Array of mapped values (int variant).
/// @param len         Number of entries.
/// @param lookupValue The value to search for.
/// @return            Index of the match, or std::nullopt if not found.
inline std::optional<int> lookup_index_opt(const int valuesMap[], int len, int lookupValue) {
    for (int i = 0; i < len; i++) {
        if (valuesMap[i] == lookupValue) {
            return i;
        }
    }
    return std::nullopt;
}

/// Look up the index of a string value (case-insensitive), returning std::nullopt on miss.
///
/// @param valuesMap   Array of mapped string values.
/// @param len         Number of entries.
/// @param lookupValue The string to search for.
/// @return            Index of the match, or std::nullopt if not found.
inline std::optional<int> lookup_index_opt(const char* valuesMap[], int len, const char* lookupValue) {
    if (lookupValue == nullptr) {
        return std::nullopt;
    }
    for (int i = 0; i < len; i++) {
        if (ascii_casecmp(valuesMap[i], lookupValue) == 0) {
            return i;
        }
    }
    return std::nullopt;
}

}  // namespace cn105_protocol
