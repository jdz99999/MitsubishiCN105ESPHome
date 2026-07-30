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

// ════════════════════════════════════════════════════════════════
// Frame geometry
// ════════════════════════════════════════════════════════════════

/// A CN105 frame is `FC | command | 01 30 10 | payload[16] | FCC`, so payload
/// index N lives at raw offset N + 5. Every payload index quoted in this file
/// matches the indices used by the official Mitsubishi clients.
inline constexpr int payload_offset(int payload_index) { return payload_index + 5; }

/// SET subtypes used by the official Kirigamine REMOTE / Kiriweb encoders.
static constexpr uint8_t SET_SUBTYPE_BASIC = 0x01;      // power, mode, temp, fan, right vane, wide vane
static constexpr uint8_t SET_SUBTYPE_RUN_STATE = 0x08;  // humidity, energy saving, special airflow, buzzer
static constexpr uint8_t SET_SUBTYPE_TOUCH_FLOW = 0x31; // Touch Flow / ventilation-assist coordinates
static constexpr uint8_t SET_SUBTYPE_VANE_EXT = 0x33;   // stopped-state sensing, left vane, Long airflow

/// Payload 15 of a subtype 0x01 SET is the command-origin marker: 0x41 for a
/// local/inside command, 0x42 for a cloud/outside command. It is NOT a vane
/// byte — the left vertical vane is written through subtype 0x33 instead.
static constexpr uint8_t SET_ORIGIN_INSIDE = 0x41;
static constexpr uint8_t SET_ORIGIN_OUTSIDE = 0x42;

/// Stamp the command-origin marker on a subtype 0x01 SET packet.
inline void set_command_origin(uint8_t* packet, uint8_t origin = SET_ORIGIN_INSIDE) {
    packet[payload_offset(15)] = origin;
}

/// Encode a wide-vane position. Airflow-control mode always requires bit 7.
inline uint8_t encode_wide_vane(uint8_t position, bool adjustment, bool airflow_control) {
    return position | ((adjustment || airflow_control) ? 0x80 : 0x00);
}

/// Apply the right vertical-vane byte (payload 7) and its control flag.
/// The left vane is deliberately absent: writing it here would overwrite the
/// origin marker at payload 15.
inline bool apply_vertical_vane_control(uint8_t* packet, std::optional<uint8_t> right_vane) {
    if (!right_vane) {
        return false;
    }
    packet[payload_offset(7)] = *right_vane;
    packet[payload_offset(1)] |= 0x10;
    return true;
}

// ════════════════════════════════════════════════════════════════
// Operating mode (subtype 0x02 payload 4)
// ════════════════════════════════════════════════════════════════

/// Direction the unit has chosen while running one of the automatic modes.
enum class AutoDirection : uint8_t { NONE = 0, HEATING = 1, COOLING = 2 };

inline const char* auto_direction_to_string(AutoDirection direction) {
    switch (direction) {
    case AutoDirection::HEATING: return "AUTO_HEATING";
    case AutoDirection::COOLING: return "AUTO_COOLING";
    default: return "NONE";
    }
}

struct ModeDecode {
    bool valid = false;              // false → caller keeps its previous value
    uint8_t mode = 0;                // normalised MODE[] byte (0x01/0x02/0x03/0x07/0x08)
    bool iSee = false;               // i-See/sensing bit (0x08) of the raw byte
    AutoDirection auto_direction = AutoDirection::NONE;
};

/// Decode the raw operating-mode byte using the official client's table.
/// Raw 0x19/0x1B are the automatic modes that report their own direction; both
/// normalise to AUTO so Home Assistant still sees a single AUTO mode.
/// Raw 0x00 is left undecoded on purpose: the official client maps it to DRY,
/// but no capture confirms that for the models this firmware supports.
/// Raw 0x0F is not in the official table either; it is kept here so the older
/// "bit 0x08 means i-See" behaviour for FAN mode is preserved.
inline ModeDecode decode_mode_byte(uint8_t raw) {
    ModeDecode result{};
    switch (raw) {
    case 0x01: case 0x09: result = { true, 0x01, raw == 0x09, AutoDirection::NONE }; break;
    case 0x02: case 0x0a: case 0x0c: result = { true, 0x02, raw != 0x02, AutoDirection::NONE }; break;
    case 0x03: case 0x0b: result = { true, 0x03, raw == 0x0b, AutoDirection::NONE }; break;
    case 0x07: case 0x0f: result = { true, 0x07, raw == 0x0f, AutoDirection::NONE }; break;
    case 0x08: result = { true, 0x08, false, AutoDirection::NONE }; break;
    case 0x19: result = { true, 0x08, true, AutoDirection::HEATING }; break;
    case 0x1b: result = { true, 0x08, true, AutoDirection::COOLING }; break;
    default: break;
    }
    return result;
}

/// Decode the auto-direction reported in payload 5 of a subtype 0x09 response.
/// The official client tests two independent bit pairs, cooling first:
/// bits 1..0 == 0b11 or bits 5..4 == 0b10 → cooling (e.g. 0x28),
/// bits 1..0 == 0b10 or bits 5..4 == 0b01 → heating (e.g. 0x18).
inline AutoDirection decode_auto_direction(uint8_t status_payload_5) {
    const uint8_t low_pair = status_payload_5 & 0x03;
    const uint8_t high_pair = status_payload_5 & 0x30;
    if (low_pair == 0x03 || high_pair == 0x20) {
        return AutoDirection::COOLING;
    }
    if (low_pair == 0x02 || high_pair == 0x10) {
        return AutoDirection::HEATING;
    }
    return AutoDirection::NONE;
}

// ════════════════════════════════════════════════════════════════
// Model capability profile (0x7B PROFILECODE frames)
// ════════════════════════════════════════════════════════════════

/// Capability tables the indoor unit publishes during the extended handshake.
/// These are read-only: they describe what the model advertises and never
/// enable a write on their own.
struct ProfileCapabilities {
    bool valid = false;
    bool has_c9 = false;
    bool has_cd = false;
    bool has_d0 = false;
    uint8_t c9_6 = 0;
    uint8_t c9_9 = 0;
    uint8_t cd_7 = 0;
    uint8_t cd_8 = 0;
    uint8_t cd_13 = 0;
    uint8_t d0_1 = 0;
    uint8_t d0_2 = 0;
    char model[13] = {};   // D1 payload 1..12, e.g. "MSZZ***9025*"

    bool supports_thermal_image() const { return has_c9 && (c9_6 & 0x08) != 0; }
    bool supports_touch_flow() const { return has_c9 && (c9_6 & 0x40) != 0; }
    bool supports_auto_mode() const { return has_c9 && (c9_6 & 0x80) != 0; }
    bool supports_outside_temperature() const { return has_c9 && (c9_9 & 0x20) != 0; }

    bool humidity_shown_as_percent() const { return has_cd && (cd_7 & 0x01) != 0; }
    bool supports_energy_saving() const { return has_cd && (cd_7 & 0x02) != 0 && (cd_7 & 0x20) == 0; }
    bool supports_remote_handling() const { return has_cd && (cd_7 & 0x04) != 0; }
    bool uses_effective_room_temperature() const { return has_cd && (cd_7 & 0x10) != 0; }
    bool supports_ventilation_assist() const { return has_cd && (cd_8 & 0x40) != 0; }
    bool supports_online_serial_write() const { return has_cd && (cd_13 & 0x40) != 0; }

    bool supports_vital_sensor() const { return has_d0 && (d0_1 & 0x10) != 0 && (d0_1 & 0x60) == 0; }
    bool supports_stopped_sensing() const { return has_d0 && (d0_2 & 0x01) != 0; }
    bool displays_fan_speed() const { return has_d0 && (d0_2 & 0x04) != 0; }
    bool displays_vertical_vane() const { return has_d0 && (d0_2 & 0x08) != 0; }
    bool displays_horizontal_vane() const { return has_d0 && (d0_2 & 0x20) != 0; }
    bool supports_long_airflow() const { return has_d0 && (d0_2 & 0x40) != 0; }
    bool supports_special_airflow() const { return has_d0 && (d0_2 & 0x80) != 0; }
};

/// Decode one PROFILECODE payload into `caps`. `payload[0]` is the table id.
/// Returns true when the frame was a recognised table with enough bytes.
inline bool decode_profile_payload(const uint8_t* payload, int length, ProfileCapabilities& caps) {
    if (payload == nullptr || length <= 0) {
        return false;
    }
    switch (payload[0]) {
    case 0xc9:
        if (length <= 9) return false;
        caps.has_c9 = true;
        caps.c9_6 = payload[6];
        caps.c9_9 = payload[9];
        break;
    case 0xcd:
        if (length <= 13) return false;
        caps.has_cd = true;
        caps.cd_7 = payload[7];
        caps.cd_8 = payload[8];
        caps.cd_13 = payload[13];
        break;
    case 0xd0:
        if (length <= 2) return false;
        caps.has_d0 = true;
        caps.d0_1 = payload[1];
        caps.d0_2 = payload[2];
        break;
    case 0xd1: {
        if (length <= 12) return false;
        int written = 0;
        for (int i = 1; i <= 12; i++) {
            const uint8_t c = payload[i];
            if (c == 0x00) break;
            caps.model[written++] = static_cast<char>(c);
        }
        caps.model[written] = '\0';
        break;
    }
    default:
        return false;
    }
    caps.valid = true;
    return true;
}

/// Choose which of the two room-temperature bytes of a 0x03 response applies.
/// The official client uses the effective temperature (payload 7) only when the
/// CD profile advertises it; otherwise it uses the legacy value (payload 6).
inline uint8_t select_room_temperature_byte(uint8_t legacy, uint8_t effective, bool use_effective) {
    return (use_effective && effective != 0x00) ? effective : legacy;
}

// ════════════════════════════════════════════════════════════════
// SET subtype 0x08 — humidity, energy saving, special airflow, buzzer
// ════════════════════════════════════════════════════════════════

/// Persistent energy-saving flag written in payload 5.
inline uint8_t encode_energy_saving(bool enabled) { return enabled ? 0x0a : 0x00; }

/// Clamp a dehumidification target to the 40..70 % range the official UI offers,
/// snapped to the 10 % steps the remote uses.
inline uint8_t clamp_target_humidity(int percent) {
    if (percent < 40) percent = 40;
    if (percent > 70) percent = 70;
    return static_cast<uint8_t>(((percent + 5) / 10) * 10);
}

struct RunStateRequest {
    std::optional<uint8_t> target_humidity;  // payload 4, control bit 0x04
    std::optional<bool> energy_saving;       // payload 5, control bit 0x08
    std::optional<uint8_t> special_airflow;  // payload 6, control bit 0x20
    bool buzzer = false;                     // payload 7, control bit 0x10
};

/// Fill the official subtype 0x08 fields. Returns true when at least one field
/// was requested. The caller owns the header, any legacy fields and the FCC.
inline bool apply_run_state_request(uint8_t* packet, const RunStateRequest& request) {
    uint8_t control = 0;
    if (request.target_humidity) {
        control |= 0x04;
        packet[payload_offset(4)] = *request.target_humidity;
    }
    if (request.energy_saving) {
        control |= 0x08;
        packet[payload_offset(5)] = encode_energy_saving(*request.energy_saving);
    }
    if (request.buzzer) {
        control |= 0x10;
        packet[payload_offset(7)] = 0x01;
    }
    if (request.special_airflow) {
        control |= 0x20;
        packet[payload_offset(6)] = *request.special_airflow;
    }
    if (control == 0) {
        return false;
    }
    packet[payload_offset(0)] = SET_SUBTYPE_RUN_STATE;
    packet[payload_offset(1)] |= control;
    return true;
}

/// The thermal-image enable/disable command is a subtype 0x08 frame of its own:
/// control bit 0x80 in payload 1 and the state in payload 9.
inline void build_thermal_image_packet(uint8_t* packet, bool enabled) {
    packet[payload_offset(0)] = SET_SUBTYPE_RUN_STATE;
    packet[payload_offset(1)] = 0x80;
    packet[payload_offset(9)] = enabled ? 0x01 : 0x00;
}

// ════════════════════════════════════════════════════════════════
// SET subtype 0x33 — stopped-state sensing, left vane, Long airflow
// ════════════════════════════════════════════════════════════════

struct VaneExtensionRequest {
    std::optional<bool> stopped_sensing;  // payload 3, control bit 0x01
    std::optional<uint8_t> left_vane;     // payload 4, control bit 0x02
    std::optional<bool> long_airflow;     // payload 5, control bit 0x04
};

/// Build the payload of a subtype 0x33 SET. Returns false when nothing was
/// requested, so the caller can skip the write entirely.
inline bool apply_vane_extension_request(uint8_t* packet, const VaneExtensionRequest& request) {
    uint8_t control = 0;
    if (request.stopped_sensing) {
        control |= 0x01;
        packet[payload_offset(3)] = *request.stopped_sensing ? 0x01 : 0x00;
    }
    if (request.left_vane) {
        control |= 0x02;
        packet[payload_offset(4)] = *request.left_vane;
    }
    if (request.long_airflow) {
        control |= 0x04;
        packet[payload_offset(5)] = *request.long_airflow ? 0x01 : 0x00;
    }
    if (control == 0) {
        return false;
    }
    packet[payload_offset(0)] = SET_SUBTYPE_VANE_EXT;
    packet[payload_offset(1)] |= control;
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
