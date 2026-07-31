/// test_protocol.cpp — Tests for cn105_protocol.h pure protocol functions.
/// Deps: cn105_protocol.h (production code, no mocks needed)
///
/// These tests validate the ACTUAL production functions, not standalone copies.
/// Any regression in cn105_protocol.h will be caught here.
#include <gtest/gtest.h>
#include "cn105_protocol.h"

using namespace cn105_protocol;

// ════════════════════════════════════════════════════════════════
// checksum() — production function
// ════════════════════════════════════════════════════════════════

TEST(ProtocolChecksum, ConnectPacket) {
    uint8_t pkt[] = {0xfc, 0x5a, 0x01, 0x30, 0x02, 0xca, 0x01};
    EXPECT_EQ(checksum(pkt, 7), 0xa8);
}

TEST(ProtocolChecksum, InfoPacket) {
    uint8_t pkt[] = {0xfc, 0x42, 0x01, 0x30, 0x10,
                     0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(checksum(pkt, 21), 0x7b);
}

TEST(ProtocolChecksum, ZeroLength) {
    EXPECT_EQ(checksum(nullptr, 0), 0xfc);
}

TEST(ProtocolChecksum, Overflow) {
    // 0xfc + 0x04 = 0x100 → wraps to 0x00 in uint8_t, checksum = (0xfc - 0x00) & 0xff = 0xfc
    uint8_t pkt[] = {0xfc, 0x04};
    EXPECT_EQ(checksum(pkt, 2), 0xfc);
}

TEST(ProtocolChecksum, RealSettingsResponse) {
    // Real captured frame: FC 62 01 30 10 02 00 00 00 08 09 00 01 00 00 03 AC 00 00 00 00 (9A)
    uint8_t pkt[] = {0xFC, 0x62, 0x01, 0x30, 0x10,
                     0x02, 0x00, 0x00, 0x00, 0x08, 0x09, 0x00, 0x01,
                     0x00, 0x00, 0x03, 0xAC, 0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(checksum(pkt, 21), 0x9A);
}

// ════════════════════════════════════════════════════════════════
// decode_temperature() — production function
// ════════════════════════════════════════════════════════════════

TEST(ProtocolDecodeTemp, EncodingB_22C) {
    // enc_b = 0xAC = 172 → (172-128)/2 = 22.0
    EXPECT_FLOAT_EQ(decode_temperature(0x09, 0xAC), 22.0f);
}

TEST(ProtocolDecodeTemp, EncodingB_19_5C) {
    // enc_b = 0xA7 = 167 → (167-128)/2 = 19.5
    EXPECT_FLOAT_EQ(decode_temperature(0x1C, 0xA7), 19.5f);
}

TEST(ProtocolDecodeTemp, EncodingB_IgnoresEncA) {
    // When enc_b != 0, enc_a is ignored
    EXPECT_FLOAT_EQ(decode_temperature(0x00, 0xAC), 22.0f);
    EXPECT_FLOAT_EQ(decode_temperature(0xFF, 0xAC), 22.0f);
}

TEST(ProtocolDecodeTemp, EncodingA_FallbackWithOffset10) {
    // enc_b == 0 → uses enc_a + offset (default 10)
    EXPECT_FLOAT_EQ(decode_temperature(0x0B, 0x00), 21.0f);   // 11 + 10
    EXPECT_FLOAT_EQ(decode_temperature(0x00, 0x00), 10.0f);   // 0 + 10
}

TEST(ProtocolDecodeTemp, EncodingA_CustomOffset) {
    // Settings use different offset (31 - enc_a for encoding A)
    EXPECT_FLOAT_EQ(decode_temperature(0x09, 0x00, 22), 31.0f);  // 9 + 22
}

TEST(ProtocolDecodeTemp, RoomTemp_BothEncodingsAgree) {
    // Real frame: data[3]=0x0B (enc_a), data[6]=0xAA (enc_b)
    float tempA = decode_temperature(0x0B, 0x00);       // 11 + 10 = 21.0
    float tempB = decode_temperature(0x0B, 0xAA);       // (170-128)/2 = 21.0
    EXPECT_FLOAT_EQ(tempA, tempB);
    EXPECT_FLOAT_EQ(tempB, 21.0f);
}

TEST(ProtocolDecodeTemp, EncodingB_MinValue) {
    // enc_b = 128 → (128-128)/2 = 0.0
    EXPECT_FLOAT_EQ(decode_temperature(0x00, 0x80), 0.0f);
}

TEST(ProtocolDecodeTemp, EncodingB_HalfDegree) {
    // enc_b = 0xA9 = 169 → (169-128)/2 = 20.5
    EXPECT_FLOAT_EQ(decode_temperature(0x00, 0xA9), 20.5f);
}

// ════════════════════════════════════════════════════════════════
// encode_temperature_b() — production function
// ════════════════════════════════════════════════════════════════

TEST(ProtocolEncodeTemp, RoundTrip_19_5) {
    uint8_t encoded = encode_temperature_b(19.5f);
    EXPECT_EQ(encoded, 0xA7);
    EXPECT_FLOAT_EQ(decode_temperature(0x00, encoded), 19.5f);
}

TEST(ProtocolEncodeTemp, RoundTrip_22_0) {
    uint8_t encoded = encode_temperature_b(22.0f);
    EXPECT_EQ(encoded, 0xAC);
    EXPECT_FLOAT_EQ(decode_temperature(0x00, encoded), 22.0f);
}

TEST(ProtocolEncodeTemp, RoundTrip_16_0) {
    uint8_t encoded = encode_temperature_b(16.0f);
    EXPECT_EQ(encoded, 0xA0);
    EXPECT_FLOAT_EQ(decode_temperature(0x00, encoded), 16.0f);
}

TEST(ProtocolEncodeTemp, RoundTrip_31_0) {
    uint8_t encoded = encode_temperature_b(31.0f);
    EXPECT_EQ(encoded, 0xBE);
    EXPECT_FLOAT_EQ(decode_temperature(0x00, encoded), 31.0f);
}

TEST(ProtocolEncodeTemp, HalfDegreeValues) {
    for (float t = 16.0f; t <= 31.0f; t += 0.5f) {
        uint8_t encoded = encode_temperature_b(t);
        float decoded = decode_temperature(0x00, encoded);
        EXPECT_FLOAT_EQ(decoded, t) << "Roundtrip failed for " << t << "°C";
    }
}

// ════════════════════════════════════════════════════════════════
// encode_remote_temperature() — production function
// ════════════════════════════════════════════════════════════════

TEST(ProtocolEncodeRemoteTemp, KeepAlive_20_9C) {
    uint8_t enc_a, enc_b;
    encode_remote_temperature(20.9f, enc_a, enc_b);
    // round(20.9*2) = round(41.8) = 42
    // enc_a = 42 - 16 = 26 = 0x1A
    // enc_b = 42 + 128 = 170 = 0xAA
    EXPECT_EQ(enc_a, 0x1A);
    EXPECT_EQ(enc_b, 0xAA);
}

TEST(ProtocolEncodeRemoteTemp, Exact_21_0C) {
    uint8_t enc_a, enc_b;
    encode_remote_temperature(21.0f, enc_a, enc_b);
    EXPECT_EQ(enc_a, 0x1A);  // 42 - 16
    EXPECT_EQ(enc_b, 0xAA);  // 42 + 128
}

TEST(ProtocolEncodeRemoteTemp, Low_10_0C) {
    uint8_t enc_a, enc_b;
    encode_remote_temperature(10.0f, enc_a, enc_b);
    // round(10.0*2) = 20
    EXPECT_EQ(enc_a, 0x04);  // 20 - 16
    EXPECT_EQ(enc_b, 0x94);  // 20 + 128
}

// ════════════════════════════════════════════════════════════════
// lookup_value() — production function
// ════════════════════════════════════════════════════════════════

// Import the protocol tables from cn105_types.h for testing
#include "cn105_types.h"

TEST(ProtocolLookup, PowerOff) {
    EXPECT_STREQ(lookup_value(POWER_MAP, POWER, 2, 0x00), "OFF");
}

TEST(ProtocolLookup, PowerOn) {
    EXPECT_STREQ(lookup_value(POWER_MAP, POWER, 2, 0x01), "ON");
}

TEST(ProtocolLookup, ModeHeat) {
    EXPECT_STREQ(lookup_value(MODE_MAP, MODE, 5, 0x01), "HEAT");
}

TEST(ProtocolLookup, ModeCool) {
    EXPECT_STREQ(lookup_value(MODE_MAP, MODE, 5, 0x03), "COOL");
}

TEST(ProtocolLookup, ModeAuto) {
    EXPECT_STREQ(lookup_value(MODE_MAP, MODE, 5, 0x08), "AUTO");
}

TEST(ProtocolLookup, FanAuto) {
    EXPECT_STREQ(lookup_value(FAN_MAP, FAN, 6, 0x00), "AUTO");
}

TEST(ProtocolLookup, FanQuiet) {
    EXPECT_STREQ(lookup_value(FAN_MAP, FAN, 6, 0x01), "QUIET");
}

TEST(ProtocolLookup, UnknownByteFallsBackToIndex0) {
    EXPECT_STREQ(lookup_value(MODE_MAP, MODE, 5, 0xFF), "HEAT");
}

TEST(ProtocolLookup, TempMapIndex0) {
    EXPECT_EQ(lookup_value(TEMP_MAP, TEMP, 16, 0x00), 31);
}

TEST(ProtocolLookup, TempMapIndex15) {
    EXPECT_EQ(lookup_value(TEMP_MAP, TEMP, 16, 0x0F), 16);
}

// ════════════════════════════════════════════════════════════════
// lookup_index() — production function
// ════════════════════════════════════════════════════════════════

TEST(ProtocolLookupIndex, FindsExistingInt) {
    EXPECT_EQ(lookup_index(TEMP_MAP, 16, 22), 9);  // 22°C is at index 9
}

TEST(ProtocolLookupIndex, ReturnsMinusOneForMissing) {
    EXPECT_EQ(lookup_index(TEMP_MAP, 16, 99), -1);
}

TEST(ProtocolLookupIndex, FindsExistingString) {
    EXPECT_EQ(lookup_index(MODE_MAP, 5, "COOL"), 2);
}

TEST(ProtocolLookupIndex, StringCaseInsensitive) {
    EXPECT_EQ(lookup_index(MODE_MAP, 5, "cool"), 2);
}

TEST(ProtocolLookupIndex, StringNotFound) {
    EXPECT_EQ(lookup_index(MODE_MAP, 5, "TURBO"), -1);
}

// ════════════════════════════════════════════════════════════════
// lookup_value_opt() — std::optional variant (graceful degradation)
// ════════════════════════════════════════════════════════════════

TEST(ProtocolLookupOpt, PowerOnReturnsValue) {
    auto result = lookup_value_opt(POWER_MAP, POWER, 2, 0x01);
    ASSERT_TRUE(result.has_value());
    EXPECT_STREQ(*result, "ON");
}

TEST(ProtocolLookupOpt, UnknownPowerReturnsNullopt) {
    auto result = lookup_value_opt(POWER_MAP, POWER, 2, 0xFF);
    EXPECT_FALSE(result.has_value());
}

TEST(ProtocolLookupOpt, ModeCoolReturnsValue) {
    auto result = lookup_value_opt(MODE_MAP, MODE, 5, 0x03);
    ASSERT_TRUE(result.has_value());
    EXPECT_STREQ(*result, "COOL");
}

TEST(ProtocolLookupOpt, UnknownModeReturnsNullopt) {
    // 0x0A could be a future "ECO" mode — should return nullopt, NOT "HEAT"
    auto result = lookup_value_opt(MODE_MAP, MODE, 5, 0x0A);
    EXPECT_FALSE(result.has_value());
}

TEST(ProtocolLookupOpt, FanQuietReturnsValue) {
    auto result = lookup_value_opt(FAN_MAP, FAN, 6, 0x01);
    ASSERT_TRUE(result.has_value());
    EXPECT_STREQ(*result, "QUIET");
}

TEST(ProtocolLookupOpt, UnknownFanReturnsNullopt) {
    auto result = lookup_value_opt(FAN_MAP, FAN, 6, 0x09);
    EXPECT_FALSE(result.has_value());
}

TEST(ProtocolLookupOpt, VaneSwingReturnsValue) {
    auto result = lookup_value_opt(VANE_MAP, VANE, 7, 0x07);
    ASSERT_TRUE(result.has_value());
    EXPECT_STREQ(*result, "SWING");
}

TEST(ProtocolLookupOpt, UnknownVaneReturnsNullopt) {
    auto result = lookup_value_opt(VANE_MAP, VANE, 7, 0x06);
    EXPECT_FALSE(result.has_value());
}

TEST(ProtocolLookupOpt, TempMapIntReturnsValue) {
    auto result = lookup_value_opt(TEMP_MAP, TEMP, 16, 0x09);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 22);
}

TEST(ProtocolLookupOpt, TempMapIntUnknownReturnsNullopt) {
    auto result = lookup_value_opt(TEMP_MAP, TEMP, 16, 0x10);
    EXPECT_FALSE(result.has_value());
}

// ════════════════════════════════════════════════════════════════
// lookup_index_opt() — std::optional variant
// ════════════════════════════════════════════════════════════════

TEST(ProtocolLookupIndexOpt, IntFound) {
    auto result = lookup_index_opt(TEMP_MAP, 16, 22);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 9);
}

TEST(ProtocolLookupIndexOpt, IntNotFound) {
    auto result = lookup_index_opt(TEMP_MAP, 16, 99);
    EXPECT_FALSE(result.has_value());
}

TEST(ProtocolLookupIndexOpt, StringFound) {
    auto result = lookup_index_opt(MODE_MAP, 5, "AUTO");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 4);
}

TEST(ProtocolLookupIndexOpt, StringCaseInsensitive) {
    auto result = lookup_index_opt(MODE_MAP, 5, "auto");
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, 4);
}

TEST(ProtocolLookupIndexOpt, StringNotFound) {
    auto result = lookup_index_opt(MODE_MAP, 5, "TURBO");
    EXPECT_FALSE(result.has_value());
}

TEST(ProtocolErrorCode, MasksStatusFlag) {
    EXPECT_EQ(decode_error_code(0x80), 0x00);
    EXPECT_EQ(decode_error_code(0x85), 0x05);
    EXPECT_EQ(decode_error_code(0x05), 0x05);
}

TEST(ProtocolJpAutoSubMode, DecodesCapturedStates) {
    auto non_auto = lookup_value_opt(AUTO_SUB_MODE_MAP, AUTO_SUB_MODE, AUTO_SUB_MODE_LEN, 0x08);
    auto heating = lookup_value_opt(AUTO_SUB_MODE_MAP, AUTO_SUB_MODE, AUTO_SUB_MODE_LEN, 0x18);
    auto cooling = lookup_value_opt(AUTO_SUB_MODE_MAP, AUTO_SUB_MODE, AUTO_SUB_MODE_LEN, 0x28);
    ASSERT_TRUE(non_auto.has_value());
    ASSERT_TRUE(heating.has_value());
    ASSERT_TRUE(cooling.has_value());
    EXPECT_STREQ(*non_auto, "JP_NON_AUTO");
    EXPECT_STREQ(*heating, "JP_AUTO_HEATING");
    EXPECT_STREQ(*cooling, "JP_AUTO_COOLING");
}

TEST(ProtocolWideVane, AirflowControlForcesAdjustmentBit) {
    EXPECT_EQ(encode_wide_vane(0x00, false, true), 0x80);
    EXPECT_EQ(encode_wide_vane(0x07, true, false), 0x87);
    EXPECT_EQ(encode_wide_vane(0x07, false, false), 0x07);
}

TEST(ProtocolVerticalVanes, WritesRightVaneAndControlFlag) {
    uint8_t packet[22] = {};
    packet[6] = 0x02;

    EXPECT_TRUE(apply_vertical_vane_control(packet, 0x04));
    EXPECT_EQ(packet[6], 0x12);
    EXPECT_EQ(packet[12], 0x04);
    // Payload 15 is the origin marker: a vane write must never land there.
    EXPECT_EQ(packet[20], 0x00);
}

TEST(ProtocolVerticalVanes, LeavesPacketUntouchedWithoutValues) {
    uint8_t packet[22] = {};
    packet[6] = 0x02;

    EXPECT_FALSE(apply_vertical_vane_control(packet, std::nullopt));
    EXPECT_EQ(packet[6], 0x02);
}

// ════════════════════════════════════════════════════════════════
// Command origin marker (subtype 0x01 payload 15)
// ════════════════════════════════════════════════════════════════

TEST(ProtocolCommandOrigin, DefaultsToLocalControl) {
    uint8_t packet[22] = {};
    set_command_origin(packet);
    EXPECT_EQ(packet[20], 0x41);
}

TEST(ProtocolCommandOrigin, AcceptsCloudMarker) {
    uint8_t packet[22] = {};
    set_command_origin(packet, SET_ORIGIN_OUTSIDE);
    EXPECT_EQ(packet[20], 0x42);
}

// ════════════════════════════════════════════════════════════════
// Operating mode decode (official getDriveMode table)
// ════════════════════════════════════════════════════════════════

TEST(ProtocolModeDecode, PlainModes) {
    EXPECT_EQ(decode_mode_byte(0x01).mode, 0x01);
    EXPECT_FALSE(decode_mode_byte(0x01).iSee);
    EXPECT_EQ(decode_mode_byte(0x03).mode, 0x03);
    EXPECT_EQ(decode_mode_byte(0x07).mode, 0x07);
    EXPECT_EQ(decode_mode_byte(0x08).mode, 0x08);
}

TEST(ProtocolModeDecode, ISeeVariants) {
    EXPECT_EQ(decode_mode_byte(0x09).mode, 0x01);
    EXPECT_TRUE(decode_mode_byte(0x09).iSee);
    EXPECT_EQ(decode_mode_byte(0x0b).mode, 0x03);
    EXPECT_TRUE(decode_mode_byte(0x0b).iSee);
    // 0x0C was previously decoded as mode 0x04 and rejected as unknown.
    EXPECT_EQ(decode_mode_byte(0x0c).mode, 0x02);
    EXPECT_TRUE(decode_mode_byte(0x0c).iSee);
}

TEST(ProtocolModeDecode, JpAutoModesReportDirection) {
    const auto heating = decode_mode_byte(0x19);
    EXPECT_TRUE(heating.valid);
    EXPECT_EQ(heating.mode, 0x08);
    EXPECT_EQ(heating.auto_direction, AutoDirection::HEATING);

    const auto cooling = decode_mode_byte(0x1b);
    EXPECT_TRUE(cooling.valid);
    EXPECT_EQ(cooling.mode, 0x08);
    EXPECT_EQ(cooling.auto_direction, AutoDirection::COOLING);
}

TEST(ProtocolModeDecode, UnknownByteIsRejected) {
    EXPECT_FALSE(decode_mode_byte(0x00).valid);
    EXPECT_FALSE(decode_mode_byte(0x55).valid);
}

TEST(ProtocolAutoDirection, DecodesCapturedValues) {
    // Hardware capture from the ZW9025: 0x28 while auto-cooling.
    EXPECT_EQ(decode_auto_direction(0x28), AutoDirection::COOLING);
    EXPECT_EQ(decode_auto_direction(0x18), AutoDirection::HEATING);
    EXPECT_EQ(decode_auto_direction(0x00), AutoDirection::NONE);
    // The official decoder also accepts the low bit pair.
    EXPECT_EQ(decode_auto_direction(0x03), AutoDirection::COOLING);
    EXPECT_EQ(decode_auto_direction(0x02), AutoDirection::HEATING);
}

// ════════════════════════════════════════════════════════════════
// PROFILECODE capability frames (hardware capture, ZW9025 family)
// ════════════════════════════════════════════════════════════════

namespace {
// fc7b013010 c9 0300200014dfe58435a0be94bea0be b9  → payload starts at the table id
const uint8_t kProfileC9[16] = { 0xc9, 0x03, 0x00, 0x20, 0x00, 0x14, 0xdf, 0xe5,
                                 0x84, 0x35, 0xa0, 0xbe, 0x94, 0xbe, 0xa0, 0xbe };
const uint8_t kProfileCD[16] = { 0xcd, 0xa0, 0xbe, 0xa0, 0xbe, 0xa0, 0xbe, 0x1f,
                                 0xd1, 0x03, 0x96, 0x14, 0xc0, 0x40, 0x03, 0x00 };
const uint8_t kProfileD0[16] = { 0xd0, 0x1c, 0xef, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
const uint8_t kProfileD1[16] = { 0xd1, 0x4d, 0x53, 0x5a, 0x5a, 0x2a, 0x2a, 0x2a,
                                 0x39, 0x30, 0x32, 0x35, 0x2a, 0x00, 0x00, 0x00 };
}  // namespace

TEST(ProtocolProfile, DecodesCapturedZw9025Tables) {
    ProfileCapabilities caps{};

    ASSERT_TRUE(decode_profile_payload(kProfileC9, 16, caps));
    EXPECT_EQ(caps.c9_6, 0xdf);
    EXPECT_EQ(caps.c9_9, 0x35);
    EXPECT_TRUE(caps.supports_thermal_image());
    EXPECT_TRUE(caps.supports_touch_flow());
    EXPECT_TRUE(caps.supports_auto_mode());
    EXPECT_TRUE(caps.supports_outside_temperature());

    ASSERT_TRUE(decode_profile_payload(kProfileCD, 16, caps));
    EXPECT_EQ(caps.cd_7, 0x1f);
    EXPECT_TRUE(caps.humidity_shown_as_percent());
    EXPECT_TRUE(caps.supports_energy_saving());
    EXPECT_TRUE(caps.uses_effective_room_temperature());
    EXPECT_TRUE(caps.supports_ventilation_assist());
    EXPECT_TRUE(caps.supports_online_serial_write());

    ASSERT_TRUE(decode_profile_payload(kProfileD0, 16, caps));
    EXPECT_TRUE(caps.supports_vital_sensor());
    EXPECT_TRUE(caps.supports_stopped_sensing());
    EXPECT_TRUE(caps.displays_vertical_vane());
    EXPECT_TRUE(caps.displays_horizontal_vane());
    EXPECT_TRUE(caps.supports_long_airflow());
    EXPECT_TRUE(caps.supports_special_airflow());

    ASSERT_TRUE(decode_profile_payload(kProfileD1, 16, caps));
    EXPECT_STREQ(caps.model, "MSZZ***9025*");
}

TEST(ProtocolProfile, IgnoresUnknownAndShortFrames) {
    ProfileCapabilities caps{};
    const uint8_t unknown[4] = { 0xce, 0x00, 0x00, 0x00 };
    EXPECT_FALSE(decode_profile_payload(unknown, 4, caps));
    EXPECT_FALSE(decode_profile_payload(kProfileC9, 4, caps));
    EXPECT_FALSE(decode_profile_payload(nullptr, 16, caps));
    EXPECT_FALSE(caps.valid);
}

TEST(ProtocolProfile, CapabilitiesAreFalseWithoutTheirTable) {
    ProfileCapabilities caps{};
    ASSERT_TRUE(decode_profile_payload(kProfileC9, 16, caps));
    // Only C9 was seen, so CD/D0 capabilities must not be claimed.
    EXPECT_FALSE(caps.uses_effective_room_temperature());
    EXPECT_FALSE(caps.supports_long_airflow());
}

TEST(ProtocolRoomTemperature, UsesEffectiveByteOnlyWhenAdvertised) {
    // Captured pair: legacy 0xB4 = 26.0 C, effective 0xB5 = 26.5 C.
    EXPECT_EQ(select_room_temperature_byte(0xb4, 0xb5, true), 0xb5);
    EXPECT_EQ(select_room_temperature_byte(0xb4, 0xb5, false), 0xb4);
    // An empty effective byte must never win.
    EXPECT_EQ(select_room_temperature_byte(0xb4, 0x00, true), 0xb4);
}

// ════════════════════════════════════════════════════════════════
// SET subtype 0x08 — run states
// ════════════════════════════════════════════════════════════════

TEST(ProtocolRunState, EncodesEnergySavingAndAirflow) {
    uint8_t packet[22] = {};
    RunStateRequest request{};
    request.energy_saving = true;
    request.special_airflow = 0x03;

    ASSERT_TRUE(apply_run_state_request(packet, request));
    EXPECT_EQ(packet[5], 0x08);
    EXPECT_EQ(packet[6], 0x28);   // 0x08 energy saving | 0x20 special airflow
    EXPECT_EQ(packet[10], 0x0a);  // payload 5
    EXPECT_EQ(packet[11], 0x03);  // payload 6
}

TEST(ProtocolRunState, MatchesOfficialBuzzerFrame) {
    // Exact frame found in the official Kiriweb encoder:
    // FC 41 01 30 10 08 10 00 00 00 00 00 01 00 00 00 00 00 00 00 00 65
    uint8_t packet[22] = { 0xfc, 0x41, 0x01, 0x30, 0x10 };
    RunStateRequest request{};
    request.buzzer = true;

    ASSERT_TRUE(apply_run_state_request(packet, request));
    packet[21] = checksum(packet, 21);

    const uint8_t expected[22] = { 0xfc, 0x41, 0x01, 0x30, 0x10, 0x08, 0x10, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                                   0x00, 0x00, 0x00, 0x00, 0x00, 0x65 };
    EXPECT_EQ(memcmp(packet, expected, sizeof(expected)), 0);
}

TEST(ProtocolRunState, EmptyRequestWritesNothing) {
    uint8_t packet[22] = {};
    RunStateRequest request{};
    EXPECT_FALSE(apply_run_state_request(packet, request));
    EXPECT_EQ(packet[5], 0x00);
    EXPECT_EQ(packet[6], 0x00);
}

TEST(ProtocolRunState, ClampsHumidityToRemoteRange) {
    // The official client floors to the 10 % step, then clamps to 40..70.
    EXPECT_EQ(clamp_target_humidity(10), 40);
    EXPECT_EQ(clamp_target_humidity(45), 40);
    EXPECT_EQ(clamp_target_humidity(49), 40);
    EXPECT_EQ(clamp_target_humidity(50), 50);
    EXPECT_EQ(clamp_target_humidity(60), 60);
    EXPECT_EQ(clamp_target_humidity(70), 70);
    EXPECT_EQ(clamp_target_humidity(99), 70);
    EXPECT_EQ(clamp_target_humidity(-5), 40);
}

TEST(ProtocolThermalImage, UsesItsOwnControlBit) {
    uint8_t packet[22] = {};
    build_thermal_image_packet(packet, true);
    EXPECT_EQ(packet[5], 0x08);
    EXPECT_EQ(packet[6], 0x80);
    EXPECT_EQ(packet[14], 0x01);  // payload 9

    build_thermal_image_packet(packet, false);
    EXPECT_EQ(packet[14], 0x00);
}

// ════════════════════════════════════════════════════════════════
// SET subtype 0x33 — left vane, Long airflow, stopped-state sensing
// ════════════════════════════════════════════════════════════════

TEST(ProtocolVaneExtension, WritesLeftVaneWithItsOwnMask) {
    uint8_t packet[22] = {};
    VaneExtensionRequest request{};
    request.left_vane = 0x05;

    ASSERT_TRUE(apply_vane_extension_request(packet, request));
    EXPECT_EQ(packet[5], 0x33);
    EXPECT_EQ(packet[6], 0x02);
    EXPECT_EQ(packet[9], 0x05);   // payload 4
}

TEST(ProtocolVaneExtension, CombinesSensingAndLongAirflow) {
    uint8_t packet[22] = {};
    VaneExtensionRequest request{};
    request.stopped_sensing = true;
    request.long_airflow = true;

    ASSERT_TRUE(apply_vane_extension_request(packet, request));
    EXPECT_EQ(packet[6], 0x05);   // 0x01 sensing | 0x04 Long
    EXPECT_EQ(packet[8], 0x01);   // payload 3
    EXPECT_EQ(packet[10], 0x01);  // payload 5
}

TEST(ProtocolVaneExtension, EmptyRequestWritesNothing) {
    uint8_t packet[22] = {};
    VaneExtensionRequest request{};
    EXPECT_FALSE(apply_vane_extension_request(packet, request));
    EXPECT_EQ(packet[5], 0x00);
}

TEST(ProtocolLookupIndex, NullStringReturnsMinusOne) {
    EXPECT_EQ(lookup_index(MODE_MAP, 5, nullptr), -1);
}

TEST(ProtocolLookupIndexOpt, NullStringReturnsNullopt) {
    auto result = lookup_index_opt(MODE_MAP, 5, nullptr);
    EXPECT_FALSE(result.has_value());
}

// ════════════════════════════════════════════════════════════════
// Subtype 0x09 status flags
// ════════════════════════════════════════════════════════════════

TEST(ProtocolStatusFlags, SpecialStoppingIsBit0x04) {
    EXPECT_TRUE(decode_special_stopping(0x04));
    EXPECT_TRUE(decode_special_stopping(0x2C));   // set alongside auto-cooling bits
    EXPECT_FALSE(decode_special_stopping(0x00));
    // Captured on a live MSZ-R2225: auto-direction values must not read as stopping.
    EXPECT_FALSE(decode_special_stopping(0x18));
    EXPECT_FALSE(decode_special_stopping(0x28));
}

TEST(ProtocolStatusFlags, MultiStandbyIsBit0x08) {
    EXPECT_TRUE(decode_multi_standby(0x08));
    EXPECT_FALSE(decode_multi_standby(0x00));
    // Sub-mode values that share payload 3 must not read as multi-standby.
    EXPECT_FALSE(decode_multi_standby(0x01));
    EXPECT_FALSE(decode_multi_standby(0x02));
    EXPECT_FALSE(decode_multi_standby(0x04));
}

TEST(ProtocolStatusFlags, StoppingAndDirectionAreIndependentBits) {
    // Payload 5 carries both; decoding one must not disturb the other.
    EXPECT_EQ(decode_auto_direction(0x2C), AutoDirection::COOLING);
    EXPECT_TRUE(decode_special_stopping(0x2C));
    EXPECT_EQ(decode_auto_direction(0x1C), AutoDirection::HEATING);
    EXPECT_TRUE(decode_special_stopping(0x1C));
}
