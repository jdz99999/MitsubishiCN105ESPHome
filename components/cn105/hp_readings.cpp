#include "cn105.h"

#include <map>

using namespace esphome;

/**
 * processInput: reads available bytes from UART and feeds them to the FrameParser.
 * When a complete frame is detected, delegates to processDataPacket().
 */


bool CN105Climate::processInput(void) {
    bool processed = false;
    while (this->get_hw_serial_()->available()) {
        processed = true;
        uint8_t inputData;
        if (this->get_hw_serial_()->read_byte(&inputData)) {
            ESP_LOGV("Decoder", "--> %02X", inputData);
            this->parser_.feed(inputData);
            if (this->parser_.frame_complete()) {
                this->processDataPacket();
                this->parser_.reset();
            }
        }
    }
    return processed;
}

/**
 * processDataPacket: called when the FrameParser has assembled a complete frame.
 * Validates checksum, sets the data pointer, and dispatches to processCommand().
 */
void CN105Climate::processDataPacket() {

    ESP_LOGV(TAG, "processing data packet...");

    // Point data at the payload section of the parser buffer
    // Note: cast away const because downstream code uses non-const data pointer
    this->data = const_cast<uint8_t*>(this->parser_.data());

    this->hpPacketDebug(this->parser_.raw(), this->parser_.frame_size(), "READ");

    // During handshake, log every received frame for diagnostics
    if (!this->isHeatpumpConnected()) {
        ESP_LOGD(LOG_CONN_TAG, "RX during handshake (cmd=0x%02X len=%d)",
            this->parser_.command(), this->parser_.data_length());
        this->hpPacketDebug(this->parser_.raw(), this->parser_.frame_size(), LOG_CONN_TAG);
    }

    if (this->parser_.checksum_valid()) {
        ESP_LOGD("chkSum", "OK");
        // checkpoint of a heatpump response
        this->lastResponseMs = CUSTOM_MILLIS;

        // processing the specific command
        processCommand();
    } else {
        ESP_LOGW("chkSum", "KO -> checksum mismatch (cmd=0x%02X len=%d)",
            this->parser_.command(), this->parser_.data_length());
        if (!this->isHeatpumpConnected()) {
            ESP_LOGD(LOG_CONN_TAG, "Checksum KO during handshake");
            this->hpPacketDebug(this->parser_.raw(), this->parser_.frame_size(), LOG_CONN_TAG);
        }
    }
}



void CN105Climate::getAutoModeStateFromResponsePacket() {
    heatpumpSettings receivedSettings{};

    if (data[10] == 0x00) {
        ESP_LOGD("Decoder", "[0x10 is 0x00]");

    } else if (data[10] == 0x01) {
        ESP_LOGD("Decoder", "[0x10 is 0x01]");

    } else if (data[10] == 0x02) {
        ESP_LOGD("Decoder", "[0x10 is 0x02]");

    } else {
        ESP_LOGD("Decoder", "[0x10 is unknown]");

    }
}

void CN105Climate::getPowerFromResponsePacket() {
    ESP_LOGD("Decoder", "[0x09 is sub modes]");

    heatpumpSettings receivedSettings{};
    if (this->isRawProbeCode(0x09)) {
        ESP_LOGD(LOG_RAW_PROBE_TAG, "0x09 candidates: data[3]=0x%02X data[4]=0x%02X data[5]=0x%02X data[6]=0x%02X data[7]=0x%02X data[8]=0x%02X data[9]=0x%02X data[10]=0x%02X data[11]=0x%02X data[12]=0x%02X data[13]=0x%02X data[14]=0x%02X data[15]=0x%02X",
            data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
    }

    // Use std::optional lookups — keep previous value on unknown bytes
    auto stage_opt = cn105_protocol::lookup_value_opt(STAGE_MAP, STAGE, 7, data[4]);
    if (stage_opt) {
        receivedSettings.stage = *stage_opt;
    } else {
        ESP_LOGW("Decoder", "Unknown stage byte 0x%02X — keeping previous value", data[4]);
        receivedSettings.stage = this->currentSettings.stage
            ? this->currentSettings.stage
            : STAGE_MAP[0];  // default to "IDLE" when no prior value exists
    }

    auto sub_mode_opt = cn105_protocol::lookup_value_opt(SUB_MODE_MAP, SUB_MODE, 6, data[3]);
    if (sub_mode_opt) {
        receivedSettings.sub_mode = *sub_mode_opt;
    } else {
        ESP_LOGW("Decoder", "Unknown sub_mode byte 0x%02X — keeping previous value", data[3]);
        receivedSettings.sub_mode = this->currentSettings.sub_mode
            ? this->currentSettings.sub_mode
            : SUB_MODE_MAP[0];  // default to "NORMAL" when no prior value exists
    }

    // Payload 5 is a bitfield: the special-stopping flag rides alongside the sub-mode
    // value, so strip it before the lookup. A JP unit running a cleaning cycle while
    // stopped otherwise reports an unknown byte on every poll.
    const uint8_t auto_sub_mode_byte = cn105_protocol::strip_status_flags(data[5]);
    auto auto_sub_mode_opt = cn105_protocol::lookup_value_opt(AUTO_SUB_MODE_MAP, AUTO_SUB_MODE, AUTO_SUB_MODE_LEN, auto_sub_mode_byte);
    if (auto_sub_mode_opt) {
        receivedSettings.auto_sub_mode = *auto_sub_mode_opt;
    } else {
        ESP_LOGW("Decoder", "Unknown auto_sub_mode byte 0x%02X (raw 0x%02X) — keeping previous value", auto_sub_mode_byte, data[5]);
        receivedSettings.auto_sub_mode = this->currentSettings.auto_sub_mode
            ? this->currentSettings.auto_sub_mode
            : AUTO_SUB_MODE_MAP[0];  // default to "AUTO_OFF" when no prior value exists
    }

    ESP_LOGD("Decoder", "[Stage : %s]", receivedSettings.stage);
    ESP_LOGD("Decoder", "[Sub Mode  : %s]", receivedSettings.sub_mode);
    ESP_LOGD("Decoder", "[Auto Mode Sub Mode  : %s]", receivedSettings.auto_sub_mode);

    // Direction the unit picked while running an automatic mode. The official
    // decoder reads it from two bit pairs of this same byte, which covers the
    // values the AUTO_SUB_MODE table does not enumerate (0x18, 0x28, ...).
    const char* auto_direction = cn105_protocol::auto_direction_to_string(
        cn105_protocol::decode_auto_direction(data[5]));
    if (this->auto_direction_sensor_ != nullptr &&
        (this->auto_direction_state_ == nullptr || strcmp(auto_direction, this->auto_direction_state_) != 0)) {
        ESP_LOGD("Decoder", "[Auto direction : %s] raw=0x%02X", auto_direction, data[5]);
        this->auto_direction_state_ = auto_direction;
        this->auto_direction_sensor_->publish_state(auto_direction);
    }

    // Payload 7 carries the thermal-image (Move Eye panorama) enable state. The
    // image itself is cloud-side; only the on/off state travels over CN105.
    if (this->wantedRunStates.thermal_image < 0) {
        this->publishRunStateSwitch(this->thermal_image_switch_, this->currentRunStates.thermal_image,
            data[7] == 0x01 ? 1 : 0, "thermal image");
    }

    // Payload 5 bit 0x04 is the official client's "special stopping mode": the
    // unit is doing real work after being switched off. On JP models the manuals
    // describe three such behaviours — the internal-clean / mould-guard cycle,
    // the filter self-cleaning mechanism, and the periodic fan of the high/low
    // temperature watch, which can also restart cooling or heating by itself.
    if (this->unit_activity_sensor_ != nullptr) {
        const bool special_stopping = cn105_protocol::decode_special_stopping(data[5]);
        const bool powered = this->currentSettings.power != nullptr &&
            strcmp(this->currentSettings.power, "ON") == 0;
        const char* activity = powered ? UNIT_ACTIVITY_RUNNING
            : (special_stopping ? UNIT_ACTIVITY_CLEANING : UNIT_ACTIVITY_IDLE);
        if (this->unit_activity_state_ == nullptr || strcmp(activity, this->unit_activity_state_) != 0) {
            ESP_LOGD("Decoder", "[Unit activity : %s] raw=0x%02X", activity, data[5]);
            this->unit_activity_state_ = activity;
            this->unit_activity_sensor_->publish_state(activity);
        }
    }

    if (this->multi_standby_sensor_ != nullptr) {
        const bool multi_standby = cn105_protocol::decode_multi_standby(data[3]);
        if (!this->multi_standby_sensor_->has_state() ||
            this->multi_standby_sensor_->state != multi_standby) {
            ESP_LOGD("Decoder", "[Multi standby : %s] raw=0x%02X", multi_standby ? "YES" : "NO", data[3]);
            this->multi_standby_sensor_->publish_state(multi_standby);
        }
    }

    //this->heatpumpUpdate(receivedSettings);
    if (this->stage_sensor_ != nullptr) {
        if (!this->currentSettings.stage || strcmp(receivedSettings.stage, this->currentSettings.stage) != 0) {
            this->currentSettings.stage = receivedSettings.stage;
            this->stage_sensor_->publish_state(receivedSettings.stage);

            // If using stage as operating fallback, update action immediately when stage changes
            // and publish to Home Assistant
            if (this->use_stage_for_operating_status_) {
                this->updateAction();
                this->publish_state();
            }
        }
    }
    if (this->Sub_mode_sensor_ != nullptr && (!this->currentSettings.sub_mode || strcmp(receivedSettings.sub_mode, this->currentSettings.sub_mode) != 0)) {
        this->currentSettings.sub_mode = receivedSettings.sub_mode;
        this->Sub_mode_sensor_->publish_state(receivedSettings.sub_mode);
    }
    if (this->Auto_sub_mode_sensor_ != nullptr && (!this->currentSettings.auto_sub_mode || strcmp(receivedSettings.auto_sub_mode, this->currentSettings.auto_sub_mode) != 0)) {
        this->currentSettings.auto_sub_mode = receivedSettings.auto_sub_mode;
        this->Auto_sub_mode_sensor_->publish_state(receivedSettings.auto_sub_mode);
    }
}

void CN105Climate::getSettingsFromResponsePacket() {
    heatpumpSettings receivedSettings{};
    heatpumpRunStates receivedRunStates{};
    ESP_LOGD("Decoder", "[0x02 is settings]");

    receivedSettings.connected = true;

    auto power_opt = cn105_protocol::lookup_value_opt(POWER_MAP, POWER, 2, data[3]);
    if (power_opt) {
        receivedSettings.power = *power_opt;
    } else {
        ESP_LOGW("Decoder", "Unknown power byte 0x%02X — keeping previous value", data[3]);
        receivedSettings.power = this->currentSettings.power
            ? this->currentSettings.power
            : POWER_MAP[0];  // default to "OFF" when no prior value exists
    }

    // Operating mode, decoded with the official client's table. Raw 0x19/0x1B are
    // the JP automatic modes that also announce the direction the unit picked;
    // both normalise to AUTO so Home Assistant still sees one AUTO mode.
    const auto modeDecode = cn105_protocol::decode_mode_byte(data[4]);
    if (modeDecode.valid) {
        receivedSettings.iSee = modeDecode.iSee;
        auto mode_opt = cn105_protocol::lookup_value_opt(MODE_MAP, MODE, 5, modeDecode.mode);
        receivedSettings.mode = mode_opt ? *mode_opt : MODE_MAP[4];
    } else {
        ESP_LOGW("Decoder", "Unknown mode byte 0x%02X — keeping previous value", data[4]);
        receivedSettings.iSee = this->currentSettings.iSee;
        receivedSettings.mode = this->currentSettings.mode
            ? this->currentSettings.mode
            : MODE_MAP[4];  // default to "AUTO" when no prior value exists
    }

    ESP_LOGD("Decoder", "[Power : %s]", receivedSettings.power);
    ESP_LOGD("Decoder", "[iSee  : %d]", receivedSettings.iSee);
    ESP_LOGD("Decoder", "[Mode  : %s]", receivedSettings.mode);

    // JP AI Auto readback. The mode byte itself is the evidence: 0x19/0x1B are
    // the automatic modes the official app labels AI Auto. The old signature
    // (data[13]==0x0A && data[14]==0x02) was withdrawn — those two bytes are
    // energy saving and sensor-directed airflow, which AI Auto merely turns on.
    if (this->Jp_ai_auto_sensor_ != nullptr) {
        const bool power_on = receivedSettings.power != nullptr && strcmp(receivedSettings.power, "ON") == 0;
        const char* jp_ai_auto_state = "OTHER";
        if (!power_on) {
            jp_ai_auto_state = "OFF";
        } else if (modeDecode.auto_direction == cn105_protocol::AutoDirection::HEATING) {
            jp_ai_auto_state = "AI_AUTO_HEATING";
        } else if (modeDecode.auto_direction == cn105_protocol::AutoDirection::COOLING) {
            jp_ai_auto_state = "AI_AUTO_COOLING";
        } else if (modeDecode.valid && modeDecode.mode == 0x08) {
            jp_ai_auto_state = "AUTO";
        }
        ESP_LOGD("Decoder", "[JP AI Auto: %s] raw mode=0x%02X", jp_ai_auto_state, data[4]);
        if (this->jp_ai_auto_state_ == nullptr || strcmp(jp_ai_auto_state, this->jp_ai_auto_state_) != 0) {
            this->jp_ai_auto_state_ = jp_ai_auto_state;
            this->Jp_ai_auto_sensor_->publish_state(jp_ai_auto_state);
        }
    }

    if (data[11] != 0x00) {
        int temp = data[11];
        temp -= 128;
        receivedSettings.temperature = (float)temp / 2;
        this->use_temperature_encoding_b_ = true;
    } else {
        auto temp_opt = cn105_protocol::lookup_value_opt(TEMP_MAP, TEMP, 16, data[5]);
        if (temp_opt) {
            receivedSettings.temperature = static_cast<float>(*temp_opt);
        } else {
            ESP_LOGW("Decoder", "Unknown temperature byte 0x%02X — keeping previous value", data[5]);
            receivedSettings.temperature = this->currentSettings.temperature;
        }
    }

    ESP_LOGD("Decoder", "[Temp °C: %f]", receivedSettings.temperature);

    auto fan_opt = cn105_protocol::lookup_value_opt(FAN_MAP, FAN, 6, data[6]);
    if (fan_opt) {
        receivedSettings.fan = *fan_opt;
    } else {
        ESP_LOGW("Decoder", "Unknown fan byte 0x%02X — keeping previous value", data[6]);
        receivedSettings.fan = this->currentSettings.fan
            ? this->currentSettings.fan
            : FAN_MAP[0];  // default to "AUTO" when no prior value exists
    }
    ESP_LOGD("Decoder", "[Fan: %s]", receivedSettings.fan);

    auto vane_opt = cn105_protocol::lookup_value_opt(VANE_MAP, VANE, 7, data[7]);
    if (vane_opt) {
        receivedSettings.vane = *vane_opt;
    } else {
        ESP_LOGW("Decoder", "Unknown vane byte 0x%02X — keeping previous value", data[7]);
        receivedSettings.vane = this->currentSettings.vane
            ? this->currentSettings.vane
            : VANE_MAP[0];  // default to "AUTO" when no prior value exists
    }
    ESP_LOGD("Decoder", "[Vane: %s]", receivedSettings.vane);

    auto left_vane_opt = cn105_protocol::lookup_value_opt(LEFT_VANE_MAP, LEFT_VANE, 7, data[15]);
    if (left_vane_opt) {
        receivedSettings.left_vane = *left_vane_opt;
    } else {
        ESP_LOGW("Decoder", "Unknown left_vane byte 0x%02X — keeping previous value", data[15]);
        receivedSettings.left_vane = this->currentSettings.left_vane;
    }
    ESP_LOGD("Decoder", "[Left Vane: %s]", receivedSettings.left_vane);

    // --- START OF MODIFIED SECTION - Reverted widevane section back to more or less original state
    if ((data[10] != 0) && (this->traits_.supports_swing_mode(climate::CLIMATE_SWING_HORIZONTAL))) {    // wideVane is not always supported
        uint8_t wideVaneByte = data[10] & 0x0F;
        auto wideVane_opt = cn105_protocol::lookup_value_opt(WIDEVANE_MAP, WIDEVANE, 11, wideVaneByte);
        if (wideVane_opt) {
            receivedSettings.wideVane = *wideVane_opt;
        } else {
            ESP_LOGW("Decoder", "Unknown wideVane byte 0x%02X — keeping previous value", wideVaneByte);
            // Guard against null: on the first settings packet currentSettings.wideVane
            // is still nullptr, and an unknown byte here would otherwise propagate a null
            // pointer into the %s log below (and downstream), panicking the ESP32.
            receivedSettings.wideVane = this->currentSettings.wideVane
                ? this->currentSettings.wideVane
                : WIDEVANE_MAP[2];  // default to "|" (center) when no prior value exists
        }
        this->wideVaneAdj = (data[10] & 0xF0) == 0x80 ? true : false;
        ESP_LOGD("Decoder", "[wideVane: %s (adj:%d)]", receivedSettings.wideVane, this->wideVaneAdj);
    } else {
        ESP_LOGD("Decoder", "widevane is not supported");
    }
    // --- END OF MODIFIED SECTION ---

    if (this->iSee_sensor_ != nullptr) {
        this->iSee_sensor_->publish_state(receivedSettings.iSee);
    }

    // --- TARGET HUMIDITY (payload 12 of the 0x02 settings packet) ---
    // Both official Mitsubishi clients read this byte as the dehumidification
    // target in percent, but only treat it as the target while the unit is in
    // DRY mode; in other modes it holds a mode-dependent value. The diagnostic
    // sensor still publishes it unconditionally (existing behaviour), while the
    // writable number only follows it in DRY mode so it does not jump around.
    const bool inDryMode = receivedSettings.mode != nullptr && strcmp(receivedSettings.mode, "DRY") == 0;
    const bool humidityIsMeaningful = inDryMode || this->humidityIsModeIndependent();
    uint8_t raw_humidity = data[12];
    if (raw_humidity > 0 && raw_humidity <= 100) {
        receivedRunStates.target_humidity = static_cast<int8_t>(raw_humidity);
        if (this->target_humidity_sensor_ != nullptr) {
            float humidity_pct = static_cast<float>(raw_humidity);
            if (this->target_humidity_sensor_->get_raw_state() != humidity_pct) {
                ESP_LOGD("Decoder", "[Target Humidity: %.0f%%]", humidity_pct);
                this->target_humidity_sensor_->publish_state(humidity_pct);
            }
        }
        if (humidityIsMeaningful) {
            if (this->target_humidity_number_ != nullptr && this->wantedRunStates.target_humidity < 0 &&
                this->currentRunStates.target_humidity != receivedRunStates.target_humidity) {
                this->target_humidity_number_->publish_state(static_cast<float>(raw_humidity));
            }
            this->currentRunStates.target_humidity = receivedRunStates.target_humidity;
        }
    } else if (raw_humidity != 0) {
        ESP_LOGD("Decoder", "[Target Humidity byte out of range: 0x%02X]", raw_humidity);
    }

    // --- ENERGY SAVING (節電, payload 13) ---
    // Written as 0x0A in payload 5 of a subtype 0x08 SET; the official decoder
    // simply tests this readback byte for a non-zero value.
    receivedRunStates.energy_saving = data[13] > 0 ? 1 : 0;
    ESP_LOGD("Decoder", "[Energy saving: %s] raw=0x%02X", receivedRunStates.energy_saving ? "ON" : "OFF", data[13]);
    if (this->wantedRunStates.energy_saving < 0) {
        this->publishRunStateSwitch(this->energy_saving_switch_, this->currentRunStates.energy_saving,
            receivedRunStates.energy_saving, "energy saving");
    }

    // --- SENSOR-DIRECTED AIRFLOW (payload 14) ---
    // The official clients read this byte unconditionally and gate the feature on
    // the D0 profile instead. Only fall back to the legacy wide-vane/i-See gate
    // when no profile was captured, so models without the feature are unaffected.
    if (this->airflow_control_select_ != nullptr) {
        const bool profileKnowsAirflow = this->profile_capabilities_.valid;
        const bool airflowReported = profileKnowsAirflow
            ? this->profile_capabilities_.supports_special_airflow()
            : (data[10] == 0x80 && receivedSettings.iSee);

        if (airflowReported) {
            auto airflow_opt = cn105_protocol::lookup_value_opt(AIRFLOW_CONTROL_MAP, AIRFLOW_CONTROL, 4, data[14]);
            if (airflow_opt) {
                receivedRunStates.airflow_control = *airflow_opt;
            } else {
                ESP_LOGW("Decoder", "Unknown airflow_control byte 0x%02X — keeping previous value", data[14]);
                receivedRunStates.airflow_control = this->currentRunStates.airflow_control;
            }
        } else {
            receivedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[0];
        }
        if (receivedRunStates.airflow_control != nullptr &&
            (!this->currentRunStates.airflow_control ||
                strcmp(receivedRunStates.airflow_control, this->currentRunStates.airflow_control) != 0)) {
            this->currentRunStates.airflow_control = receivedRunStates.airflow_control;
            this->airflow_control_select_->publish_state(receivedRunStates.airflow_control);
        }
    }

    this->heatpumpUpdate(receivedSettings);
}

/**
 * Publishes a run-state switch only when the decoded value actually changed,
 * mirroring the behaviour of the legacy HVAC option switches.
 */
void CN105Climate::publishRunStateSwitch(HVACOptionSwitch* target, int8_t& current, int8_t received, const char* label) {
    if (target == nullptr) {
        current = received;
        return;
    }
    if (current != received || target->state != (received != 0)) {
        ESP_LOGD("Decoder", "[%s : %s]", label, received ? "ON" : "OFF");
        current = received;
        target->publish_state(received != 0);
    }
}

void CN105Climate::getRoomTemperatureFromResponsePacket() {

    heatpumpStatus receivedStatus{};

    //ESP_LOGD("Decoder", "[0x03 room temperature]");
    //this->last_received_packet_sensor->publish_state("0x62-> 0x03: Data -> Room temperature");
    //                 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
    // FC 62 01 30 10 03 00 00 0E 00 94 B0 B0 FE 42 00 01 0A 64 00 00 A9
    //                         RT    OT RT ER ?? ?? ?? RM RM RM SN LG
    // RT = room temperature (in old format and in new format)
    // OT = outside air temperature
    // ER = effective room temperature — what the unit regulates on. The official
    //      client uses it instead of RT when the CD profile sets bit 0x10.
    // RM = indoor unit operating time in minutes
    // SN = bit 0: stopped-state sensing (thermal/vital sensor) is on
    // LG = bit 0: Long-airflow fan extension is on

    if (data[5] > 1) {
        receivedStatus.outsideAirTemperature = (data[5] - 128) / 2.0f;
    } else {
        receivedStatus.outsideAirTemperature = NAN;
    }

    if (this->isRawProbeCode(0x03)) {
        const float jp_oat_no_offset = data[4] / 2.0f;
        const float jp_oat_minus_8 = (data[4] - 8) / 2.0f;
        ESP_LOGD(LOG_RAW_PROBE_TAG, "0x03 candidates: room_a_data[3]=0x%02X room_b_data[6]=0x%02X oat_current_data[5]=0x%02X oat_jp_data[4]=0x%02X jp_no_offset=%.1f jp_minus_8=%.1f data[7]=0x%02X data[8]=0x%02X data[13]=0x%02X",
            data[3], data[6], data[5], data[4], jp_oat_no_offset, jp_oat_minus_8, data[7], data[8], data[13]);
    }

    // Payload 6 is the legacy room temperature and payload 7 the "effective"
    // one (what the unit is actually regulating on). The official client picks
    // between them from the CD profile bit 0x10 rather than heuristically; on
    // the captured ZW9025 the two differ by half a degree.
    const bool useEffectiveTemp = this->profile_capabilities_.uses_effective_room_temperature();
    const uint8_t roomTempByte = cn105_protocol::select_room_temperature_byte(data[6], data[7], useEffectiveTemp);

    if (roomTempByte != 0x00) {
        int temp = roomTempByte;
        temp -= 128;
        receivedStatus.roomTemperature = temp / 2.0f;
        ESP_LOGD(LOG_TEMP_SENSOR_TAG, "data[%d]  --> [Room °C: %f]",
            (useEffectiveTemp && data[7] != 0x00) ? 7 : 6, receivedStatus.roomTemperature);
    } else {
        auto room_temp_opt = cn105_protocol::lookup_value_opt(ROOM_TEMP_MAP, ROOM_TEMP, 32, data[3]);
        if (room_temp_opt) {
            receivedStatus.roomTemperature = static_cast<float>(*room_temp_opt);
        } else {
            ESP_LOGW("Decoder", "Unknown room_temp byte 0x%02X — keeping previous value", data[3]);
            receivedStatus.roomTemperature = this->currentStatus.roomTemperature;
        }
        ESP_LOGD(LOG_TEMP_SENSOR_TAG, "data[3] map --> [Room °C : %f]", receivedStatus.roomTemperature);
    }

    // Update the remote temperature control sensor (Issue 290)
    if (this->remote_temp_sensor_ != nullptr) {
        bool is_remote = false;
        if (this->remote_temp_keepalive_active_ && this->remoteTemperature_ > 0) {
            float diff = abs(receivedStatus.roomTemperature - this->remoteTemperature_);
            if (diff <= this->remote_temp_margin_) {
                is_remote = true;
            }
        }
        this->remote_temp_sensor_->publish_state(is_remote);
    }

    receivedStatus.runtimeHours = float((data[11] << 16) | (data[12] << 8) | data[13]) / 60;

    // Payload 14 bit 0 is the stopped-state sensing (thermal/vital sensor) flag
    // and payload 15 bit 0 the Long-airflow extension; both are written through
    // subtype 0x33 and read back here.
    if (this->wantedRunStates.stopped_sensing < 0) {
        this->publishRunStateSwitch(this->stopped_sensing_switch_, this->currentRunStates.stopped_sensing,
            (data[14] & 0x01) ? 1 : 0, "stopped state sensing");
    }
    if (this->wantedRunStates.long_airflow < 0) {
        this->publishRunStateSwitch(this->long_airflow_switch_, this->currentRunStates.long_airflow,
            (data[15] & 0x01) ? 1 : 0, "long airflow");
    }

    ESP_LOGD("Decoder", "[Room °C: %f]", receivedStatus.roomTemperature);
    ESP_LOGD("Decoder", "[OAT  °C: %f]", receivedStatus.outsideAirTemperature);

    // no change with this packet to currentStatus for operating and compressorFrequency
    receivedStatus.operating = currentStatus.operating;
    receivedStatus.compressorFrequency = currentStatus.compressorFrequency;
    receivedStatus.inputPower = currentStatus.inputPower;
    receivedStatus.kWh = currentStatus.kWh;
    this->statusChanged(receivedStatus);
}

void CN105Climate::getOperatingAndCompressorFreqFromResponsePacket() {
    //FC 62 01 30 10 06 00 00 1A 01 00 00 00 00 00 00 00 00 00 00 00 3C
    //MSZ-RW25VGHZ-SC1 / MUZ-RW25VGHZ-SC1
    //FC 62 01 30 10 06 00 00 00 01 00 08 05 50 00 00 42 00 00 00 00 B7
    //                           OP IP IP EU EU       ??
    // OP = operating status (1 = compressor running, 0 = standby)
    // IP = Current input power in Watts (16-bit decimal)
    // EU = energy usage
    //      (used energy in kWh = value/10)
    //      TODO: Currently the maximum size of the counter is not known and
    //            if the counter extends to other bytes.
    // ?? = unknown bytes that appear to have a fixed/constant value
    heatpumpStatus receivedStatus{};
    ESP_LOGD("Decoder", "[0x06 is status]");
    if (this->isRawProbeCode(0x06)) {
        ESP_LOGD(LOG_RAW_PROBE_TAG, "0x06 candidates: comp_data[3]=0x%02X operating_data[4]=0x%02X input_hi_data[5]=0x%02X input_lo_or_stage_data[6]=0x%02X energy_hi_data[7]=0x%02X energy_lo_data[8]=0x%02X data[9]=0x%02X data[10]=0x%02X data[11]=0x%02X data[12]=0x%02X data[13]=0x%02X data[14]=0x%02X data[15]=0x%02X",
            data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
    }
    //this->last_received_packet_sensor->publish_state("0x62-> 0x06: Data -> Heatpump Status");

    // reset counter (because a reply indicates it is connected)
    this->nonResponseCounter = 0;
    receivedStatus.operating = data[4];
    // Some models (e.g. PAA/PUZ combo) seem to have some noise on the compressor frequency sensor, even when not in operation.
    // To avoid reporting random values, set the compressor frequency to 0 when the heatpump is not operating.
    receivedStatus.compressorFrequency = (data[4]) ? data[3] : 0;
    receivedStatus.inputPower = convert_input_power_to_W(float((data[5] << 8) | data[6]));
    receivedStatus.kWh = convert_energy_usage_to_kWh(float((data[7] << 8) | data[8]));

    // no change with this packet to roomTemperature
    receivedStatus.roomTemperature = currentStatus.roomTemperature;
    receivedStatus.outsideAirTemperature = currentStatus.outsideAirTemperature;
    receivedStatus.runtimeHours = currentStatus.runtimeHours;
    this->statusChanged(receivedStatus);
}

void CN105Climate::getHVACOptionsFromResponsePacket() {
    //MSZ-LN25VG2W
    //FC 62 01 30 10 42 01 01 01 00 00 00 00 00 00 00 00 00 00 00 00 18
    //                  AP NM CL
    // AP = air purifier (1 = on, 0 = off)
    // NM = night mode (1 = on, 0 = off)
    // CL = circulator (1 = on, 0 = off) ! MIGHT BE SAME BYTE AS ECONOCOOL - NEEDS TESTING !
    heatpumpRunStates receivedRunStates{};
    ESP_LOGD("Decoder", "[0x42 is HVAC options]");

    if (this->air_purifier_switch_ != nullptr) {
        receivedRunStates.air_purifier = data[1];
        ESP_LOGD("Decoder", "[Air purifier : %s]", receivedRunStates.air_purifier ? "ON" : "OFF");
        if (receivedRunStates.air_purifier != this->currentRunStates.air_purifier || receivedRunStates.air_purifier != this->air_purifier_switch_->state) {
            this->currentRunStates.air_purifier = receivedRunStates.air_purifier;
            this->air_purifier_switch_->publish_state(receivedRunStates.air_purifier);
        }
    }
    if (this->night_mode_switch_ != nullptr) {
        receivedRunStates.night_mode = data[2];
        ESP_LOGD("Decoder", "[Night mode : %s]", receivedRunStates.night_mode ? "ON" : "OFF");
        if (receivedRunStates.night_mode != this->currentRunStates.night_mode || receivedRunStates.night_mode != this->night_mode_switch_->state) {
            this->currentRunStates.night_mode = receivedRunStates.night_mode;
            this->night_mode_switch_->publish_state(receivedRunStates.night_mode);
        }
    }
    if (this->circulator_switch_ != nullptr) {
        receivedRunStates.circulator = data[3];
        ESP_LOGD("Decoder", "[Circulator : %s]", receivedRunStates.circulator ? "ON" : "OFF");
        if (receivedRunStates.circulator != this->currentRunStates.circulator || receivedRunStates.circulator != this->circulator_switch_->state) {
            this->currentRunStates.circulator = receivedRunStates.circulator;
            this->circulator_switch_->publish_state(receivedRunStates.circulator);
        }
    }
}

void CN105Climate::decodeProfileFrame() {
    // PROFILECODE frames arrive on command 0x7B alongside the connection
    // acknowledgement. The first payload byte identifies the capability table
    // (C9/CD/D0) or the model string (D1). The tables are read-only: they say
    // what the model supports and never enable a write by themselves.
    if (!cn105_protocol::decode_profile_payload(this->parser_.data(), this->parser_.data_length(),
            this->profile_capabilities_)) {
        return;
    }

    const ProfileCapabilities& caps = this->profile_capabilities_;
    switch (this->parser_.data()[0]) {
    case 0xc9:
        ESP_LOGI("Profile", "C9 [6]=0x%02X [9]=0x%02X thermal_image=%s touch_flow=%s auto_mode=%s outside_temp=%s",
            caps.c9_6, caps.c9_9,
            caps.supports_thermal_image() ? "yes" : "no",
            caps.supports_touch_flow() ? "yes" : "no",
            caps.supports_auto_mode() ? "yes" : "no",
            caps.supports_outside_temperature() ? "yes" : "no");
        break;
    case 0xcd:
        ESP_LOGI("Profile", "CD [7]=0x%02X [8]=0x%02X [13]=0x%02X humidity_percent=%s energy_saving=%s effective_temp=%s ventilation_assist=%s serial_write=%s",
            caps.cd_7, caps.cd_8, caps.cd_13,
            caps.humidity_shown_as_percent() ? "yes" : "no",
            caps.supports_energy_saving() ? "yes" : "no",
            caps.uses_effective_room_temperature() ? "yes" : "no",
            caps.supports_ventilation_assist() ? "yes" : "no",
            caps.supports_online_serial_write() ? "yes" : "no");
        break;
    case 0xd0:
        ESP_LOGI("Profile", "D0 [1]=0x%02X [2]=0x%02X vital_sensor=%s stopped_sensing=%s fan=%s vane_ud=%s vane_lr=%s long=%s special_airflow=%s",
            caps.d0_1, caps.d0_2,
            caps.supports_vital_sensor() ? "yes" : "no",
            caps.supports_stopped_sensing() ? "yes" : "no",
            caps.displays_fan_speed() ? "yes" : "no",
            caps.displays_vertical_vane() ? "yes" : "no",
            caps.displays_horizontal_vane() ? "yes" : "no",
            caps.supports_long_airflow() ? "yes" : "no",
            caps.supports_special_airflow() ? "yes" : "no");
        break;
    case 0xd1:
        ESP_LOGI("Profile", "D1 model=%s", caps.model);
        break;
    default:
        break;
    }

    this->publishProfileSummary();
}

/**
 * Publishes the capability profile, whether it arrived as a PROFILECODE frame or was
 * supplied from YAML. JP units never send the frames, so without this a configured
 * profile would show as unavailable.
 */
void CN105Climate::publishProfileSummary() {
    if (this->profile_sensor_ == nullptr) {
        return;
    }
    const ProfileCapabilities& caps = this->profile_capabilities_;
    if (!caps.valid) {
        return;
    }
    char summary[96];
    snprintf(summary, sizeof(summary), "%s C9=%02X CD=%02X%02X D0=%02X",
        caps.model[0] != '\0' ? caps.model : "configured",
        caps.c9_6, caps.cd_7, caps.cd_8, caps.d0_2);
    this->profile_sensor_->publish_state(summary);
}

void CN105Climate::terminateCycle() {
    if (this->shouldSendExternalTemperature_) {
        // We will receive ACK packet for this.
        // Sending WantedSettings must be delayed in this case (lastSend timestamp updated).        
        ESP_LOGD(LOG_REMOTE_TEMP, "Sending remote temperature...");
        this->sendRemoteTemperature();
    }

    this->loopCycle.cycleEnded();

    this->nbCompleteCycles_++;
}
void CN105Climate::getErrorInfoFromResponsePacket() {
    ESP_LOGD("Decoder", "0x04 error info");
    if (this->error_code_sensor_ != nullptr) {
        uint8_t error_raw = this->data[4];
        uint8_t error_sub = this->data[5];
        // Bit 7 (0x80) is a protocol status flag ("error reporting available"),
        // not an actual error code. Use lower 7 bits for real error detection.
        uint8_t error_code = cn105_protocol::decode_error_code(error_raw);
        if (error_code == 0x00 && error_sub == 0x00) {
            this->error_code_sensor_->publish_state("No Error");
        } else {
            char buf[32];
            snprintf(buf, sizeof(buf), "Error 0x%02X sub 0x%02X", error_code, error_sub);
            this->error_code_sensor_->publish_state(buf);
        }
    }
}

void CN105Climate::getDataFromResponsePacket() {

    // Let the request scheduler handle registered response codes first.
    const uint8_t code = this->data[0];
    if (this->scheduler_.process_response(code)) {
        return;
    }
    // Fall back to response codes that are not registered with the scheduler.
    switch (code) {

    case 0x04:
        // Handled by orchestrator (r_error_info onResponse → getErrorInfoFromResponsePacket)
        // Reaching here means the scheduler did not intercept this response — unexpected
        ESP_LOGW("Decoder", "[0x04] reached switch fallback — should have been handled by orchestrator");
        break; // orchestrator

    case 0x05:
        /* timer packet */
        ESP_LOGW("Decoder", "[0x05 is Timer : not implemented]");
        //this->last_received_packet_sensor->publish_state("0x62-> 0x05: Data -> Timer Packet");
        break;

    case 0x06:
        break; // orchestrator
    case 0x09:
        break; // orchestrator

    case 0x10:
        ESP_LOGD("Decoder", "[0x10 is Unknown : not implemented]");
        //this->getAutoModeStateFromResponsePacket();
        break;

    case 0x20: // fallthrough
    case 0x22:
        break; // orchestrator

    case 0x42:
        break; // orchestrator

    default:
        ESP_LOGW("Decoder", "packet type [%02X] <-- unknown and unexpected", data[0]);
        //this->last_received_packet_sensor->publish_state("0x62-> ?? : Data -> Unknown");
        break;
    }

}

void CN105Climate::updateSuccess() {
    ESP_LOGD(LOG_ACK, "Last heatpump data update successful!");
    // nothing can be done here because we have no mean to know wether it is an external temp ack
    // or a wantedSettings update ack
}

void CN105Climate::processCommand() {
    switch (this->parser_.command()) {
    case 0x61:  /* last update was successful */
        this->hpPacketDebug(this->parser_.raw(), this->parser_.frame_size(), LOG_ACK);
        this->updateSuccess();
        break;

    case 0x62:  /* packet contains data (room °C, settings, timer, status, or functions...)*/
        this->getDataFromResponsePacket();
        break;
    case 0x7a:  // Connection success (User / standard)
    case 0x7b:  // Connection success (Installer / extended)
        this->decodeProfileFrame();
        // Log the event on its INFO tag and packet details through hpPacketDebug.
        ESP_LOGI(LOG_CONN_TAG, "--> Heatpump did reply: connection success (%s, 0x%02X)! <--",
            (this->parser_.command() == 0x7b) ? "Installer" : "User",
            this->parser_.command());
        this->hpPacketDebug(this->parser_.raw(), this->parser_.frame_size(), LOG_CONN_TAG);
        // isHeatpumpConnected_ replaced by FSM transition in setHeatpumpConnected()
        this->setHeatpumpConnected(true);
        // let's say that the last complete cycle was over now
        this->loopCycle.lastCompleteCycleMs = CUSTOM_MILLIS;
        this->currentSettings.resetSettings();      // each time we connect, we need to reset current setting to force a complete sync with ha component state and receievdSettings
        this->currentRunStates.resetSettings();
        break;
    default:
        break;
    }
}


void CN105Climate::statusChanged(heatpumpStatus status) {

    if (status != currentStatus) {
        this->debugStatus("received", status);
        this->debugStatus("current", currentStatus);


        this->currentStatus.operating = status.operating;
        this->currentStatus.compressorFrequency = status.compressorFrequency;
        this->currentStatus.inputPower = status.inputPower;
        this->currentStatus.kWh = status.kWh;
        this->currentStatus.runtimeHours = status.runtimeHours;
        this->currentStatus.roomTemperature = status.roomTemperature;
        this->currentStatus.outsideAirTemperature = status.outsideAirTemperature;
        this->setCurrentTemperature(this->currentStatus.roomTemperature);

        this->updateAction();       // update action info on HA climate component
        this->publish_state();

        if (this->compressor_frequency_sensor_ != nullptr) {
            this->compressor_frequency_sensor_->publish_state(currentStatus.compressorFrequency);
        }

        if (this->input_power_sensor_ != nullptr) {
            this->input_power_sensor_->publish_state(currentStatus.inputPower);
        }

        if (this->kwh_sensor_ != nullptr) {
            this->kwh_sensor_->publish_state(currentStatus.kWh);
        }

        if (this->runtime_hours_sensor_ != nullptr) {
            this->runtime_hours_sensor_->publish_state(currentStatus.runtimeHours);
        }

        if (this->outside_air_temperature_sensor_ != nullptr) {
            this->outside_air_temperature_sensor_->publish_state(this->fahrenheitSupport_.normalizeHeatpumpTemperatureToUiTemperature(currentStatus.outsideAirTemperature));
        }
    } // else no change
}


void CN105Climate::publishStateToHA(heatpumpSettings& settings) {

    if ((this->wantedSettings.mode == nullptr) && (this->wantedSettings.power == nullptr)) {        // to prevent overwriting a user demand
        checkPowerAndModeSettings(settings);
    }

    this->updateAction();       // update action info on HA climate component

    if (this->wantedSettings.fan == nullptr) {  // to prevent overwriting a user demand
        checkFanSettings(settings);
    }

    if (this->wantedSettings.vane == nullptr) { // to prevent overwriting a user demand
        checkVaneSettings(settings);
    }

    if (this->wantedRunStates.left_vane == nullptr) { // to prevent overwriting a user demand
        currentSettings.left_vane = settings.left_vane;
    }

    if (this->wantedSettings.wideVane == nullptr) { // to prevent overwriting a user demand
        checkWideVaneSettings(settings);
    }

    // HA Temp
    // Temporarily ignore an incoming setpoint while a user request is pending.
    bool hasPendingUserTemp = (this->wantedSettings.temperature != -1.0f) && (this->wantedSettings.hasChanged) && (!this->wantedSettings.hasBeenSent);
    uint32_t graceWindowMs = this->get_update_interval() + DEFER_SCHEDULE_UPDATE_LOOP_DELAY;
    bool graceAfterSend = (this->wantedSettings.hasBeenSent) && ((CUSTOM_MILLIS - this->wantedSettings.lastChange) < graceWindowMs);
    if (!hasPendingUserTemp && !graceAfterSend) {
        if (this->wantedSettings.temperature == -1) { // to prevent overwriting a user demand
            this->updateTargetTemperaturesFromSettings(settings.temperature);
            this->currentSettings.temperature = settings.temperature;
        }
    } else {
        ESP_LOGD(LOG_SETTINGS_TAG, "Ignoring incoming setpoint due to pending user change or grace window");
    }

    this->currentSettings.iSee = settings.iSee;

    this->currentSettings.connected = true;

    // publish to HA
    this->publish_state();

}



void CN105Climate::heatpumpUpdate(heatpumpSettings& settings) {
    // settings correponds to current settings
    ESP_LOGV(LOG_SETTINGS_TAG, "Settings received");
    // if received settings are different from current settings 
    if (settings != this->currentSettings) {
        ESP_LOGI(LOG_SETTINGS_TAG, "Settings changed, updating HA states");
        this->debugSettings("current", this->currentSettings);
        this->debugSettings("received", settings);
        this->debugSettings("wanted", this->wantedSettings);
        this->debugClimate("climate");
        this->publishStateToHA(settings);
    }

}

void CN105Climate::checkVaneSettings(heatpumpSettings& settings, bool updateCurrentSettings) {
    if (this->hasChanged(currentSettings.vane, settings.vane, "vane")) {    // widevane setting change ?
        ESP_LOGI(LOG_SETTINGS_TAG, "vane setting changed");

        //this->debugSettings("settings", settings);

        if (updateCurrentSettings) {
            //ESP_LOGD(LOG_SETTINGS_TAG, "updating currentSetting with new value");
            currentSettings.vane = settings.vane;
        }

        if (strcmp(settings.vane, "SWING") == 0) {
            if ((currentSettings.wideVane != nullptr) && (strcmp(currentSettings.wideVane, "SWING") == 0)) {
                this->swing_mode = climate::CLIMATE_SWING_BOTH;
            } else {
                this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
            }
        } else {
            if ((currentSettings.wideVane != nullptr) && (strcmp(currentSettings.wideVane, "SWING") == 0)) {
                this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
            } else {
                this->swing_mode = climate::CLIMATE_SWING_OFF;
            }
        }
        ESP_LOGD(LOG_SETTINGS_TAG, "Swing mode is: %i", this->swing_mode);
    }


    updateExtraSelectComponents(settings);
}

void CN105Climate::checkWideVaneSettings(heatpumpSettings& settings, bool updateCurrentSettings) {

    /* ******** HANDLE MITSUBISHI VANE CHANGES ********
     * VANE_MAP[7]        = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
     * WIDEVANE_MAP[8]   = { "<<", "<",  "|",  ">",  ">>", "<>", "SWING", "AIRFLOW CONTROL" }
     */

    if (this->hasChanged(currentSettings.wideVane, settings.wideVane, "wideVane")) {    // widevane setting change ?
        ESP_LOGI(TAG, "widevane setting changed");
        this->debugSettings("settings", settings);

        // here I hope that the vane and widevane are always sent together
        if (updateCurrentSettings) {
            currentSettings.wideVane = settings.wideVane;
        }

        if (strcmp(settings.wideVane, "SWING") == 0) {
            if ((currentSettings.vane != nullptr) && (strcmp(currentSettings.vane, "SWING") == 0)) {
                this->swing_mode = climate::CLIMATE_SWING_BOTH;
            } else {
                this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
            }
        } else {
            if ((currentSettings.vane != nullptr) && (strcmp(currentSettings.vane, "SWING") == 0)) {
                this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
            } else {
                this->swing_mode = climate::CLIMATE_SWING_OFF;
            }
        }
        ESP_LOGD(TAG, "Swing mode is: %i", this->swing_mode);
    }

    /*if (this->hasChanged(this->van_orientation->state.c_str(), settings.vane, "select vane")) {
        ESP_LOGI(TAG, "vane setting (extra select component) changed");
        this->van_orientation->publish_state(currentSettings.vane);
    }*/

    updateExtraSelectComponents(settings);
}
void CN105Climate::updateExtraSelectComponents(heatpumpSettings& settings) {
    if (this->vertical_vane_select_ != nullptr) {
        if (this->hasChanged(this->vertical_vane_select_->current_option(), settings.vane, "select vane")) {
            ESP_LOGI(TAG, "vane setting (extra select component) changed");
            this->vertical_vane_select_->publish_state(settings.vane);
        }
    }
    if (this->left_vane_select_ != nullptr) {
        if (this->hasChanged(this->left_vane_select_->current_option(), settings.left_vane, "select left vane")) {
            ESP_LOGI(TAG, "left vane setting (extra select component) changed");
            this->left_vane_select_->publish_state(settings.left_vane);
        }
    }
    if (this->horizontal_vane_select_ != nullptr) {
        if (this->hasChanged(this->horizontal_vane_select_->current_option(), settings.wideVane, "select wideVane")) {
            ESP_LOGI(TAG, "widevane setting (extra select component) changed");
            this->horizontal_vane_select_->publish_state(settings.wideVane);
        }
    }
}
void CN105Climate::checkFanSettings(heatpumpSettings& settings, bool updateCurrentSettings) {
    /*
         * ******* HANDLE FAN CHANGES ********
         *
         * const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
         */
         // currentSettings.fan== NULL is true when it is the first time we get en answer from hp

    if (this->hasChanged(currentSettings.fan, settings.fan, "fan")) { // fan setting change ?
        ESP_LOGI(TAG, "fan setting changed");
        if (updateCurrentSettings) {
            currentSettings.fan = settings.fan;
        }

        if (strcmp(settings.fan, "QUIET") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_QUIET;
        } else if (strcmp(settings.fan, "1") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_LOW;
        } else if (strcmp(settings.fan, "2") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
        } else if (strcmp(settings.fan, "3") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
        } else if (strcmp(settings.fan, "4") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_HIGH;
        } else { //case "AUTO" or default:
            this->fan_mode = climate::CLIMATE_FAN_AUTO;
        }
        if (this->fan_mode.has_value()) {
            ESP_LOGD(TAG, "Fan mode is: %i", static_cast<int>(this->fan_mode.value()));
        } else {
            ESP_LOGD(TAG, "Fan mode is not set");
        }
    }
}


void CN105Climate::checkPowerAndModeSettings(heatpumpSettings& settings, bool updateCurrentSettings) {
    // currentSettings.power== NULL is true when it is the first time we get en answer from hp
    if (this->hasChanged(currentSettings.power, settings.power, "power") ||
        this->hasChanged(currentSettings.mode, settings.mode, "mode")) {           // mode or power change ?

        ESP_LOGI(TAG, "power or mode changed");
        if (updateCurrentSettings) {
            currentSettings.power = settings.power;
            currentSettings.mode = settings.mode;
        }
        if (strcmp(settings.power, "ON") == 0) {
            if (strcmp(settings.mode, "HEAT") == 0) {
                // A dual-setpoint unit driven in HEAT_COOL runs the heat pump in
                // hardware AUTO, and the unit reports its *active operating
                // direction* ("HEAT" here) back in the settings packet. Letting
                // that overwrite this->mode silently drops the user out of
                // HEAT_COOL and collapses the dual band to a single setpoint
                // (updateTargetTemperaturesFromSettings then runs single-setpoint).
                // Keep HEAT_COOL; the operating direction is surfaced separately
                // by the auto_sub_mode sensor.
                if (!(this->supports_dual_setpoint_ &&
                      this->mode == climate::CLIMATE_MODE_HEAT_COOL)) {
                    this->mode = climate::CLIMATE_MODE_HEAT;
                }
            } else if (strcmp(settings.mode, "DRY") == 0) {
                this->mode = climate::CLIMATE_MODE_DRY;
            } else if (strcmp(settings.mode, "COOL") == 0) {
                // Same as the HEAT branch: hardware AUTO reports "COOL" as the
                // active operating direction; don't let it clobber HEAT_COOL.
                if (!(this->supports_dual_setpoint_ &&
                      this->mode == climate::CLIMATE_MODE_HEAT_COOL)) {
                    this->mode = climate::CLIMATE_MODE_COOL;
                }
                /*if (cool_setpoint != currentSettings.temperature) {
                    cool_setpoint = currentSettings.temperature;
                    save(currentSettings.temperature, cool_storage);
                }*/
            } else if (strcmp(settings.mode, "FAN") == 0) {
                this->mode = climate::CLIMATE_MODE_FAN_ONLY;
            } else if (strcmp(settings.mode, "AUTO") == 0) {
                // If we were in HEAT_COOL via HA, stay in HEAT_COOL even if HP says AUTO
                if (this->mode != climate::CLIMATE_MODE_HEAT_COOL) {
                    this->mode = climate::CLIMATE_MODE_AUTO;
                }
            } else {
                ESP_LOGW(
                    TAG,
                    "Unknown climate mode value %s received from HeatPump",
                    settings.mode
                );
            }
        } else {
            this->mode = climate::CLIMATE_MODE_OFF;
        }
    }
}
