// Diagnostic sensor: model identity and capability bytes read from the
// PROFILECODE frames the indoor unit sends during the 0x7B handshake.
#pragma once
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

using namespace esphome;

class ProfileSensor : public text_sensor::TextSensor, public Component {
};
