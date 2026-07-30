// Diagnostic sensor: direction the unit picked while running an automatic mode.
// Decoded from payload 5 of the 0x09 response (0x18 heating, 0x28 cooling).
#pragma once
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

using namespace esphome;

class AutoDirectionSensor : public text_sensor::TextSensor, public Component {
};
