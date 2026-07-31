// Diagnostic sensors decoded from the subtype 0x09 status response.
//
// UnitActivitySensor turns payload 5 bit 0x04 -- the official client's
// `isStoppingOperationMode` -- into a readable status. That bit is set while the
// unit is doing real work after being switched off, which on JP models covers the
// internal-clean / mould-guard cycle, the filter self-cleaning mechanism, and the
// periodic fan of the high/low temperature watch. A single bit cannot say which of
// the three is running, so the label covers all of them.
//
// MultiStandbySensor reports payload 3 bit 0x08, the multi-split standby state.
#pragma once
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome {

    // Reported states. RUNNING means the unit is powered on, in which case the
    // climate entity already describes what it is doing.
    static const char* UNIT_ACTIVITY_RUNNING = "Running";
    static const char* UNIT_ACTIVITY_CLEANING = "Cleaning / protection";
    static const char* UNIT_ACTIVITY_IDLE = "Idle";

    class UnitActivitySensor : public text_sensor::TextSensor, public Component {
    };

    class MultiStandbySensor : public binary_sensor::BinarySensor, public Component {
    };

}  // namespace esphome
