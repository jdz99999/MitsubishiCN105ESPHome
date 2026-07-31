// Diagnostic binary sensors decoded from the subtype 0x09 status response.
//
// SpecialStoppingSensor reports payload 5 bit 0x04 — the official client's
// `isStoppingOperationMode`. It is set while the unit is doing real work after
// being switched off: the internal-clean / mould-guard cycle, the filter
// self-cleaning mechanism, or the periodic fan of the temperature watch.
//
// MultiStandbySensor reports payload 3 bit 0x08, the multi-split standby state.
#pragma once
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

namespace esphome {

    class SpecialStoppingSensor : public binary_sensor::BinarySensor, public Component {
    };

    class MultiStandbySensor : public binary_sensor::BinarySensor, public Component {
    };

}  // namespace esphome
