#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace esphome {
    enum class FahrenheitMode {
        OFF = 0,
        STANDARD = 1,
        ALT = 2
    };

    class FahrenheitSupport {
    public:
        void setUseFahrenheitSupportMode(FahrenheitMode mode) {
            fahrenheit_mode_ = mode;
        }

        float normalizeHeatpumpTemperatureToUiTemperature(const float c) {
            if (fahrenheit_mode_ == FahrenheitMode::OFF || std::isnan(c)) {
                return c; // If disabled, return the Celsius value as is.
            }

            // Select the appropriate conversion table based on mode
            const std::vector<std::pair<float, float>>& localTable =
                (fahrenheit_mode_ == FahrenheitMode::ALT) ? fahrenheitToCelsiusTableAlt : fahrenheitToCelsiusTable;

            auto it = std::upper_bound(
                localTable.begin(),
                localTable.end(),
                std::make_pair(0.0f, c),
                [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                    return a.second < b.second;  // compare Celsius
                }
            );

            if (it == localTable.begin() || it == localTable.end()) {
                return c;
            }

            auto prev = it;
            --prev;
            float fahrenheitResult = std::abs(prev->second - c) < std::abs(it->second - c) ? prev->first : it->first;

            return (fahrenheitResult - 32.0f) / 1.8f;
        }

        float normalizeUiTemperatureToHeatpumpTemperature(const float c) {
            if (fahrenheit_mode_ == FahrenheitMode::OFF || std::isnan(c)) {
                return c; // If disabled, return the Celsius value as is.
            }

            // Select the appropriate conversion table based on mode
            const std::vector<std::pair<float, float>>& localTable =
                (fahrenheit_mode_ == FahrenheitMode::ALT) ? fahrenheitToCelsiusTableAlt : fahrenheitToCelsiusTable;

            float fahrenheitInput = (c * 1.8f) + 32.0f;

            // Due to vagaries of floating point math across architectures, we can't
            // just look up `c` in the map -- we're very unlikely to find a matching
            // value. Instead, we find the first value greater than `c`, and the
            // next-lowest value in the map. We return whichever `c` is closer to.
            auto it = std::upper_bound(localTable.begin(), localTable.end(), std::make_pair(fahrenheitInput, 0.0f),
                [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                    return a.first < b.first;
                });
            if (it == localTable.begin() || it == localTable.end()) {
                return c;
            }

            auto prev = it;
            --prev;

            return std::abs(prev->first - fahrenheitInput) < std::abs(it->first - fahrenheitInput) ? prev->second : it->second;
        }

    private:
        FahrenheitMode fahrenheit_mode_ = FahrenheitMode::OFF;

        // Given a temperature in Celsius that was converted from Fahrenheit, converts
        // it to the Celsius value (at half-degree precision) that matches what
        // Mitsubishi thermostats would have converted the Fahrenheit value to. For
        // instance, 72°F is 22.22°C, but this class returns 22.5°C.
        const std::vector<std::pair<float, float>> fahrenheitToCelsiusTable = {
            {61.0f, 16.0f}, {62.0f, 16.5f}, {63.0f, 17.0f}, {64.0f, 17.5f}, {65.0f, 18.0f},
            {66.0f, 18.5f}, {67.0f, 19.0f}, {68.0f, 20.0f}, {69.0f, 21.0f}, {70.0f, 21.5f},
            {71.0f, 22.0f}, {72.0f, 22.5f}, {73.0f, 23.0f}, {74.0f, 23.5f}, {75.0f, 24.0f},
            {76.0f, 24.5f}, {77.0f, 25.0f}, {78.0f, 25.5f}, {79.0f, 26.0f}, {80.0f, 26.5f},
            {81.0f, 27.0f}, {82.0f, 27.5f}, {83.0f, 28.0f}, {84.0f, 28.5f}, {85.0f, 29.0f},
            {86.0f, 29.5f}, {87.0f, 30.0f}, {88.0f, 30.5f}
        };
        const std::vector<std::pair<float, float>> fahrenheitToCelsiusTableAlt = {
            {61.0f, 16.0f}, {62.0f, 16.5f}, {63.0f, 17.0f}, {64.0f, 18.0f}, {65.0f, 18.5f},
            {66.0f, 19.0f}, {67.0f, 19.5f}, {68.0f, 20.0f}, {69.0f, 20.5f}, {70.0f, 21.0f},
            {71.0f, 21.5f}, {72.0f, 22.0f}, {73.0f, 23.0f}, {74.0f, 23.5f}, {75.0f, 24.0f},
            {76.0f, 24.5f}, {77.0f, 25.0f}, {78.0f, 25.5f}, {79.0f, 26.0f}, {80.0f, 26.5f},
            {81.0f, 27.0f}, {82.0f, 28.0f}, {83.0f, 28.5f}, {84.0f, 29.0f}, {85.0f, 29.5f},
            {86.0f, 30.0f}, {87.0f, 30.5f}, {88.0f, 31.0f}
        };
    };
}
