/// esphome_stubs.h — Minimal stubs for compiling CN105 functions without ESPHome.
/// Deps: <cstdio>, <cstdint>, <cstring>, <cmath>, <string>, <functional>
#pragma once

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <functional>

// ESPHome logging macro stubs are no-ops in unit tests.
#define ESP_LOGD(tag, fmt, ...) ((void)0)
#define ESP_LOGI(tag, fmt, ...) ((void)0)
#define ESP_LOGW(tag, fmt, ...) ((void)0)
#define ESP_LOGE(tag, fmt, ...) ((void)0)
#define ESP_LOGV(tag, fmt, ...) ((void)0)
