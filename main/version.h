#pragma once

#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  0
#define FW_VERSION_PATCH  0
#define FW_VERSION_STR    "1.0.0"

#define FW_BUILD_DATE  __DATE__
#define FW_BUILD_TIME  __TIME__

// Stringa completa es. "1.0.0 (May 28 2026 12:00:00)" — log, BLE, coach
#define FW_VERSION_FULL  FW_VERSION_STR " (" FW_BUILD_DATE " " FW_BUILD_TIME ")"
