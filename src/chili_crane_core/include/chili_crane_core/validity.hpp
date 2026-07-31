#pragma once

#include <cstdint>

namespace chili_crane_core {

enum class Validity : std::uint8_t {
    UNKNOWN = 0,
    NOT_CONFIGURED = 1,
    STALE = 2,
    DEGRADED = 3,
    VALID = 4,
};

inline const char* validityName(Validity validity) noexcept {
    switch (validity) {
        case Validity::UNKNOWN:
            return "UNKNOWN";
        case Validity::NOT_CONFIGURED:
            return "NOT_CONFIGURED";
        case Validity::STALE:
            return "STALE";
        case Validity::DEGRADED:
            return "DEGRADED";
        case Validity::VALID:
            return "VALID";
    }
    return "UNKNOWN";
}

inline bool isUsable(Validity validity) noexcept {
    return validity == Validity::VALID;
}

}  // namespace chili_crane_core
