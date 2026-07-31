#pragma once

#include <cstdint>
#include <string>

#include "chili_crane_core/validity.hpp"

namespace chili_crane_perception {

enum class GrabTrackingState : std::uint8_t {
    UNKNOWN = 0,
    CANDIDATE,
    TRACKED,
    LOST_HOLD,
    LOST,
};

enum class GrabOpeningState : std::uint8_t {
    UNKNOWN = 0,
    OPEN,
    CLOSED,
    TRANSITION,
    CONFLICT,
};

struct GrabGeometryProfile {
    std::string profile_id;
    GrabOpeningState opening_state = GrabOpeningState::UNKNOWN;
    double length_m = 0.0;
    double width_m = 0.0;
    double height_m = 0.0;
    chili_crane_core::Validity validity =
        chili_crane_core::Validity::NOT_CONFIGURED;
};

}  // namespace chili_crane_perception
