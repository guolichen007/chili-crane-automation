#pragma once

#include <cstdint>

namespace chili_crane_slam {

enum class RuntimeMode : std::uint8_t {
    NOT_CONFIGURED = 0,
    MAPPING = 1,
    LOCALIZATION = 2,
    REPLAY = 3,
};

inline bool mapMutationAllowed(RuntimeMode mode) noexcept {
    return mode == RuntimeMode::MAPPING;
}

}  // namespace chili_crane_slam
