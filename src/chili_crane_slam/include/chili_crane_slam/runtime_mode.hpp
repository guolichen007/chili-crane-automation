#pragma once

#include <cstdint>

namespace chili_crane_slam {

enum class RuntimeMode : std::uint8_t {
    MAPPING = 0,
    LOCALIZATION = 1,
    REPLAY = 2,
};

inline bool mapMutationAllowed(RuntimeMode mode) noexcept {
    return mode == RuntimeMode::MAPPING;
}

}  // namespace chili_crane_slam
