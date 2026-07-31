#pragma once

#include <cstddef>
#include <string>

#include "chili_crane_core/validity.hpp"

namespace chili_crane_slam {

struct DualLidarSyncContract {
    std::string left_topic;
    std::string right_topic;
    std::string output_frame;
    double maximum_pair_delta_sec = 0.0;
    double stale_timeout_sec = 0.0;
    std::size_t maximum_queue_size = 0U;
    bool single_lidar_fallback_enabled = false;
    chili_crane_core::Validity validity =
        chili_crane_core::Validity::NOT_CONFIGURED;
};

struct DualLidarDiagnostics {
    double pair_delta_sec = 0.0;
    double evidence_age_sec = 0.0;
    std::size_t paired_count = 0U;
    std::size_t fallback_count = 0U;
    std::size_t dropped_count = 0U;
    bool reused_old_frame = false;
    chili_crane_core::Validity validity =
        chili_crane_core::Validity::UNKNOWN;
};

}  // namespace chili_crane_slam
