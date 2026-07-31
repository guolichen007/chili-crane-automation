#pragma once

#include <cstddef>

#include "chili_crane_core/validity.hpp"

namespace chili_crane_perception {

struct PitSurfaceCell {
    double robust_height_m = 0.0;
    double roughness_m = 0.0;
    double slope_rad = 0.0;
    std::size_t point_count = 0U;
    double coverage = 0.0;
    chili_crane_core::Validity validity =
        chili_crane_core::Validity::UNKNOWN;
};

struct GraspCandidateScore {
    double material_height = 0.0;
    double footprint_coverage = 0.0;
    double wall_clearance = 0.0;
    double roughness = 0.0;
    double reachability = 0.0;
    double visibility = 0.0;
    double recent_grab_penalty = 0.0;
    chili_crane_core::Validity validity =
        chili_crane_core::Validity::UNKNOWN;
};

}  // namespace chili_crane_perception
