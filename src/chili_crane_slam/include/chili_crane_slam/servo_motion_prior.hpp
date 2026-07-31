#pragma once

#include <string>

#include "chili_crane_core/validity.hpp"

namespace chili_crane_slam {

struct ServoCalibration {
    double direction = 0.0;
    double scale_m_per_raw_unit = 0.0;
    double offset_m = 0.0;
    std::string calibration_id;
    chili_crane_core::Validity validity =
        chili_crane_core::Validity::NOT_CONFIGURED;
};

struct ServoMotionPrior {
    double stamp_sec = 0.0;
    double map_x_m = 0.0;
    double delta_x_m = 0.0;
    double velocity_mps = 0.0;
    double variance_m2 = 0.0;
    bool homed = false;
    bool fault = false;
    chili_crane_core::Validity validity =
        chili_crane_core::Validity::UNKNOWN;
};

}  // namespace chili_crane_slam
