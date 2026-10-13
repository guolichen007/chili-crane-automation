#pragma once

#include <string>
#include <vector>

#include "chili_crane_core/validity.hpp"

namespace chili_crane_core {

// References SemanticMap V1, never owns a second copy of PitDefinition geometry.
struct GeofenceProvenance {
    std::string geofence_id;
    std::string geofence_version;
    std::string map_id;
    std::string semantic_version;
    std::string pit_id;
    std::string x_calibration_id;
    std::string y_calibration_id;
    std::string axis_to_map_calibration_id;
    std::string config_version;
    std::string config_sha256;
};

// Diagnostic necessary-condition evaluation; deliberately NOT a SafetyPermit.
struct OperationalGeofenceShadow {
    GeofenceProvenance provenance;
    Validity validity = Validity::NOT_CONFIGURED;
    std::vector<std::string> reasons;
    bool x_geometry_eligible = false;
    bool y_geometry_eligible = false;
    bool inside = false;
    bool x_positive = false;
    bool x_negative = false;
    bool y_positive = false;
    bool y_negative = false;
    const bool valid_for_control = false;
};

}  // namespace chili_crane_core
