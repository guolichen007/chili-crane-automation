#pragma once

#include <array>
#include <string>
#include <vector>

#include "chili_crane_core/validity.hpp"

namespace chili_crane_core {

struct Point2d {
    double x_m = 0.0;
    double y_m = 0.0;
};

struct Point3d {
    double x_m = 0.0;
    double y_m = 0.0;
    double z_m = 0.0;
};

struct Pose2d {
    Point2d position_map;
    double yaw_rad = 0.0;
    Validity validity = Validity::NOT_CONFIGURED;
};

struct PitDefinition {
    std::string pit_id;
    std::vector<Point2d> polygon_map;
    double wall_top_z_m = 0.0;
    double pit_bottom_z_m = 0.0;
    double safety_inset_m = 0.0;
    Validity validity = Validity::NOT_CONFIGURED;
};

struct UnloadingStationDefinition {
    std::string station_id;
    Pose2d target_pose_map;
    std::vector<Point2d> roi_polygon_map;
    Validity validity = Validity::NOT_CONFIGURED;
};

struct RestrictedRegionDefinition {
    std::string region_id;
    std::vector<Point2d> polygon_map;
    std::string reason;
    Validity validity = Validity::NOT_CONFIGURED;
};

struct CalibrationAnchorDefinition {
    std::string anchor_id;
    Point3d position_map;
    std::string kind;
    Validity validity = Validity::NOT_CONFIGURED;
};

struct TrackDefinition {
    std::string track_id;
    Point2d origin_map;
    Point2d direction_map;
    std::array<double, 2> valid_x_range_m{{0.0, 0.0}};
    Validity validity = Validity::NOT_CONFIGURED;
};

struct SemanticMap {
    std::string map_id;
    std::string version;
    std::string frame_id = "map";
    TrackDefinition track;
    std::vector<PitDefinition> pits;
    std::vector<UnloadingStationDefinition> unloading_stations;
    Pose2d safe_wait_pose;
    std::vector<RestrictedRegionDefinition> restricted_regions;
    std::vector<CalibrationAnchorDefinition> calibration_anchors;
    Validity validity = Validity::NOT_CONFIGURED;
};

}  // namespace chili_crane_core
