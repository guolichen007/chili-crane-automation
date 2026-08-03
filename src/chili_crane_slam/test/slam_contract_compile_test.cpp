#include <gtest/gtest.h>

#include "chili_crane_slam/dual_lidar_contract.hpp"
#include "chili_crane_slam/runtime_mode.hpp"
#include "chili_crane_slam/servo_motion_prior.hpp"

TEST(SlamContractCompileTest, PublicHeadersInstantiate) {
    chili_crane_slam::DualLidarSyncContract lidar_contract;
    chili_crane_slam::ServoMotionPrior servo_prior;
    EXPECT_FALSE(lidar_contract.single_lidar_fallback_enabled);
    EXPECT_FALSE(servo_prior.homed);
    EXPECT_TRUE(chili_crane_slam::mapMutationAllowed(
        chili_crane_slam::RuntimeMode::MAPPING));
    EXPECT_FALSE(chili_crane_slam::mapMutationAllowed(
        chili_crane_slam::RuntimeMode::LOCALIZATION));
}
