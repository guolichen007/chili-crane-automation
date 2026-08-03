#include <gtest/gtest.h>

#include "chili_crane_perception/grab_tracking_contract.hpp"
#include "chili_crane_perception/pit_surface_contract.hpp"

TEST(PerceptionContractCompileTest, PublicHeadersInstantiate) {
    chili_crane_perception::GrabGeometryProfile grab_profile;
    chili_crane_perception::PitSurfaceCell surface_cell;
    chili_crane_perception::GraspCandidateScore candidate_score;
    EXPECT_EQ(
        grab_profile.opening_state,
        chili_crane_perception::GrabOpeningState::UNKNOWN);
    EXPECT_EQ(surface_cell.point_count, 0U);
    EXPECT_DOUBLE_EQ(candidate_score.wall_clearance, 0.0);
}
