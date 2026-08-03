#include <gtest/gtest.h>

#include "chili_crane_core/semantic_map.hpp"
#include "chili_crane_core/task_state.hpp"
#include "chili_crane_core/task_transition_policy.hpp"
#include "chili_crane_core/validity.hpp"

TEST(CoreContractCompileTest, PublicHeadersInstantiate) {
    chili_crane_core::SemanticMap semantic_map;
    EXPECT_EQ(semantic_map.schema_version, 1U);
    EXPECT_EQ(
        semantic_map.config_state,
        chili_crane_core::Validity::NOT_CONFIGURED);
    EXPECT_STREQ(
        chili_crane_core::validityName(chili_crane_core::Validity::VALID),
        "VALID");
    EXPECT_TRUE(chili_crane_core::canTransitionTaskState(
        chili_crane_core::TaskState::IDLE,
        chili_crane_core::TaskState::TASK_ACCEPTED));
}
