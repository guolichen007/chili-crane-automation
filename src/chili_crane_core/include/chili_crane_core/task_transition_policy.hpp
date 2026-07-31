#pragma once

#include "chili_crane_core/task_state.hpp"

namespace chili_crane_core {

inline bool isSequentialTaskTransition(
    TaskState from, TaskState to) noexcept {
    return
        (from == TaskState::IDLE &&
         to == TaskState::TASK_ACCEPTED) ||
        (from == TaskState::TASK_ACCEPTED &&
         to == TaskState::LOCALIZATION_READY) ||
        (from == TaskState::LOCALIZATION_READY &&
         to == TaskState::NAVIGATING_TO_PIT) ||
        (from == TaskState::NAVIGATING_TO_PIT &&
         to == TaskState::PIT_POSITIONED) ||
        (from == TaskState::PIT_POSITIONED &&
         to == TaskState::SCANNING_PIT) ||
        (from == TaskState::SCANNING_PIT &&
         to == TaskState::GRASP_TARGET_READY) ||
        (from == TaskState::GRASP_TARGET_READY &&
         to == TaskState::POSITIONING_TROLLEY) ||
        (from == TaskState::POSITIONING_TROLLEY &&
         to == TaskState::WAITING_STABLE) ||
        (from == TaskState::WAITING_STABLE &&
         to == TaskState::LOWERING) ||
        (from == TaskState::LOWERING &&
         to == TaskState::CONTACT_DETECTED) ||
        (from == TaskState::CONTACT_DETECTED &&
         to == TaskState::CLOSING_GRAB) ||
        (from == TaskState::CLOSING_GRAB &&
         to == TaskState::LOAD_VERIFY) ||
        (from == TaskState::LOAD_VERIFY &&
         to == TaskState::RAISING_TO_SAFE_HEIGHT) ||
        (from == TaskState::RAISING_TO_SAFE_HEIGHT &&
         to == TaskState::NAVIGATING_TO_UNLOAD) ||
        (from == TaskState::NAVIGATING_TO_UNLOAD &&
         to == TaskState::UNLOAD_READY) ||
        (from == TaskState::UNLOAD_READY &&
         to == TaskState::OPENING_GRAB) ||
        (from == TaskState::OPENING_GRAB &&
         to == TaskState::UNLOAD_VERIFY) ||
        (from == TaskState::UNLOAD_VERIFY &&
         to == TaskState::RETURNING_TO_SAFE_WAIT) ||
        (from == TaskState::RETURNING_TO_SAFE_WAIT &&
         to == TaskState::DONE);
}

inline bool canTransitionTaskState(
    TaskState from, TaskState to) noexcept {
    if (from == to) {
        return true;
    }
    if (to == TaskState::FAULT ||
        to == TaskState::MANUAL_OVERRIDE) {
        return from != TaskState::DONE;
    }
    if ((from == TaskState::DONE ||
         from == TaskState::FAULT ||
         from == TaskState::MANUAL_OVERRIDE) &&
        to == TaskState::IDLE) {
        return true;
    }
    return isSequentialTaskTransition(from, to);
}

}  // namespace chili_crane_core
