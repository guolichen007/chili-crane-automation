#pragma once

#include <cstdint>

namespace chili_crane_core {

enum class TaskState : std::uint8_t {
    IDLE = 0,
    TASK_ACCEPTED,
    LOCALIZATION_READY,
    NAVIGATING_TO_PIT,
    PIT_POSITIONED,
    SCANNING_PIT,
    GRASP_TARGET_READY,
    POSITIONING_TROLLEY,
    WAITING_STABLE,
    LOWERING,
    CONTACT_DETECTED,
    CLOSING_GRAB,
    LOAD_VERIFY,
    RAISING_TO_SAFE_HEIGHT,
    NAVIGATING_TO_UNLOAD,
    UNLOAD_READY,
    OPENING_GRAB,
    UNLOAD_VERIFY,
    RETURNING_TO_SAFE_WAIT,
    DONE,
    FAULT,
    MANUAL_OVERRIDE,
};

inline bool isTerminalTaskState(TaskState state) noexcept {
    return state == TaskState::DONE || state == TaskState::FAULT;
}

}  // namespace chili_crane_core
