"""Repeated grab-cycle skeleton, independent of control ownership and hardware."""
from enum import IntEnum
from .contracts import SafetyEvent, Severity, EventDomain


class CycleState(IntEnum):
    SCAN = 0
    PLAN_PICK = 1
    POSITION_Y = 2
    WAIT_STABLE = 3
    ENSURE_GRAB_OPEN = 4
    LOWER = 5
    CLOSE = 6
    VERIFY_LOAD = 7
    RAISE = 8
    MOVE_TO_UNLOAD = 9
    VERIFY_CART = 10
    POSITION_UNLOAD = 11
    OPEN_GRAB = 12
    VERIFY_EMPTY = 13
    COMPLETE = 14
    ABORTED = 15


class TaskCycles:
    def __init__(self, run_id, task_id):
        if not run_id or not task_id:
            raise ValueError("run and task identity required")
        self.run_id, self.task_id = run_id, task_id
        self.cycle_index = 1
        self.state = CycleState.SCAN
        self.last_scan_counter = 0
        self.needs_new_scan = True
        self.task_state = "ACTIVE"
        self.events = []

    @property
    def cycle_id(self):
        return self.task_id + ":" + str(self.cycle_index)

    def accept_scan(self, source_counter, fresh=False):
        if (self.task_state != "ACTIVE" or self.state != CycleState.SCAN or fresh is not True or type(source_counter) is not int
                or source_counter <= self.last_scan_counter):
            return False
        self.last_scan_counter = source_counter
        self.needs_new_scan = False
        self.state = CycleState.PLAN_PICK
        return True

    def advance(self, evidence_verified=False):
        if self.task_state != "ACTIVE" or evidence_verified is not True or self.needs_new_scan or self.state not in (
                CycleState.PLAN_PICK, CycleState.POSITION_Y, CycleState.WAIT_STABLE,
                CycleState.ENSURE_GRAB_OPEN, CycleState.LOWER, CycleState.CLOSE,
                CycleState.VERIFY_LOAD, CycleState.RAISE, CycleState.MOVE_TO_UNLOAD,
                CycleState.VERIFY_CART, CycleState.POSITION_UNLOAD, CycleState.OPEN_GRAB,
                CycleState.VERIFY_EMPTY):
            return False
        self.state = CycleState(self.state + 1)
        return True

    def next_cycle(self, continue_task=False):
        if self.task_state != "ACTIVE" or self.state != CycleState.COMPLETE:
            return False
        if continue_task is True:
            self.cycle_index += 1
            self.state = CycleState.SCAN
            self.needs_new_scan = True
        else:
            self.task_state = "RETURN_SAFE"
        return True

    def returned_safe(self, verified=False):
        if self.task_state == "RETURN_SAFE" and verified is True:
            self.task_state = "DONE"
            return True
        return False

    def fail_cycle(self, code, retry_authorized=False):
        if self.task_state != "ACTIVE":
            return False
        self.events.append(SafetyEvent(Severity.WARNING, EventDomain.CYCLE_FAILURE, code,
                                     "task_cycles", "cycle_failed", recoverable=True,
                                     run_id=self.run_id, task_id=self.task_id, cycle_id=self.cycle_id))
        self.state = CycleState.ABORTED
        self.needs_new_scan = True
        if retry_authorized is True:
            self.cycle_index += 1
            self.state = CycleState.SCAN
        return True

    def remote_abort(self):
        self.state = CycleState.ABORTED
        self.needs_new_scan = True
        self.task_state = "ABORTED"
