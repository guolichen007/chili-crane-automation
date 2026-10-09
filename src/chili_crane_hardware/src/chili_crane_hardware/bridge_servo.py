"""Vendor-neutral X adapter protocol; no real servo protocol is assumed."""
from typing import Protocol


class BridgeServoAdapter(Protocol):
    def move_to(self, position_m: float, command_id: str) -> None: ...
    def stop(self) -> None: ...
    def ready(self) -> bool: ...
    def at_target(self) -> bool: ...
    def fault(self) -> bool | None: ...
    def state(self) -> dict: ...


class UnconfiguredBridgeServo:
    def move_to(self, position_m, command_id):
        raise RuntimeError("X_SERVO_PROTOCOL_NOT_CONFIGURED")

    def stop(self):
        return {"validity": "NOT_CONFIGURED", "physical_stop_verified": False}

    def ready(self):
        return False

    def at_target(self):
        return False

    def fault(self):
        return None

    def state(self):
        return {"validity": "NOT_CONFIGURED", "reason": "vendor_protocol_unknown"}
