"""Source provenance is part of authorization, not just a display label."""
from enum import IntEnum
from dataclasses import dataclass


class SourceType(IntEnum):
    UNKNOWN = 0
    PHYSICAL = 1
    SIMULATED = 2
    REPLAY = 3
    SYNTHETIC = 4


@dataclass(frozen=True)
class RuntimePolicy:
    mode: str = "production"
    physical_output_enabled: bool = False
    automatic_control_enabled: bool = False

    def sources_allowed(self, sources):
        if not sources or any(type(x) is not SourceType or x == SourceType.UNKNOWN for x in sources):
            return False
        if self.mode == "production":
            return all(x == SourceType.PHYSICAL for x in sources)
        return self.mode in {"algorithm_dev", "shadow_control", "synthetic_test", "replay"}

    def physical_permission(self, sources):
        return (self.mode == "production" and self.physical_output_enabled is True
                and self.automatic_control_enabled is True and self.sources_allowed(sources))
