# Configuration

All files in `sensors/`, `hardware/`, `perception/`, and `semantic/` are
templates. `NOT_CONFIGURED` and `null` are deliberate and must not be replaced
with guessed production values.

Runtime profiles define lifecycle behavior:

- `mapping.yaml`: map mutation contract is enabled, implementation not ready;
- `localization.yaml`: frozen map required and mutation disabled;
- `replay.yaml`: simulated-time/static replay contract;
- `mock.yaml`: fail-safe interface development.

Site-specific calibrated files should be versioned only after field review.
