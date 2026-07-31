# ADR 0002: Separate Mapping and Localization Modes

- Status: `ACCEPTED`

## Decision

Mapping and production localization are explicit profiles.

- Mapping may create a new versioned static-track snapshot when quality gates
  allow.
- Localization loads a frozen snapshot and forbids automatic map mutation by
  default.

Pit interiors, chili, the grab, people, temporary equipment, and movable carts
are excluded from the localization registration product.

## Consequences

- map provenance and rollback are straightforward;
- changing chili cannot silently contaminate localization;
- maintenance remapping is an intentional operation;
- online long-term map logic from NDT-SLAM-Warehouse is not copied into Phase 0.
