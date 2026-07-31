# Desktop Codex Start Prompt

Use this prompt when opening a fresh Desktop Codex task for this repository.
Replace local paths if the workspaces move.

```text
Open the local chili-crane-automation repository and treat its AGENTS.md as
authoritative repository instructions. You may read the local reference
workspace C:/Users/13576/Desktop/NDT-slam-ws, but it is read-only upstream
evidence rather than the chili project's source of truth.

Before changing code, read completely:
- AGENTS.md
- docs/PROJECT_CONTEXT.md
- docs/NDT_REUSE_PLAN.md
- docs/SYSTEM_ARCHITECTURE_AND_ROADMAP.md
- docs/ARCHITECTURE_REVIEW.md
- docs/api/TOPIC_AND_TF_CONTRACT.md
- docs/api/STATE_AND_SAFETY_CONTRACT.md
- the relevant accepted/proposed ADRs under docs/decisions/

For any NDT adaptation, also inspect the exact upstream implementation at the
SHA recorded in docs/NDT_REUSE_PLAN.md. Do not rely only on upstream prose.

Start by reporting:
1. the requested outcome and current repository state;
2. assumptions and unresolved hardware/interface questions;
3. the package boundaries affected;
4. the exact NDT files or ideas proposed for reuse;
5. Windows-static checks that can run here;
6. Ubuntu, ROS bag, hardware, and field checks that remain NOT_RUN;
7. the intended branch/commit scope.

Then implement the smallest complete vertical slice that preserves the
established contracts.

Hard constraints:
- all first-party text is UTF-8 without BOM, LF-only, and newline-terminated;
- runtime/config paths use Linux conventions and never a Windows drive;
- Windows static checks cannot be described as Ubuntu ROS validation;
- never guess an unconfirmed device protocol, register, topic, extrinsic,
  calibration, pit dimension, grab dimension, safety threshold, or speed;
- hardware protocols stay behind adapters;
- missing/stale/conflicting/unconfigured evidence fails closed;
- mapping and frozen-map localization remain separate modes;
- changing chili surfaces are excluded from localization-map evidence;
- raw GrabIoState and fused perception GrabState remain separate;
- do not copy the whole warehouse application or its cargo safety vocabulary;
- every NDT-derived change records upstream SHA, source paths and adaptations.

Before each commit run:
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"

At handoff report:
INPUT_SHA / OUTPUT_SHA / BRANCH / PUSHED
WINDOWS_STATIC_STATUS
UBUNTU_BUILD_STATUS / ROSLAUNCH_STATUS / BAG_STATUS / FIELD_STATUS
open decisions, new configuration required, and the next Ubuntu commands.
```

## Canonical four-file handoff

The compact long-term handoff consists of:

1. `PROJECT_CONTEXT.md` — requirements, equipment baseline and unknowns;
2. `NDT_REUSE_PLAN.md` — inspected upstream evidence and adaptation boundaries;
3. `SYSTEM_ARCHITECTURE_AND_ROADMAP.md` — stable system structure and phases;
4. this file — repeatable execution prompt for a fresh Desktop Codex task.

`AGENTS.md` is the automatically discovered repository policy above those four
handoff documents.
