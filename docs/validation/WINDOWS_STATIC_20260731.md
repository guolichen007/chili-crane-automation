# Windows Static Validation — 2026-07-31

```text
BRANCH: codex/bootstrap-architecture-v1
INPUT_SHA: EMPTY_REMOTE
EVIDENCE_SHA: NOT_AVAILABLE_UNBORN_BRANCH
WINDOWS_STATIC_STATUS: PASS
UBUNTU_BUILD_STATUS: NOT_RUN
ROSLAUNCH_STATUS: NOT_RUN
BAG_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN
REFERENCE_NDT_SHA: 42f921e91574ff0f29b91fc08cbade67c976aa36
```

Commands run from the repository root:

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
```

Observed result before the initial commit:

- repository contract checker: passed;
- static test suite: 16 tests passed;
- Git whitespace check: no error output.

The checks were repeated after the raw-grab-I/O separation and ROS private
parameter-layout correction. They do not compile ROS packages and do not
validate PCL, NDT, TF timing, roslaunch, bag replay, device protocols, control
outputs, or field behavior.

This record intentionally refers to an unborn local branch because the remote
repository was empty when the checks ran. After a commit exists, rerun the
checks and add a SHA-bound validation record; do not edit this historical
result to imply later evidence.
