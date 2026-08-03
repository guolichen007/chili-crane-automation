# Windows Static Validation — Bootstrap Hardening

```text
BRANCH: codex/bootstrap-hardening-v1
INPUT_SHA: 01ce545bd064d7f80470f76d3e2b0658530c3ae6
OUTPUT_SHA: PENDING_UNTIL_COMMIT
WINDOWS_STATIC_STATUS: PASS
GITHUB_ACTIONS_STATUS: NOT_RUN
UBUNTU_BUILD_STATUS: NOT_RUN
ROSLAUNCH_STATUS: NOT_RUN
BAG_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN
```

Windows environment:

```text
OS: Windows
PYTHON_VERSION: 3.12.4
GIT_VERSION: 2.46.2.windows.1
PYYAML_VERSION: 6.0.2
JSONSCHEMA_VERSION: 4.23.0
```

Executed Windows checks:

```text
git diff --check
python -m pip install -r requirements-dev.txt
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
python -m compileall tools tests_static src
Git Bash: bash -n scripts/validation/ubuntu20_phase0_validate.sh
```

Observed results:

- repository contract checker: passed;
- static unit tests: 37 passed;
- Python compileall: passed;
- Git Bash syntax parse: passed;
- Git diff whitespace check: passed;
- repository Git configuration: `core.autocrlf=false`, `core.eol=lf`;
- local-only context: confirmed ignored by Git.

This evidence record binds to the exact input SHA. The final output SHA must be
reported after commit creation and must not be invented inside that commit.
Windows static evidence is not an Ubuntu build, roslaunch, bag, hardware, or
field result.
