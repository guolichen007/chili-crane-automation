#!/usr/bin/env python3
"""Validate repository contracts without requiring ROS or Ubuntu."""

from __future__ import annotations

import json
import py_compile
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List

import yaml


TEXT_SUFFIXES = {
    ".c",
    ".cc",
    ".cmake",
    ".cpp",
    ".h",
    ".hpp",
    ".json",
    ".launch",
    ".md",
    ".msg",
    ".py",
    ".sh",
    ".txt",
    ".xml",
    ".yaml",
    ".yml",
}
TEXT_NAMES = {
    ".editorconfig",
    ".gitattributes",
    ".gitignore",
    "CMakeLists.txt",
    "LICENSE",
}
EXCLUDED_DIRECTORIES = {
    ".git",
    ".idea",
    ".vscode",
    "__pycache__",
    "build",
    "devel",
    "install",
    "logs",
}
EXPECTED_PACKAGES = {
    "chili_crane_bringup",
    "chili_crane_control",
    "chili_crane_core",
    "chili_crane_msgs",
    "chili_crane_perception",
    "chili_crane_slam",
}
REQUIRED_FILES = {
    ".editorconfig",
    ".gitattributes",
    "AGENTS.md",
    "README.md",
    "config/profiles/localization.yaml",
    "config/profiles/mapping.yaml",
    "config/sensors/dual_lidar.template.yaml",
    "config/semantic/track_01.template.yaml",
    "docs/ARCHITECTURE_REVIEW.md",
    "docs/CODEX_START_PROMPT.md",
    "docs/NDT_REUSE_PLAN.md",
    "docs/PROJECT_CONTEXT.md",
    "docs/SYSTEM_ARCHITECTURE_AND_ROADMAP.md",
    "docs/api/map_manifest.schema.json",
    "docs/api/semantic_map.schema.json",
    "docs/validation/UBUNTU_VALIDATION_RUNBOOK.md",
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _relative(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def iter_first_party_files(root: Path) -> Iterable[Path]:
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        if any(part in EXCLUDED_DIRECTORIES for part in path.parts):
            continue
        yield path


def iter_text_files(root: Path) -> Iterable[Path]:
    for path in iter_first_party_files(root):
        if path.name in TEXT_NAMES or path.suffix.lower() in TEXT_SUFFIXES:
            yield path


def check_required_files(root: Path, errors: List[str]) -> None:
    for relative in sorted(REQUIRED_FILES):
        if not (root / relative).is_file():
            errors.append(f"missing required file: {relative}")


def check_text_contract(root: Path, errors: List[str]) -> None:
    for path in iter_text_files(root):
        relative = _relative(path, root)
        data = path.read_bytes()
        if data.startswith(b"\xef\xbb\xbf"):
            errors.append(f"UTF-8 BOM is forbidden: {relative}")
        if b"\r" in data:
            errors.append(f"non-LF line ending found: {relative}")
        try:
            data.decode("utf-8")
        except UnicodeDecodeError as exc:
            errors.append(f"not valid UTF-8: {relative}: {exc}")
            continue
        if data and not data.endswith(b"\n"):
            errors.append(f"missing final newline: {relative}")


def check_no_runtime_windows_paths(root: Path, errors: List[str]) -> None:
    drive_path = re.compile(r"[A-Za-z]:[\\/]")
    for base in ("src", "config"):
        for path in iter_text_files(root / base):
            text = path.read_text(encoding="utf-8")
            if drive_path.search(text):
                errors.append(
                    "Windows drive path in runtime/config file: "
                    + _relative(path, root)
                )


def check_packages(root: Path, errors: List[str]) -> None:
    package_names = {
        path.parent.name for path in (root / "src").glob("*/package.xml")
    }
    if package_names != EXPECTED_PACKAGES:
        errors.append(
            "package set mismatch: expected "
            f"{sorted(EXPECTED_PACKAGES)}, got {sorted(package_names)}"
        )

    for path in (root / "src").glob("*/package.xml"):
        try:
            tree = ET.parse(path)
        except ET.ParseError as exc:
            errors.append(f"invalid package XML: {_relative(path, root)}: {exc}")
            continue
        declared = tree.getroot().findtext("name")
        if declared != path.parent.name:
            errors.append(
                f"package name/path mismatch in {_relative(path, root)}"
            )


def check_xml_and_launch(root: Path, errors: List[str]) -> None:
    paths = list((root / "src").rglob("*.launch"))
    for path in paths:
        try:
            root_element = ET.parse(path).getroot()
        except ET.ParseError as exc:
            errors.append(f"invalid launch XML: {_relative(path, root)}: {exc}")
            continue
        if root_element.tag != "launch":
            errors.append(f"launch root is not <launch>: {_relative(path, root)}")


def load_yaml(path: Path, root: Path, errors: List[str]):
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        errors.append(f"invalid YAML: {_relative(path, root)}: {exc}")
        return None


def check_yaml(root: Path, errors: List[str]) -> None:
    for path in (root / "config").rglob("*.yaml"):
        load_yaml(path, root, errors)


def check_templates_fail_safe(root: Path, errors: List[str]) -> None:
    for path in (root / "config").rglob("*.template.yaml"):
        data = load_yaml(path, root, errors)
        if not isinstance(data, dict):
            continue
        if data.get("config_state") != "NOT_CONFIGURED":
            errors.append(
                f"template must be NOT_CONFIGURED: {_relative(path, root)}"
            )


def check_profiles(root: Path, errors: List[str]) -> None:
    profile_dir = root / "config" / "profiles"
    mapping = load_yaml(profile_dir / "mapping.yaml", root, errors) or {}
    localization = load_yaml(
        profile_dir / "localization.yaml", root, errors
    ) or {}
    mock = load_yaml(profile_dir / "mock.yaml", root, errors) or {}

    if mapping.get("runtime", {}).get("mode") != "mapping":
        errors.append("mapping profile runtime.mode must be mapping")
    if not mapping.get("map_lifecycle", {}).get("map_mutation_allowed"):
        errors.append("mapping profile must explicitly allow map mutation")
    if localization.get("runtime", {}).get("mode") != "localization":
        errors.append("localization profile runtime.mode must be localization")
    if localization.get("map_lifecycle", {}).get("map_mutation_allowed"):
        errors.append("localization profile must forbid map mutation")
    if not localization.get("map_lifecycle", {}).get("require_frozen_map"):
        errors.append("localization profile must require a frozen map")

    for path in profile_dir.glob("*.yaml"):
        data = load_yaml(path, root, errors) or {}
        if data.get("runtime", {}).get("hardware_adapter") != "mock":
            errors.append(
                "real hardware adapter cannot be a Phase 0 default: "
                + _relative(path, root)
            )
        if data.get("safety", {}).get("default_allow_auto_task") is not False:
            errors.append(
                "profile must fail closed for auto task: "
                + _relative(path, root)
            )
    if (
        mock.get("mock_hardware_adapter", {}).get(
            "configured_valid_mock"
        )
        is not False
    ):
        errors.append("mock profile must default to an unconfigured state")

    for path in profile_dir.glob("*.yaml"):
        data = load_yaml(path, root, errors) or {}
        mock_adapter = data.get("mock_hardware_adapter", {})
        if mock_adapter.get("configured_valid_mock") is not False:
            errors.append(
                "mock adapter private params must fail closed: "
                + _relative(path, root)
            )


def check_messages(root: Path, errors: List[str]) -> None:
    message_dir = root / "src" / "chili_crane_msgs" / "msg"
    for path in message_dir.glob("*.msg"):
        text = path.read_text(encoding="utf-8")
        if "uint8 validity" not in text:
            errors.append(f"message lacks validity: {_relative(path, root)}")
        if "string reason" not in text:
            errors.append(f"message lacks reason: {_relative(path, root)}")

    safety = (message_dir / "SafetyPermit.msg").read_text(encoding="utf-8")
    for permission in (
        "allow_x_move",
        "allow_y_move",
        "allow_lower",
        "allow_raise",
        "allow_grab_open",
        "allow_grab_close",
        "allow_unload",
        "allow_auto_task",
    ):
        if f"bool {permission}" not in safety:
            errors.append(f"SafetyPermit missing permission: {permission}")


def check_json_schemas(root: Path, errors: List[str]) -> None:
    for path in (root / "docs" / "api").glob("*.schema.json"):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            errors.append(
                f"invalid JSON schema: {_relative(path, root)}: {exc}"
            )
            continue
        if data.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
            errors.append(
                f"unexpected JSON schema dialect: {_relative(path, root)}"
            )


def check_python_syntax(root: Path, errors: List[str]) -> None:
    for path in (root / "src").rglob("*.py"):
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as exc:
            errors.append(f"Python syntax error: {_relative(path, root)}: {exc}")


def check_validation_claims(root: Path, errors: List[str]) -> None:
    validation_root = root / "docs" / "validation"
    pass_claim = re.compile(
        r"^(UBUNTU_BUILD_STATUS|ROSLAUNCH_STATUS|BAG_STATUS|FIELD_STATUS):"
        r"\s*PASS\s*$",
        re.MULTILINE,
    )
    for path in validation_root.glob("*.md"):
        text = path.read_text(encoding="utf-8")
        if pass_claim.search(text) and not re.search(
            r"^EVIDENCE_SHA:\s*[0-9a-f]{40}\s*$", text, re.MULTILINE
        ):
            errors.append(
                "validation PASS claim lacks exact EVIDENCE_SHA: "
                + _relative(path, root)
            )


def collect_errors(root: Path | None = None) -> List[str]:
    root = root or repo_root()
    errors: List[str] = []
    check_required_files(root, errors)
    check_text_contract(root, errors)
    check_no_runtime_windows_paths(root, errors)
    check_packages(root, errors)
    check_xml_and_launch(root, errors)
    check_yaml(root, errors)
    check_templates_fail_safe(root, errors)
    check_profiles(root, errors)
    check_messages(root, errors)
    check_json_schemas(root, errors)
    check_python_syntax(root, errors)
    check_validation_claims(root, errors)
    return errors


def main() -> int:
    errors = collect_errors()
    if errors:
        print(f"Repository contract check failed ({len(errors)} error(s)):")
        for error in errors:
            print(f"  - {error}")
        return 1
    print("Repository contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
