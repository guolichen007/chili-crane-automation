#!/usr/bin/env python3
"""Validate repository contracts without requiring ROS or Ubuntu."""

import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, List, Optional

import jsonschema
import yaml


TEXT_SUFFIXES = {
    ".c",
    ".cc",
    ".cmake",
    ".cpp",
    ".h",
    ".hpp",
    ".json",
    ".in",
    ".launch",
    ".md",
    ".msg",
    ".py",
    ".rviz",
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
    ".github/workflows/static-contracts.yml",
    "config/contracts/task_states.yaml",
    "config/profiles/localization.yaml",
    "config/profiles/mapping.yaml",
    "config/sensors/dual_lidar.template.yaml",
    "config/sensors/cameras.template.yaml",
    "config/semantic/track_01.template.yaml",
    "docs/ARCHITECTURE_REVIEW.md",
    "docs/CODEX_START_PROMPT.md",
    "docs/NDT_REUSE_PLAN.md",
    "docs/PROJECT_CONTEXT.md",
    "docs/SYSTEM_ARCHITECTURE_AND_ROADMAP.md",
    "docs/api/map_manifest.schema.json",
    "docs/api/semantic_map.schema.json",
    "docs/validation/UBUNTU_VALIDATION_RUNBOOK.md",
    "docs/validation/WINDOWS_STATIC_01ce545.md",
    "scripts/validation/ubuntu20_phase0_validate.sh",
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
        if data.endswith(b"\n\n"):
            errors.append(f"more than one trailing newline: {relative}")


def check_no_runtime_windows_paths(root: Path, errors: List[str]) -> None:
    drive_path = re.compile(r"[A-Za-z]:[\\/]")
    hardcoded_home = re.compile(r"/home/[A-Za-z0-9_.-]+/")
    workspace_backslash = re.compile(r"\\workspace\\", re.IGNORECASE)
    for base in ("src", "config", "scripts"):
        if not (root / base).exists():
            continue
        for path in iter_text_files(root / base):
            text = path.read_text(encoding="utf-8")
            if drive_path.search(text):
                errors.append(
                    "Windows drive path in runtime/config file: "
                    + _relative(path, root)
                )
            if hardcoded_home.search(text):
                errors.append(
                    "hardcoded Linux user home in runtime/config file: "
                    + _relative(path, root)
                )
            if workspace_backslash.search(text):
                errors.append(
                    "Windows-style workspace path in runtime/config file: "
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


def check_launch_config_root(root: Path, errors: List[str]) -> None:
    launch_dir = root / "src" / "chili_crane_bringup" / "launch"
    public_entries = {
        "bag_replay.launch",
        "localization.launch",
        "mapping.launch",
        "mock_system.launch",
    }
    for name in sorted(public_entries):
        path = launch_dir / name
        if not path.is_file():
            errors.append(f"missing public launch entry: {_relative(path, root)}")
            continue
        launch = ET.parse(path).getroot()
        args = {
            element.attrib.get("name"): element.attrib
            for element in launch.findall("arg")
        }
        if "config_root" not in args:
            errors.append(f"launch entry lacks config_root: {_relative(path, root)}")
        elif "default" not in args["config_root"]:
            errors.append(
                f"launch config_root is not overridable: {_relative(path, root)}"
            )


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
    allowed_modes = {"mapping", "localization", "replay"}

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
    if mock.get("runtime", {}).get("mode") != "localization":
        errors.append("mock profile must use localization runtime mode")

    for path in profile_dir.glob("*.yaml"):
        data = load_yaml(path, root, errors) or {}
        if data.get("runtime", {}).get("mode") not in allowed_modes:
            errors.append(
                "profile has unsupported runtime mode: " + _relative(path, root)
            )
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
        "allow_x_positive",
        "allow_x_negative",
        "allow_y_positive",
        "allow_y_negative",
        "allow_lower",
        "allow_raise",
        "allow_grab_open",
        "allow_grab_close",
        "allow_unload",
        "allow_auto_task",
    ):
        if f"bool {permission}" not in safety:
            errors.append(f"SafetyPermit missing permission: {permission}")

    authorized = (message_dir / "AuthorizedCommand.msg").read_text(
        encoding="utf-8"
    )
    for field in (
        "string command_id",
        "string intent_id",
        "string permit_id",
        "uint64 permit_generation",
        "time issued_stamp",
        "time expire_stamp",
    ):
        if field not in authorized:
            errors.append(f"AuthorizedCommand missing field: {field}")


def check_message_generation(root: Path, errors: List[str]) -> None:
    message_dir = root / "src" / "chili_crane_msgs" / "msg"
    actual = {path.name for path in message_dir.glob("*.msg")}
    cmake = (message_dir.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    declared = set(
        re.findall(
            r"^\s+([A-Za-z0-9_]+\.msg)\s*$",
            cmake,
            re.MULTILINE,
        )
    )
    if actual != declared:
        errors.append(
            "message CMake list mismatch: expected {}, got {}".format(
                sorted(actual), sorted(declared)
            )
        )


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
            continue
        try:
            jsonschema.Draft202012Validator.check_schema(data)
        except jsonschema.SchemaError as exc:
            errors.append(f"invalid JSON schema: {_relative(path, root)}: {exc}")


def check_python_syntax(root: Path, errors: List[str]) -> None:
    for path in iter_first_party_files(root):
        if path.suffix.lower() != ".py":
            continue
        try:
            source = path.read_text(encoding="utf-8")
            compile(source, str(path), "exec")
        except SyntaxError as exc:
            errors.append(f"Python syntax error: {_relative(path, root)}: {exc}")


def check_python38_compatibility(root: Path, errors: List[str]) -> None:
    forbidden_patterns = (
        (
            re.compile(r"\b[A-Za-z_][A-Za-z0-9_.]*\s*\|\s*None\b"),
            "PEP 604 union annotation",
        ),
        (
            re.compile(r"\bNone\s*\|\s*[A-Za-z_][A-Za-z0-9_.]*\b"),
            "PEP 604 union annotation",
        ),
        (
            re.compile(r"\b(?:list|dict|set|tuple|frozenset|type)\s*\["),
            "Python 3.9 built-in generic annotation",
        ),
        (
            re.compile(r"^\s*match\s+.+:\s*$", re.MULTILINE),
            "Python 3.10 match statement",
        ),
        (
            re.compile(r"^\s*(?:from\s+tomllib\s+import|import\s+tomllib)\b", re.MULTILINE),
            "Python 3.11 tomllib module",
        ),
        (
            re.compile(r"^\s*(?:from\s+(?:zoneinfo|graphlib)\s+import|import\s+(?:zoneinfo|graphlib))\b", re.MULTILINE),
            "Python 3.9 standard-library module",
        ),
        (
            re.compile(r"\.(?:removeprefix|removesuffix)\("),
            "Python 3.9 string method",
        ),
        (
            re.compile(r"\.(?:is_relative_to|hardlink_to)\("),
            "post-Python-3.8 pathlib method",
        ),
    )
    for path in iter_first_party_files(root):
        if path.suffix.lower() != ".py":
            continue
        text = path.read_text(encoding="utf-8")
        for pattern, description in forbidden_patterns:
            if pattern.search(text):
                errors.append(
                    f"{description} is forbidden for Python 3.8: "
                    + _relative(path, root)
                )


def check_gitattributes(root: Path, errors: List[str]) -> None:
    text = (root / ".gitattributes").read_text(encoding="utf-8")
    if "* text=auto eol=lf" not in text:
        errors.append(".gitattributes must enforce LF for first-party text")
    if re.search(r"eol\s*=\s*crlf", text, re.IGNORECASE):
        errors.append(".gitattributes must not contain CRLF exceptions")


def check_camera_template(root: Path, errors: List[str]) -> None:
    path = root / "config" / "sensors" / "cameras.template.yaml"
    data = load_yaml(path, root, errors) or {}
    cameras = data.get("cameras", [])
    by_id = {
        camera.get("sensor_id"): camera
        for camera in cameras
        if isinstance(camera, dict)
    }
    if set(by_id) != {"camera_left", "camera_right"}:
        errors.append("camera template must contain camera_left and camera_right")
    for sensor_id, camera in by_id.items():
        if camera.get("enabled") is not False:
            errors.append(f"camera must default disabled: {sensor_id}")
        for key in ("topic", "frame_id", "intrinsic_version", "extrinsic_version"):
            if camera.get(key) != "NOT_CONFIGURED":
                errors.append(f"camera {key} must be NOT_CONFIGURED: {sensor_id}")


def _parse_cpp_task_states(text: str):
    match = re.search(
        r"enum class TaskState\s*:\s*std::uint8_t\s*\{(?P<body>.*?)\};",
        text,
        re.DOTALL,
    )
    if match is None:
        return []
    states = []
    next_value = 0
    for raw_item in match.group("body").split(","):
        item = raw_item.strip()
        if not item:
            continue
        parts = [part.strip() for part in item.split("=", 1)]
        if len(parts) == 2:
            next_value = int(parts[1], 0)
        states.append((parts[0], next_value))
        next_value += 1
    return states


def check_task_state_consistency(root: Path, errors: List[str]) -> None:
    contract_path = root / "config" / "contracts" / "task_states.yaml"
    contract = load_yaml(contract_path, root, errors) or {}
    expected = [
        (item.get("name"), item.get("value"))
        for item in contract.get("states", [])
    ]
    header = (
        root
        / "src"
        / "chili_crane_core"
        / "include"
        / "chili_crane_core"
        / "task_state.hpp"
    ).read_text(encoding="utf-8")
    message = (
        root / "src" / "chili_crane_msgs" / "msg" / "TaskStatus.msg"
    ).read_text(encoding="utf-8")
    message_values = {
        name: int(value)
        for name, value in re.findall(
            r"^uint8 ([A-Z][A-Z0-9_]*)=(\d+)$", message, re.MULTILINE
        )
    }
    message_states = [
        (name, message_values[name])
        for name, _ in expected
        if name in message_values
    ]
    if expected != _parse_cpp_task_states(header):
        errors.append("TaskState C++ enum differs from task_states.yaml")
    if expected != message_states:
        errors.append("TaskStatus message differs from task_states.yaml")


def check_hardware_authorization_boundary(root: Path, errors: List[str]) -> None:
    adapter = (
        root
        / "src"
        / "chili_crane_control"
        / "scripts"
        / "mock_hardware_adapter.py"
    ).read_text(encoding="utf-8")
    if "control/authorized_command" not in adapter:
        errors.append("hardware adapter must consume authorized_command")
    for forbidden in ("control/intent", "control/requested_intent", "ControlIntent"):
        if forbidden in adapter:
            errors.append(
                "hardware adapter must not consume raw intent: " + forbidden
            )


def check_hardware_templates(root: Path, errors: List[str]) -> None:
    for path in (root / "config" / "hardware").glob("*.yaml"):
        data = load_yaml(path, root, errors) or {}
        adapter = data.get("adapter")
        if adapter not in (None, "mock", "NOT_CONFIGURED"):
            errors.append("real hardware adapter cannot be default: " + _relative(path, root))
        protocol = data.get("protocol")
        if protocol not in (None, "NOT_CONFIGURED"):
            errors.append("real hardware protocol cannot be default: " + _relative(path, root))


def check_cpp_compile_contracts(root: Path, errors: List[str]) -> None:
    packages = {
        "chili_crane_core": "test/core_contract_compile_test.cpp",
        "chili_crane_slam": "test/slam_contract_compile_test.cpp",
        "chili_crane_perception": "test/perception_contract_compile_test.cpp",
    }
    for package, test_source in packages.items():
        package_dir = root / "src" / package
        cmake = (package_dir / "CMakeLists.txt").read_text(encoding="utf-8")
        if "set(CMAKE_CXX_STANDARD 17)" not in cmake:
            errors.append(f"C++17 is not enforced: {package}")
        if "catkin_add_gtest" not in cmake:
            errors.append(f"public-header gtest target missing: {package}")
        if not (package_dir / test_source).is_file():
            errors.append(f"public-header compile source missing: {package}")


def check_no_phase1_algorithm_dependencies(root: Path, errors: List[str]) -> None:
    forbidden = re.compile(r"\b(?:PCL|Sophus|ndt_omp)\b", re.IGNORECASE)
    for path in (root / "src").glob("*/CMakeLists.txt"):
        if forbidden.search(path.read_text(encoding="utf-8")):
            errors.append(
                "Phase 0 hardening added a forbidden algorithm dependency: "
                + _relative(path, root)
            )
    for path in (root / "src").glob("*/package.xml"):
        if forbidden.search(path.read_text(encoding="utf-8")):
            errors.append(
                "Phase 0 hardening added a forbidden algorithm dependency: "
                + _relative(path, root)
            )


def check_static_ci(root: Path, errors: List[str]) -> None:
    path = root / ".github" / "workflows" / "static-contracts.yml"
    text = path.read_text(encoding="utf-8")
    for required in (
        'python-version: "3.8"',
        "python tools/check_repo_contracts.py",
        'python -m unittest discover -s tests_static -p "test_*.py"',
    ):
        if required not in text:
            errors.append("static CI missing command: " + required)
    for misleading_name in (
        "ROS Build",
        "Ubuntu Validation",
        "Field Validation",
    ):
        if misleading_name in text:
            errors.append("static CI has misleading name: " + misleading_name)


def check_ubuntu_validation_script(root: Path, errors: List[str]) -> None:
    path = root / "scripts" / "validation" / "ubuntu20_phase0_validate.sh"
    text = path.read_text(encoding="utf-8")
    if not text.startswith("#!/usr/bin/env bash\nset -euo pipefail\n"):
        errors.append("Ubuntu validation script must start fail-fast")
    for argument in ("--workspace", "--evidence-dir", "--expected-sha"):
        if argument not in text:
            errors.append("Ubuntu validation script missing argument: " + argument)
    for required in (
        "Ubuntu 20.04.6",
        "ROS_DISTRO",
        'python_version" != "3.8"',
        'gcc_version%%.*}" != "9"',
        "catkin build",
        "catkin test",
        "control_board_state",
    ):
        if required not in text:
            errors.append("Ubuntu validation script missing check: " + required)


def check_runtime_baseline(root: Path, errors: List[str]) -> None:
    adr = (
        root / "docs" / "decisions" / "0001-runtime_baseline.md"
    ).read_text(encoding="utf-8")
    for required in (
        "Status: `ACCEPTED`",
        "Ubuntu 20.04.6 LTS",
        "ROS Noetic",
        "catkin_tools",
        "Python: 3.8",
        "C++: C++17",
        "GCC 9.x",
    ):
        if required not in adr:
            errors.append("runtime ADR missing accepted baseline: " + required)


def check_validation_claims(root: Path, errors: List[str]) -> None:
    validation_root = root / "docs" / "validation"
    pass_claim = re.compile(r"^[A-Z0-9_]+_STATUS:\s*PASS\s*$", re.MULTILINE)
    exact_sha = re.compile(
        r"^(?:EVIDENCE_SHA|INPUT_SHA|OUTPUT_SHA):\s*[0-9a-f]{40}\s*$",
        re.MULTILINE,
    )
    for path in validation_root.glob("*.md"):
        if path.name == "WINDOWS_STATIC_20260731.md":
            continue
        text = path.read_text(encoding="utf-8")
        if pass_claim.search(text) and not exact_sha.search(text):
            errors.append(
                "validation PASS claim lacks exact EVIDENCE_SHA: "
                + _relative(path, root)
            )


def collect_errors(root: Optional[Path] = None) -> List[str]:
    root = root or repo_root()
    errors: List[str] = []
    check_required_files(root, errors)
    check_text_contract(root, errors)
    check_no_runtime_windows_paths(root, errors)
    check_packages(root, errors)
    check_xml_and_launch(root, errors)
    check_launch_config_root(root, errors)
    check_yaml(root, errors)
    check_templates_fail_safe(root, errors)
    check_profiles(root, errors)
    check_messages(root, errors)
    check_message_generation(root, errors)
    check_json_schemas(root, errors)
    check_python_syntax(root, errors)
    check_python38_compatibility(root, errors)
    check_gitattributes(root, errors)
    check_camera_template(root, errors)
    check_task_state_consistency(root, errors)
    check_hardware_authorization_boundary(root, errors)
    check_hardware_templates(root, errors)
    check_cpp_compile_contracts(root, errors)
    check_no_phase1_algorithm_dependencies(root, errors)
    check_static_ci(root, errors)
    check_ubuntu_validation_script(root, errors)
    check_runtime_baseline(root, errors)
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
