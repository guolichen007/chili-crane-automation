#!/usr/bin/env python3
"""Validate repository contracts without requiring ROS or Ubuntu."""

import ast
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
    "log",
    ".local-data",
}
EXPECTED_PACKAGES = {
    "chili_crane_bringup",
    "chili_crane_control",
    "chili_crane_core",
    "chili_crane_hardware",
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
    "scripts/validation/ubuntu22_ros2_phase0_validate.sh",
    "docs/decisions/0003-ros2-humble-hardware-bench.md",
    "config/hardware/io_mapping.template.yaml",
    "config/hardware/pull_wire.template.yaml",
    ".github/workflows/ros2-humble.yml",
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
    if list((root / "src").rglob("*.launch")):
        errors.append("active ROS1 XML launch is forbidden")
    forbidden = re.compile(r"\b(?:catkin|rospy|roslaunch|rostest|message_generation|message_runtime)\b")
    for path in iter_text_files(root / "src"):
        if path.suffix == ".md":
            continue
        if forbidden.search(path.read_text(encoding="utf-8")):
            errors.append("active ROS1 dependency: " + _relative(path, root))
    for path in (root / "src").glob("*/package.xml"):
        package = ET.parse(path).getroot()
        if package.attrib.get("format") != "3":
            errors.append("ROS2 package format must be 3: " + _relative(path, root))
        if package.findtext("export/build_type") != "ament_cmake":
            errors.append("package must export ament_cmake: " + _relative(path, root))

def check_launch_config_root(root: Path, errors: List[str]) -> None:
    launch_dir = root / "src" / "chili_crane_bringup" / "launch"
    for name in ("mapping", "localization", "mock_system", "bag_replay", "production", "bench_io"):
        path = launch_dir / (name + ".launch.py")
        if not path.is_file():
            errors.append("missing ROS2 launch: " + str(path))
            continue
        text = path.read_text(encoding="utf-8")
        if 'DeclareLaunchArgument("config_root", default_value=share + "/config")' not in text:
            errors.append("launch must use installed overridable config_root: " + name)
        if "adam_do_test" in text:
            errors.append("launch must never load bench output writer")
    cmake = (launch_dir.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    if "install(DIRECTORY ../../config/" not in cmake:
        errors.append("bringup must install configuration into package share")

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
        "builtin_interfaces/Time issued_stamp",
        "builtin_interfaces/Time expire_stamp",
    ):
        if field not in authorized:
            errors.append(f"AuthorizedCommand missing field: {field}")


def check_message_generation(root: Path, errors: List[str]) -> None:
    message_dir = root / "src" / "chili_crane_msgs" / "msg"
    actual = {path.name for path in message_dir.glob("*.msg")}
    cmake = (message_dir.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    declared = set(
        re.findall(
            r'^\s+"msg/([A-Za-z0-9_]+\.msg)"\s*$',
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


def check_python310_compatibility(root: Path, errors: List[str]) -> None:
    for path in iter_first_party_files(root):
        if path.suffix != ".py" or ".local-data" in path.parts:
            continue
        source = path.read_text(encoding="utf-8")
        try:
            tree = ast.parse(source, feature_version=(3, 10))
        except SyntaxError as exc:
            errors.append("Python 3.10 syntax: " + _relative(path, root) + ": " + str(exc))
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Import) and any(item.name == "tomllib" for item in node.names):
                errors.append("Python 3.11 tomllib is forbidden: " + _relative(path, root))
            if isinstance(node, ast.ImportFrom) and node.module == "tomllib":
                errors.append("Python 3.11 tomllib is forbidden: " + _relative(path, root))

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
        / "chili_crane_hardware"
        / "scripts"
        / "mock_hardware_adapter.py"
    ).read_text(encoding="utf-8")
    if "control/actuation_request" not in adapter:
        errors.append("hardware adapter must consume actuation_request")
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
        if "ament_add_gtest" not in cmake:
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
        'python-version: "3.10"',
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
    path = root / "scripts" / "validation" / "ubuntu22_ros2_phase0_validate.sh"
    text = path.read_text(encoding="utf-8")
    if not text.startswith("#!/usr/bin/env bash\nset -euo pipefail\n"):
        errors.append("Ubuntu22 validation script must be fail-fast")
    for required in ("--workspace", "--evidence-dir", "--expected-sha", "22.04",
                     "humble", "3.10", " build --base-paths", "colcon test", "validate_ros2_mock.py"):
        if required not in text:
            errors.append("Ubuntu22 script missing: " + required)

def check_runtime_baseline(root: Path, errors: List[str]) -> None:
    adr = (root / "docs/decisions/0003-ros2-humble-hardware-bench.md").read_text(encoding="utf-8")
    for required in ("Status: `ACCEPTED`", "Ubuntu 22.04", "ROS 2 Humble",
                     "Python 3.10", "C++17", "GCC 11", "24DI"):
        if required not in adr:
            errors.append("runtime ADR missing: " + required)
    old = (root / "docs/decisions/0001-runtime_baseline.md").read_text(encoding="utf-8")
    if "Status: `SUPERSEDED`" not in old:
        errors.append("ROS1 historical ADR must be SUPERSEDED")

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


def check_hardware_bench(root: Path, errors: List[str]) -> None:
    config = load_yaml(root / "config/hardware/io_mapping.template.yaml", root, errors) or {}
    if config.get("total_di_channels") != 24:
        errors.append("current baseline must have 24 DI including remote inputs")
    if config.get("devices") != {"adam6052": {"di_count": 8}, "adam6251": {"di_count": 16}}:
        errors.append("ADAM channel inventory must be 8+16")
    inputs = config.get("essential_signals", [])
    if "digital_inputs" in config or config.get("contract_version") != 4:
        errors.append("DI mapping must use canonical physical contract v4")
    for name in ("mode_auto", "safety_ok", "remote_y_left", "remote_g_close"):
        if name not in inputs:
            errors.append("missing semantic input: " + name)
    for item in config.get("optional_capabilities", {}).values():
        if item != "NOT_CONFIGURED":
            errors.append("template must not guess optional capabilities")
    for package in ("chili_crane_core", "chili_crane_control"):
        for path in iter_text_files(root / "src" / package):
            if path.suffix == ".md":
                continue
            if re.search(r"\b(?:ModbusTcpTransport|Adam6052RegisterMap|Adam6251RegisterMap)\b",
                         path.read_text(encoding="utf-8")):
                errors.append("vendor protocol outside hardware: " + _relative(path, root))
    qos = (root / "src/chili_crane_control/src/chili_crane_control/qos.py").read_text(encoding="utf-8")
    if "TRANSIENT_LOCAL" in qos or "VOLATILE" not in qos:
        errors.append("command QoS must be VOLATILE")
    for path in (root / "src/chili_crane_hardware/src/chili_crane_hardware").rglob("*.py"):
        if "nodes" in path.parts:
            continue
        text = path.read_text(encoding="utf-8")
        if re.search(r"^\s*(?:import rclpy|from rclpy)", text, re.MULTILINE):
            errors.append("pure hardware module must be ROS independent: " + str(path))


def check_phase05(root: Path, errors: List[str]) -> None:
    required = (
        "docs/decisions/0004-phase05-architecture-freeze.md",
        "docs/api/PHASE05_CONTRACTS.md", "docs/api/phase05_acceptance.yaml",
        "docs/review/REVIEW_GUIDE.md", "CONTRIBUTING.md",
        ".github/PULL_REQUEST_TEMPLATE.md", "tools/check_delivery.py",
        "config/hardware/sensor_inventory.yaml", "config/control/axis_capabilities.template.yaml",
        "config/control/profiles/variable_speed.yaml", "config/control/profiles/fixed_slow.yaml",
        "config/recording/run_manifest.template.yaml", "config/recording/rosbag2_profile.yaml",
    )
    for relative in required:
        if not (root / relative).is_file():
            errors.append("Phase0.5 required file missing: " + relative)
    physical = load_yaml(root / "config/hardware/io_mapping.template.yaml", root, errors) or {}
    channels = physical.get("physical_channels", {})
    for device, count in (("adam6052", 8), ("adam6251", 16)):
        items = channels.get(device, [])
        if [item.get("channel") for item in items] != list(range(count)):
            errors.append("physical channel inventory must enumerate all24: " + device)
        if any(item.get("assignment") != "NOT_CONFIGURED" for item in items):
            errors.append("template must not guess physical assignments: " + device)
        if any(item.get("signal") != "NOT_CONFIGURED" or item.get("invert") != "NOT_CONFIGURED" for item in items):
            errors.append("template must not guess physical signal/polarity: " + device)
    adapter = (root / "src/chili_crane_hardware/scripts/mock_hardware_adapter.py").read_text(encoding="utf-8")
    if "control/authorized_command" in adapter or "ActuationRequest" not in adapter:
        errors.append("Phase0.5 hardware must consume only ActuationRequest")
    cmake = (root / "src/chili_crane_control/CMakeLists.txt").read_text(encoding="utf-8")
    if "ament_add_pytest_test" not in cmake:
        errors.append("pure executor scenario tests must run in colcon")
    recording = load_yaml(root / "config/recording/rosbag2_profile.yaml", root, errors) or {}
    if recording.get("enabled") is not False:
        errors.append("recording skeleton must default disabled")
    for name in ("AuthorizedCommand", "ActuationRequest"):
        text = (root / ("src/chili_crane_msgs/msg/" + name + ".msg")).read_text(encoding="utf-8")
        sequences = ("uint64 sequence",) if name == "AuthorizedCommand" else (
            "uint64 command_sequence", "uint64 actuation_sequence")
        for field in ("string session_id", "uint64 command_epoch") + sequences:
            if field not in text:
                errors.append(name + " missing ownership field: " + field)
    matrix = load_yaml(root / "docs/api/phase05_acceptance.yaml", root, errors) or {}
    if len(matrix.get("contracts", [])) != 15:
        errors.append("Phase0.5 must expose the15 named acceptance contracts")
    for item in matrix.get("contracts", []):
        if not (root / item["implementation"]).is_file():
            errors.append("acceptance implementation path missing: " + item["implementation"])


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
    check_python310_compatibility(root, errors)
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
    check_hardware_bench(root, errors)
    check_phase05(root, errors)
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
