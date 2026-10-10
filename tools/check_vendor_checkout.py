#!/usr/bin/env python3
"""Reject any vendor change other than the reviewed exact XYZIRT build switch."""
import argparse
import subprocess
from pathlib import Path


def git(path, *args):
    return subprocess.check_output(["git", "-C", str(path), *args])


def check(workspace):
    sdk = workspace / "src/sdk"
    expected = git(sdk, "show", "HEAD:CMakeLists.txt").replace(
        b"set(POINT_TYPE XYZI)", b"set(POINT_TYPE XYZIRT)")
    if (sdk / "CMakeLists.txt").read_bytes() != expected:
        raise ValueError("vendor CMake differs from the exact reviewed XYZIRT switch")
    if git(sdk, "diff", "--name-only").strip() != b"CMakeLists.txt":
        raise ValueError("unexpected SDK worktree edits")
    for path in (sdk, sdk / "src/rs_driver", workspace / "src/rslidar_msg"):
        if git(path, "diff", "--cached", "--name-only").strip() or git(path, "ls-files", "--others", "--exclude-standard").strip():
            raise ValueError("vendor index/untracked edits detected")
        if path != sdk and git(path, "diff", "--name-only").strip():
            raise ValueError("vendor message/driver edits detected")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("workspace", type=Path)
    check(parser.parse_args().workspace)
    print("VENDOR_CHECKOUT=EXACT_REVIEWED_PATCH")
