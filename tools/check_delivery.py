#!/usr/bin/env python3
"""Read-only pre-push checks; not a substitute for human review or secret scanning."""
import argparse
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TITLE = re.compile(r"^(feat|fix|refactor|docs|test|ci|build|chore|perf|style|revert)(\([a-z0-9_-]+\))?!?: .+$")


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True, encoding="utf-8").strip()


def title_valid(title):
    return TITLE.fullmatch(title) is not None


def forbidden_path(path):
    parts = Path(path).parts
    return (path == "LOCAL_PROJECT_CONTEXT.md" or ".local-data" in parts
            or any(p in {"build", "install", "devel", "bags", ".aws"} for p in parts)
            or Path(path).name in {".env", "id_rsa", "id_ed25519"}
            or Path(path).suffix in {".pem", ".key", ".bag", ".db3"})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-sha", required=True)
    parser.add_argument("--ci", action="store_true", help="read-only CI review, not a local push")
    args = parser.parse_args()
    if re.fullmatch("[0-9a-f]{40}", args.base_sha) is None:
        parser.error("base SHA must be an exact 40-character commit")
    errors = []
    branch = git("branch", "--show-current")
    if not args.ci and not branch.startswith(("codex/", "ubuntu/")):
        errors.append("push candidate must be a review branch, never main")
    if git("status", "--porcelain"):
        errors.append("worktree/index must be clean before delivery")
    if not args.ci and git("remote", "get-url", "origin") != "git@github.com:guolichen007/chili-crane-automation.git":
        errors.append("origin must be the approved SSH repository")
    if subprocess.run(["git", "merge-base", "--is-ancestor", args.base_sha, "HEAD"], cwd=ROOT).returncode:
        errors.append("base SHA is not an ancestor of HEAD")
    for line in git("log", "--no-merges", "--format=%h %s", args.base_sha + "..HEAD").splitlines():
        sha, title = line.split(" ", 1)
        if not title_valid(title):
            errors.append("non-conventional new commit: " + sha)
    for path in git("ls-files").splitlines():
        if forbidden_path(path):
            errors.append("non-public/generated path is tracked: " + path)
    for error in errors:
        print("FAIL: " + error)
    if errors:
        return 1
    print("PRE_PUSH_STATIC: PASS; remote CI and human data review still required")
    print("BASE_SHA: " + args.base_sha)
    print("OUTPUT_SHA: " + git("rev-parse", "HEAD"))
    print("BRANCH: " + branch)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
