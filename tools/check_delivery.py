#!/usr/bin/env python3
"""Read-only pre-push checks; not a substitute for human review or secret scanning."""
import argparse
import json
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


def event_base(event):
    """Use the reviewed event range, never a permanent historical adoption SHA."""
    if "pull_request" in event:
        base = event["pull_request"]["base"]["sha"]
    elif "before" in event:
        base = event["before"]
        if base == "0" * 40:
            # No prior branch tip exists. Check the tip title and all public paths.
            # The full first-delivery range is checked locally with an explicit base.
            base = git("rev-parse", "HEAD^")
    else:
        raise ValueError("unsupported delivery event")
    if not isinstance(base, str) or re.fullmatch("[0-9a-f]{40}", base) is None:
        raise ValueError("event base must be an exact commit SHA")
    return base


def main():
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--base-sha")
    source.add_argument("--event-path", type=Path, help="GitHub push/PR event; CI only")
    parser.add_argument("--ci", action="store_true", help="read-only CI review, not a local push")
    args = parser.parse_args()
    if args.event_path:
        if not args.ci:
            parser.error("--event-path is CI-only; local delivery needs an explicit base")
        try:
            args.base_sha = event_base(json.loads(args.event_path.read_text(encoding="utf-8")))
        except (KeyError, ValueError, OSError, subprocess.CalledProcessError) as exc:
            parser.error("cannot resolve event range: " + str(exc))
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
    if args.event_path:
        print("CI_RANGE: event push/PR base; new branch uses tip parent only")
    print("OUTPUT_SHA: " + git("rev-parse", "HEAD"))
    print("BRANCH: " + branch)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
