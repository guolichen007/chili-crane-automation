"""Scan public text for credential assignments; never print matched values."""
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATTERNS = (
    re.compile(r"(?im)^\s*(?:password|passwd|token|cookie|authorization|credential|密码|口令)\s*[:=]\s*\S+"),
    re.compile(r"-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----"),
    re.compile(r"\b(?:ghp_|github_pat_)[A-Za-z0-9_]{20,}"),
    re.compile(r"(?i)(?:digest\s+(?:username|realm)|set-cookie\s*:|authorization\s*:\s*(?:basic|bearer))"),
)


def violations(root=ROOT):
    paths = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=root).decode("utf-8").split("\0")
    errors = []
    for name in set(paths):
        path = root / name
        if not name or path.suffix not in {".md", ".yaml", ".yml", ".json", ".html"}:
            continue
        if path.name == "LOCAL_PROJECT_CONTEXT.md":
            continue
        for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if any(pattern.search(line) for pattern in PATTERNS):
                errors.append(f"{name}:{number}: credential-like material (redacted)")
    return errors


if __name__ == "__main__":
    found = violations()
    for error in found:
        print(error)
    print("PUBLIC_SECRET_SCAN=" + ("FAIL" if found else "PASS"))
    raise SystemExit(bool(found))
