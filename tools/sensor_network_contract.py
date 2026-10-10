#!/usr/bin/env python3
"""Sensor-only read-only site checks; ADAM status cannot affect their outcome."""
import argparse
from ipaddress import IPv4Interface
from pathlib import Path
import re
import shlex
import subprocess
import yaml

ROOT = Path(__file__).resolve().parents[1]


def settings(site, interface=None, profile=None, address=None):
    data = yaml.safe_load((site / "network/interfaces.yaml").read_text(encoding="utf-8"))
    cfg = dict(data["lidar"])
    for key, value in (("interface", interface), ("profile", profile), ("address", address)):
        if value is not None:
            cfg[key] = value
    for key in ("interface", "profile"):
        if not isinstance(cfg[key], str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", cfg[key]):
            raise ValueError("invalid network " + key)
    IPv4Interface(cfg["address"])
    if cfg.get("gateway") or cfg.get("dns") or cfg.get("never_default") is not True:
        raise ValueError("SENSOR_NETWORK_MUST_NOT_SET_DEFAULT_ROUTE_OR_DNS")
    cfg["sensor_ips"] = [str(yaml.safe_load((site / "sensors" / (s + ".yaml")).read_text())["host"])
                         for s in ("er1_204", "er1_205")]
    for ip in cfg["sensor_ips"]:
        if IPv4Interface(ip).ip not in IPv4Interface(cfg["address"]).network:
            raise ValueError("sensor outside host subnet")
    return cfg, data.get("control", {})


def run(*command):
    return subprocess.run(command, check=False, capture_output=True, text=True)


def profile_exists(cfg):
    result = run("nmcli", "-g", "connection.id", "connection", "show", cfg["profile"])
    return result.returncode == 0


def verify_profile(cfg):
    expected = {"connection.interface-name": cfg["interface"], "ipv4.method": "manual",
                "ipv4.addresses": cfg["address"], "ipv4.never-default": "yes",
                "ipv4.gateway": "", "ipv4.dns": "", "ipv6.method": "disabled"}
    failures = []
    for key, value in expected.items():
        result = run("nmcli", "-g", key, "connection", "show", cfg["profile"])
        if result.returncode or result.stdout.strip() != value:
            failures.append("PROFILE_MISMATCH:" + key)
    return failures


def check(cfg, control, check_adam=False):
    failures = verify_profile(cfg)
    host = run("ip", "-4", "-o", "address", "show", "dev", cfg["interface"])
    if host.returncode or cfg["address"] not in host.stdout.split():
        failures.append("HOST_ADDRESS_MISMATCH")
    for ip in cfg["sensor_ips"]:
        route = run("ip", "-4", "route", "get", ip)
        words = route.stdout.split()
        if route.returncode or not any(words[i:i+2] == ["dev", cfg["interface"]] for i in range(len(words))):
            failures.append("SENSOR_ROUTE_MISMATCH:" + ip)
    adam = "NOT_CHECKED"
    if check_adam:
        interface = control.get("interface")
        if not isinstance(interface, str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", interface):
            raise ValueError("invalid ADAM interface")
        result = run("ip", "-o", "link", "show", "dev", interface)
        adam = "CONNECTED" if result.returncode == 0 and "LOWER_UP" in result.stdout else "DISCONNECTED"
    return failures, adam


def dry_run(cfg):
    if profile_exists(cfg):
        failures = verify_profile(cfg)
        return failures, ["PROFILE_REVIEW_REQUIRED" if failures else "NO_CHANGE_REQUIRED"]
    command = ["sudo", "nmcli", "connection", "add", "type", "ethernet", "ifname", cfg["interface"],
               "con-name", cfg["profile"], "ipv4.method", "manual", "ipv4.addresses", cfg["address"],
               "ipv4.never-default", "yes", "ipv4.gateway", "", "ipv4.dns", "", "ipv6.method", "disabled"]
    return [], ["PROFILE_MISSING_REVIEW_ONLY", shlex.join(command)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["check", "dry-run"])
    parser.add_argument("--site", type=Path, default=ROOT / "config/sites/crane_01")
    for key in ("interface", "profile", "address"):
        parser.add_argument("--" + key)
    parser.add_argument("--check-adam", action="store_true")
    args = parser.parse_args()
    cfg, control = settings(args.site, args.interface, args.profile, args.address)
    print("READ_ONLY_NETWORK_CHECK / NO_DO_WRITE / NO_HOST_MODIFICATION")
    print("PROFILE=" + cfg["profile"] + " INTERFACE=" + cfg["interface"] + " ADDRESS=" + cfg["address"])
    if args.mode == "dry-run":
        failures, lines = dry_run(cfg)
        for line in lines:
            print(line)
    else:
        failures, adam = check(cfg, control, args.check_adam)
        print("ADAM_LINK=" + adam + " ADAM_REACHABILITY=NOT_CHECKED")
    for reason in failures:
        print(reason)
    print("SENSOR_ONLY_CHECK=" + ("FAIL" if failures else "PASS"))
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
