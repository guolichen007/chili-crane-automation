"""One bounded isolated DO pulse with best-effort OFF and readback."""
import argparse
import math
import signal
import time
from chili_crane_hardware.adam.adam6052_driver import Adam6052Driver, BenchOutputGate
from chili_crane_hardware.adam.session import output_recovery_latch
from .common import load_config, tcp_from_config


def pulse_once(driver, channel, duration_sec, sleeper=time.sleep):
    driver.gate.require()
    if type(channel) is not int or not 0 <= channel <= 7:
        raise ValueError("channel must be 0..7")
    if not math.isfinite(duration_sec) or not 0 < duration_sec <= 1.0:
        raise ValueError("bench pulse must be >0 and <=1 second")
    try:
        driver.all_off()
        driver.bench_on(channel)
        sleeper(duration_sec)
    finally:
        # Network loss can make this fail; configured device WDT is mandatory.
        driver.all_off()


def interrupted(signum, frame):
    raise KeyboardInterrupt("bench interrupted by signal " + str(signum))


def main():
    parser = argparse.ArgumentParser(description="Isolated ADAM6052 one-shot pulse")
    parser.add_argument("--config", required=True)
    parser.add_argument("--channel", type=int, required=True)
    parser.add_argument("--duration-ms", type=int, required=True)
    parser.add_argument("--enable-physical-output", action="store_true")
    parser.add_argument("--confirm-isolated-bench", action="store_true")
    parser.add_argument("--confirm-exclusive-device-session", action="store_true")
    args = parser.parse_args()
    config = load_config(args.config)
    acceptance = config.get("acceptance", {})
    gate = BenchOutputGate(
        physical_output_enabled=args.enable_physical_output and config.get("physical_output_enabled") is True,
        isolated_bench_confirmed=args.confirm_isolated_bench,
        adam_fsv_configured=acceptance.get("ADAM_FSV_CONFIGURED") is True,
        adam_watchdog_configured=acceptance.get("ADAM_WATCHDOG_CONFIGURED") is True,
        adam_safe_output_verified=acceptance.get("ADAM_SAFE_OUTPUT_VERIFIED") is True)
    gate.require()
    if not args.confirm_exclusive_device_session:
        raise PermissionError("stop all other device clients before a pulse")
    if config.get("automatic_control_enabled") is not False:
        raise PermissionError("automatic control must be disabled")
    watchdog = acceptance.get("watchdog_timeout_sec")
    if type(watchdog) not in (int, float) or not math.isfinite(watchdog) or not 0 < watchdog <= 2:
        raise PermissionError("verified watchdog timeout must be >0 and <=2 seconds")
    if acceptance.get("verification_evidence") in (None, "", "NOT_CONFIGURED"):
        raise PermissionError("device OFF/watchdog evidence must be recorded")
    if not 1 <= args.duration_ms <= 1000 or not 0 <= args.channel <= 7:
        parser.error("channel 0..7 and duration 1..1000ms are required")
    driver = Adam6052Driver(tcp_from_config(config), gate)
    # Readers in this framework cannot refresh device WDT during a pulse.
    with driver.transport.exclusive_session(), output_recovery_latch(
            driver.transport.host, driver.transport.port, driver.transport.unit_id):
        previous = {sig: signal.signal(sig, interrupted) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            pulse_once(driver, args.channel, args.duration_ms / 1000.0)
            print("BENCH_PULSE: COMPLETE; ALL_OUTPUTS_OFF_READBACK: VERIFIED")
        finally:
            # Ignore further termination signals only while attempting finite cleanup.
            for sig in previous:
                signal.signal(sig, signal.SIG_IGN)
            try:
                driver.all_off()
            finally:
                for sig, handler in previous.items():
                    signal.signal(sig, handler)


if __name__ == "__main__":
    main()
