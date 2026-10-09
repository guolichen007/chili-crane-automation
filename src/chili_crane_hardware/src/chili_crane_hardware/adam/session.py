"""Linux bench locks and crash latch; external clients must still be stopped."""
import hashlib
import tempfile
from pathlib import Path
from contextlib import contextmanager


def session_paths(host, port, unit_id):
    identity = (str(host) + ":" + str(port) + ":" + str(unit_id)).encode()
    key = hashlib.sha256(identity).hexdigest()[:24]
    base = Path(tempfile.gettempdir()) / ("chili-adam-bench-" + key)
    return Path(str(base) + ".lock"), Path(str(base) + ".active")


@contextmanager
def device_lock(host, port, unit_id, exclusive=False):
    import fcntl
    path, latch = session_paths(host, port, unit_id)
    with path.open("a") as lock:
        try:
            fcntl.flock(lock, (fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH) | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError("device session is held by another bench client") from exc
        try:
            if not exclusive and latch.exists():
                raise RuntimeError("unverified bench OFF: readers blocked until isolated recovery")
            yield
        finally:
            fcntl.flock(lock, fcntl.LOCK_UN)


@contextmanager
def output_recovery_latch(host, port, unit_id):
    """Clear only on successful OFF readback; a killed writer leaves this latched."""
    _, latch = session_paths(host, port, unit_id)
    with latch.open("w", encoding="utf-8", newline="\n") as marker:
        marker.write("OFF_READBACK_PENDING\n")
        marker.flush()
        import os
        os.fsync(marker.fileno())
    try:
        yield
    except BaseException:
        # Remain latched on uncertain cleanup, even if an OFF attempt was made.
        raise
    else:
        latch.unlink()
