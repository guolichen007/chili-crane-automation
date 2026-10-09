"""Load an explicitly chosen hardware configuration."""
from pathlib import Path
import yaml
from chili_crane_hardware.adam.modbus_tcp_transport import ModbusTcpTransport


def load_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("hardware configuration must be a mapping")
    return data


def tcp_from_config(config):
    return ModbusTcpTransport(config.get("host"), config.get("port", 502),
                              config.get("unit_id"), config.get("timeout_sec", 0.5))
