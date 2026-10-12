#!/usr/bin/env python3
"""Executable shim; run the frozen observer implementation without modifying it."""
from pathlib import Path
import runpy
from ament_index_python.packages import get_package_share_directory

if __name__ == "__main__":
    implementation = Path(get_package_share_directory("chili_crane_hardware")) / "frozen/er1_clock_observer.py"
    runpy.run_path(str(implementation), run_name="__main__")
