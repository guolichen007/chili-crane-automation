#!/usr/bin/env python3
"""Own the mock launch and its wrappers; no device transport is started."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
from owned_ros_process import start_owned,stop_owned,owned_process_leak_count


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--evidence-dir',type=Path,required=True);args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    with (args.evidence_dir/'mock-launch.log').open('w',encoding='utf-8') as log:
        process=start_owned(['ros2','launch','chili_crane_bringup','mock_system.launch.py'],stdout=log,stderr=subprocess.STDOUT)
        try:
            result=subprocess.run([sys.executable,str(root/'tools/validate_ros2_mock.py'),'--timeout-sec','25'],
                check=True,timeout=35,capture_output=True,text=True)
            assert process.poll() is None
            (args.evidence_dir/'mock-evidence.json').write_text(result.stdout,encoding='utf-8')
            print(result.stdout.strip())
        finally:stop_owned(process)
    assert owned_process_leak_count()==0


if __name__=='__main__':main()
