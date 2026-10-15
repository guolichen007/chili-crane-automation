#!/usr/bin/env python3
"""Fail on mixed stage prefixes; emit only bounded ROS/environment provenance."""
import argparse
import json
import importlib
import os
from pathlib import Path
import subprocess

PREFIX_KEYS=('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH','PATH')
PACKAGES=('chili_crane_core','chili_crane_msgs','chili_crane_slam','chili_crane_hardware',
          'chili_crane_control','chili_crane_perception','chili_crane_bringup','chili_crane_commissioning')


def assert_clean_prefixes(environment,forbidden):
    roots=[Path(p).resolve() for p in forbidden]
    for key in PREFIX_KEYS:
        for entry in environment.get(key,'').split(os.pathsep):
            if entry and any(Path(entry).resolve().is_relative_to(root) for root in roots):
                raise ValueError('MIXED_OVERLAY_'+key)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--expected-root',type=Path)
    parser.add_argument('--forbidden-root',action='append',default=[])
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    assert_clean_prefixes(os.environ,args.forbidden_root)
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    domain=os.environ.get('ROS_DOMAIN_ID','')
    assert domain.isascii() and domain.isdecimal() and 0<=int(domain)<=232
    resolved={};modules={}
    if args.expected_root:
        from ament_index_python.packages import get_package_prefix
        for package in PACKAGES:
            prefix=Path(get_package_prefix(package)).resolve()
            command_prefix=Path(subprocess.check_output(['ros2','pkg','prefix',package],text=True).strip()).resolve()
            assert prefix==command_prefix and prefix.is_relative_to(args.expected_root.resolve()),(package,prefix)
            resolved[package]=str(prefix)
        for package in ('chili_crane_msgs.msg','chili_crane_slam','chili_crane_hardware','chili_crane_control',
                        'chili_crane_perception','chili_crane_commissioning'):
            module=importlib.import_module(package)
            file=Path(module.__file__).resolve()
            assert_clean_prefixes({'PYTHONPATH':str(file)},args.forbidden_root)
            modules[package]=str(file)
        from chili_crane_msgs.msg import PitSurfaceStatus,GraspProposal
        assert hasattr(PitSurfaceStatus(),'scan_cycle_id') and hasattr(GraspProposal(),'surface_window_end')
    result=dict(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=int(domain),NORMAL_OVERLAY_PRESENT_IN_SYMLINK_ENV='NO',
        MIXED_PACKAGE_RESOLUTION='NO',resolved_packages=resolved,resolved_python_modules=modules,
        environment={k:os.environ.get(k,'') for k in PREFIX_KEYS})
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
