#!/usr/bin/env python3
"""Exact-base native gate reproduction. Does not reproduce historical DDS timing."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT=Path(__file__).resolve().parents[1]
BASE='1c14ac89d6429a31fa1cfcd3253b9eb7feabe56a'
sys.path[:0]=[str(ROOT/'tests_static'),str(ROOT/'src/chili_crane_control/src')]
from test_phase2c_evidence import Harness


def reproduce():
    path='src/chili_crane_control/src/chili_crane_control/grasp_interaction.py'
    source=subprocess.check_output(['git','show',BASE+':'+path],cwd=ROOT).decode('utf-8')
    name='chili_crane_control.r11_exact_baseline'
    module=types.ModuleType(name);sys.modules[name]=module
    exec(compile(source,BASE+':'+path,'exec'),module.__dict__)
    baseline=Harness();baseline.engine=module.GraspInteractionEngine(baseline.config)
    baseline.engine.bind(baseline.binding,10.1);baseline.warm();baseline.interact();old=baseline.lift(refresh=False)
    current=Harness();current.warm();current.interact();new=current.lift(refresh=False)
    gates=new.diagnostics
    required=('closure_confirmed','interaction_committed','bound_clear','raise_seen','vertical_settled',
              'load_sensor_valid','load_baseline_valid','load_source_calibration_allowed',
              'settling_window_valid','payload_gate_pass')
    assert all(gates[k] for k in required),gates
    assert old.payload['state']=='PAYLOAD_CANDIDATE'
    assert new.payload['state']=='PAYLOAD_CONFIRMED' and not new.lift['refresh_ready']
    return dict(scope='EXACT_BASE_NATIVE_ONLY_NOT_HISTORIC_ROS_REPRODUCTION',base_sha=BASE,
        BASELINE_NATIVE_GATE_REPRODUCED=True,BASELINE_NATIVE_BLOCKING_GATE='updated_clear',
        baseline_payload=old.payload['state'],r11_payload=new.payload['state'],gates=gates,
        HISTORIC_TIMEOUT_REPRODUCED=False,HISTORIC_TIMEOUT_FIXED=False,
        HISTORIC_STABLE_LIFT_TIMEOUT_ROOT_CAUSE='UNRESOLVED_MISSING_GATE_SAMPLES_UBUNTU_PENDING')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=reproduce();args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result))


if __name__=='__main__':main()
