#!/usr/bin/env python3
"""Four deterministic integrated shadow scenarios; no physical/ROS claims."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/chili_crane_control/src'),str(ROOT/'src/chili_crane_perception/src')]
from chili_crane_perception.phase2c_synthetic import SyntheticInteractionRun


def validate():
    config=yaml.safe_load((ROOT/'config/algorithms/phase2c_interaction.synthetic.yaml').read_text(encoding='utf-8'))
    results=[];counters={'TARGET_SWAP_ACCEPTED_COUNT':0,'ACCELERATION_PAYLOAD_CONFIRMED_COUNT':0,
        'INTERACTION_WITHOUT_LOCAL_INVALIDATION_COUNT':0,'OLD_GENERATION_NEXT_BIND_ACCEPTED_COUNT':0}
    for name in ('SUCCESSFUL_GRASP','FAILED_GRASP_AFTER_INTERACTION','LOAD_ACCELERATION_FALSE_POSITIVE','TARGET_SWAP_ATTACK'):
        run=SyntheticInteractionRun(config,ROOT/'config/safety/pit_operational_geofence.template.yaml');run.warm()
        if name=='TARGET_SWAP_ATTACK':
            candidate=replace(run.binding,candidate_id='attacker-target',binding_epoch_ns=int(round(run.t*1e9)),
                evidence=replace(run.binding.evidence,epoch_ns=int(round(run.t*1e9))),binding_digest_sha256='')
            try:run.engine.bind(candidate,run.t);counters['TARGET_SWAP_ACCEPTED_COUNT']+=1
            except ValueError:pass
            assert run.engine.binding==run.binding
            results.append(dict(scenario=name,status='PASS_REJECTED',binding_digest_unchanged=True));continue
        run.interact()
        counters['INTERACTION_WITHOUT_LOCAL_INVALIDATION_COUNT']+=int(not run.life.affected or not run.life.interaction_id)
        result=run.lift(value=100. if name=='FAILED_GRASP_AFTER_INTERACTION' else 101.,
                        acceleration=name=='LOAD_ACCELERATION_FALSE_POSITIVE')
        payload_after_lift=result.payload['state']
        if name=='SUCCESSFUL_GRASP':
            assert result.payload['state']=='PAYLOAD_CONFIRMED',result.payload
            assert result.lift['clear_of_updated_surface'] and result.lift['clear_of_pit']
            for _ in range(60):result=run.step('VERIFY_EMPTY',value=100.)
            assert result.unload['state']=='UNLOAD_CONFIRMED'
        elif name=='FAILED_GRASP_AFTER_INTERACTION':
            assert result.payload['state']=='PAYLOAD_NOT_CONFIRMED'
            assert result.interaction['invalidate'] and run.refresh.ready
        else:
            counters['ACCELERATION_PAYLOAD_CONFIRMED_COUNT']+=int(result.payload['state']=='PAYLOAD_CONFIRMED')
            assert not run.refresh.ready
        run.engine.last_phase='COMPLETE'
        old=replace(run.binding,cycle_id='synthetic-task:2',cycle_index=2,binding_id='old-generation-attempt',
            binding_epoch_ns=int(round(run.t*1e9)),evidence=replace(run.binding.evidence,epoch_ns=int(round(run.t*1e9))),binding_digest_sha256='')
        try:run.engine.bind(old,run.t);counters['OLD_GENERATION_NEXT_BIND_ACCEPTED_COUNT']+=1
        except ValueError:pass
        results.append(dict(scenario=name,status='PASS',contact=result.contact['state'],interaction=result.interaction['state'],
            payload_after_lift=payload_after_lift,unload=result.unload['state'],affected_cells=len(run.life.affected),
            frozen_generation=run.binding.surface_generation,current_generation=run.builder.grid.generation,
            refreshed=run.refresh.ready if run.refresh else False))
    assert not any(counters.values()),counters
    return dict(scope='DETERMINISTIC_INTEGRATED_SYNTHETIC_ONLY',status='PASS_SOFTWARE_SHADOW_NOT_FIELD',
        scenarios=results,counters=counters,physical_output_enabled=False,automatic_control_enabled=False,
        BENCH_DO_STATUS='NOT_RUN',FIELD_GRASP_VALIDATION='NOT_RUN',FULL_AUTO='NOT_RUN')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    result=validate();args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
