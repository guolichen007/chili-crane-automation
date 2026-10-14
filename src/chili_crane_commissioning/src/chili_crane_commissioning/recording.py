"""Owned run directories and append-only, provenance-complete JSONL black box."""
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import re
import threading
import time


def clean(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value


class RunRecorder:
    def __init__(self,root,run_id,git_sha,config_hash,profile='state_only'):
        if re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,95}',run_id) is None:
            raise ValueError('RUN_ID_PATH_INVALID')
        if re.fullmatch('[0-9a-f]{40}',git_sha) is None or re.fullmatch('[0-9a-f]{64}',config_hash) is None:
            raise ValueError('EXACT_GIT_SHA_AND_CONFIG_SHA_REQUIRED')
        base=Path(root).resolve();base.mkdir(parents=True,exist_ok=True)
        self.directory=base/run_id;self.directory.mkdir(exist_ok=False)
        if self.directory.resolve().parent!=base:raise ValueError('RUN_DIRECTORY_ESCAPE')
        for name in ('rosbag','reports'):(self.directory/name).mkdir()
        self.run_id,self.git_sha,self.config_hash=run_id,git_sha,config_hash
        self.lock=threading.RLock()
        self.ros_now=None
        manifest=dict(schema_version='RUN_MANIFEST_V3',phase='2C',run_id=run_id,git_sha=git_sha,config_hash=config_hash,
            recording_profile=profile,started_stamp=datetime.now(timezone.utc).isoformat(),
            journal_enabled=True,rosbag_configured=False,rosbag_enabled=False,rosbag_process_alive=False,
            recording_enabled=False,
            source_type='SIMULATED',physical_output_enabled=False,automatic_control_enabled=False,
            BENCH_DO_STATUS='NOT_RUN',FIELD_GRASP_VALIDATION='NOT_RUN',FULL_AUTO='NOT_RUN')
        with (self.directory/'manifest.json').open('x',encoding='utf-8',newline='\n') as stream:
            stream.write(json.dumps(manifest,ensure_ascii=False,sort_keys=True,indent=2)+'\n')

    def recording_status(self,**status):
        """Manifest is current status; the append-only journal preserves transitions."""
        with self.lock:
            path=self.directory/'manifest.json'
            manifest=json.loads(path.read_text(encoding='utf-8'));manifest.update(status)
            temporary=self.directory/'manifest.tmp'
            temporary.write_text(json.dumps(manifest,ensure_ascii=False,sort_keys=True,indent=2)+'\n',encoding='utf-8',newline='\n')
            temporary.replace(path)

    def append(self,name,event_type,source_type='SIMULATED',ros_timestamp=None,task_id='',cycle_id='',binding_id='',
               old_state='',new_state='',reason='',**details):
        if name not in ('events','operator_actions','bench_measurements'):raise ValueError('RECORD_STREAM_NOT_ALLOWED')
        if ros_timestamp is None and self.ros_now is not None:ros_timestamp=self.ros_now()
        record=dict(wall_timestamp=datetime.now(timezone.utc).isoformat(),ros_timestamp=ros_timestamp,
            monotonic_timestamp=time.monotonic(),run_id=self.run_id,task_id=task_id,cycle_id=cycle_id,binding_id=binding_id,
            git_sha=self.git_sha,config_hash=self.config_hash,source_type=source_type,event_type=event_type,
            old_state=old_state,new_state=new_state,reason=reason,details=clean(details))
        raw=json.dumps(record,ensure_ascii=False,sort_keys=True,allow_nan=False,separators=(',',':'))+'\n'
        with self.lock:
            with (self.directory/(name+'.jsonl')).open('a',encoding='utf-8',newline='\n') as stream:
                stream.write(raw);stream.flush()
        return record

    def read(self,name,maximum_bytes=1_048_576):
        allowed={'manifest':'manifest.json','events':'events.jsonl','operator_actions':'operator_actions.jsonl',
                 'bench_measurements':'bench_measurements.jsonl'}
        if name not in allowed:raise ValueError('RECORD_DOWNLOAD_NOT_ALLOWED')
        path=self.directory/allowed[name]
        if not path.exists():return b''
        if path.stat().st_size>maximum_bytes:raise ValueError('RECORD_TOO_LARGE_EXPORT_FROM_OWNED_RUN_DIRECTORY')
        return path.read_bytes()
