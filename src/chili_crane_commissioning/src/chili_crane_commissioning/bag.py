"""Explicitly configured bounded ROS bag capture, never raw-cloud all-day default."""
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time


class BagRecorder:
    def __init__(self,config,recorder,namespace,executable=None,process_factory=subprocess.Popen):
        if re.fullmatch('[A-Za-z][A-Za-z0-9_/]*',namespace) is None:raise ValueError('BAG_NAMESPACE_INVALID')
        self.config,self.recorder,self.namespace=dict(config),recorder,namespace.strip('/')
        self.executable=executable;self.process_factory=process_factory;self.process=None
        self.started=0.;self.output=None

    def command(self,profile):
        c=self.config
        if c.get('config_state')!='VALID' or c.get('enabled') is not True:raise PermissionError('BAG_PROFILE_NOT_CONFIGURED')
        if c.get('storage_id') not in ('sqlite3','mcap'):raise PermissionError('BAG_STORAGE_NOT_CONFIGURED')
        for key in ('max_bag_size_bytes','max_bag_duration_sec','max_total_storage_bytes','max_recording_duration_sec'):
            if type(c.get(key)) is not int or c[key]<=0:raise PermissionError('BAG_ROTATION_NOT_CONFIGURED')
        profiles=c.get('profiles',{})
        if profile not in ('state_only','event_debug') or profile not in profiles:raise ValueError('BAG_PROFILE_INVALID')
        topics=profiles[profile].get('topics',[])
        if not topics or len(topics)>256:raise PermissionError('BAG_TOPIC_BOUND')
        for topic in topics:
            if not isinstance(topic,str) or re.fullmatch('[A-Za-z][A-Za-z0-9_/]*',topic) is None:
                raise ValueError('BAG_TOPIC_INVALID')
            if 'points' in topic and (profile!='event_debug' or c.get('high_bandwidth',{}).get('config_state')!='VALID'):
                raise PermissionError('RAW_CLOUD_CAPTURE_NOT_APPROVED')
        executable=self.executable or shutil.which('ros2')
        if not executable:raise PermissionError('ROS2_BAG_NOT_AVAILABLE')
        output=self.recorder.directory/'rosbag'/(profile+'-'+str(time.time_ns()))
        self.output=output
        return [executable,'bag','record','--storage',c['storage_id'],'--output',str(output),
                '--max-bag-size',str(c['max_bag_size_bytes']),'--max-bag-duration',str(c['max_bag_duration_sec'])]+[
                    '/'+self.namespace+'/'+t for t in topics]

    def start(self,profile):
        if self.process is not None and self.process.poll() is None:raise ValueError('BAG_ALREADY_RECORDING')
        command=self.command(profile)
        self.process=self.process_factory(command,start_new_session=True)
        self.started=time.monotonic()
        self.recorder.append('events','ROSBAG_CAPTURE_STARTED',source_type='SOFTWARE',reason='NOT_FIELD_ACCEPTANCE',profile=profile)
        return {'profile':profile,'status':'RECORDING_NOT_FIELD_ACCEPTANCE'}

    def tick(self):
        if self.process is None or self.process.poll() is not None:return
        if time.monotonic()-self.started>=self.config['max_recording_duration_sec']:
            self.stop();return
        if self.output is not None and self.output.exists():
            entries=list(self.output.rglob('*'))
            total=sum(p.stat().st_size for p in entries if p.is_file() and not p.is_symlink())
            if len(entries)>10000 or total>=self.config['max_total_storage_bytes']:self.stop()

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:self.process.wait(timeout=10.)
            except subprocess.TimeoutExpired:self.process.terminate();self.process.wait(timeout=5.)
        self.process=None
        self.recorder.append('events','ROSBAG_CAPTURE_STOPPED',source_type='SOFTWARE',reason='OWNED_PROCESS_ONLY')
        return {'status':'STOPPED_OR_NOT_STARTED_NOT_BAG_ACCEPTANCE'}
