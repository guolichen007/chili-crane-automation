"""Only validator-created sessions may be signalled; wrappers cannot leak children."""
import ctypes
import atexit
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

_owned={}
_tokens={}
_groups={}
_subreaper=False


def _enable_subreaper():
    global _subreaper
    if _subreaper:return
    if os.name!='posix' or not Path('/proc').is_dir():raise RuntimeError('OWNED_ROS_PROCESSES_REQUIRE_LINUX')
    # Reap descendants orphaned by a ros2 wrapper, including after its own exit.
    if ctypes.CDLL(None,use_errno=True).prctl(36,1,0,0,0)!=0:
        raise OSError(ctypes.get_errno(),'PR_SET_CHILD_SUBREAPER')
    _subreaper=True


def start_owned(argv,**kwargs):
    _enable_subreaper()
    if 'start_new_session' in kwargs and kwargs['start_new_session'] is not True:
        raise ValueError('OWNED_PROCESS_MUST_START_NEW_SESSION')
    kwargs['start_new_session']=True
    token=uuid.uuid4().hex
    env=dict(kwargs.pop('env',os.environ));env['CHILI_VALIDATOR_PROCESS_OWNER']=token
    process=subprocess.Popen(argv,env=env,**kwargs)
    _owned[process.pid]=process
    _tokens[process.pid]=token;_groups[process.pid]={process.pid}
    return process


def _processes():
    rows=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            fields=(entry/'stat').read_text().rsplit(')',1)[1].split()
            marker=next((v.split(b'=',1)[1].decode() for v in (entry/'environ').read_bytes().split(b'\0')
                         if v.startswith(b'CHILI_VALIDATOR_PROCESS_OWNER=')),None)
            rows.append((int(entry.name),int(fields[2]),marker))
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return rows


def _members(process):
    rows=_processes();groups=_groups[process.pid]
    groups.update(group for _,group,marker in rows if marker==_tokens[process.pid])
    if os.getpgrp() in groups:raise RuntimeError('VALIDATOR_PARENT_GROUP_MUST_NEVER_BE_SIGNALLED')
    return [pid for pid,group,_ in rows if group in groups]


def _reap(process):
    process.poll()
    for group in _groups[process.pid]:
        while True:
            try:
                pid,_=os.waitpid(-group,os.WNOHANG)
                if pid==0:break
            except ChildProcessError:break


def stop_owned(process,interrupt_timeout=3.,terminate_timeout=2.):
    if _owned.get(process.pid) is not process:raise ValueError('PROCESS_GROUP_NOT_OWNED_BY_THIS_VALIDATOR')
    for sig,budget in ((signal.SIGINT,interrupt_timeout),(signal.SIGTERM,terminate_timeout)):
        _reap(process)
        if not _members(process):break
        for group in _groups[process.pid]:
            try:os.killpg(group,sig)
            except ProcessLookupError:pass
        deadline=time.monotonic()+budget
        while time.monotonic()<deadline:
            _reap(process)
            if not _members(process):break
            time.sleep(.02)
    _reap(process)
    leaked=len(_members(process))
    if leaked:raise RuntimeError('VALIDATOR_OWNED_PROCESS_LEAK_COUNT='+str(leaked))
    _owned.pop(process.pid)
    _tokens.pop(process.pid);_groups.pop(process.pid)
    print('VALIDATOR_OWNED_PROCESS_LEAK_COUNT=0 PGID='+str(process.pid),file=sys.stderr)
    return 0


def owned_process_leak_count():
    for process in list(_owned.values()):_reap(process)
    return sum(len(_members(process)) for process in _owned.values())


def stop_all_owned():
    errors=[]
    for process in list(_owned.values()):
        try:stop_owned(process)
        except (OSError,RuntimeError,ValueError) as exc:errors.append(str(exc))
    if errors:raise RuntimeError('; '.join(errors))


atexit.register(stop_all_owned)
