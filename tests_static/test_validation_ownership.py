"""Atomic frame scheduling and Linux process-group ownership counterexamples."""
import copy
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as S
import unittest
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from lidar_frame_bundle import AtomicLidarPublisher
from owned_ros_process import start_owned,stop_owned,owned_process_leak_count
from check_ros_overlay import assert_clean_prefixes


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,m):self.messages.append(copy.deepcopy(m))


def bundle(counter):
    header=S(frame_id='sensor',stamp=S(sec=counter,nanosec=0))
    cloud=S(header=copy.deepcopy(header))
    meta=S(header=copy.deepcopy(header),measurement_stamp=copy.deepcopy(header.stamp),source_counter=counter)
    context=S(header=copy.deepcopy(header),evidence=S(header=copy.deepcopy(header),
        measurement_stamp=copy.deepcopy(header.stamp),source_counter=counter))
    return cloud,meta,context


class FrameBundleTests(unittest.TestCase):
    def test_no_interframe_metadata_overwrite_and_atomic_identity(self):
        pubs={k:Publisher() for k in ('cloud','meta','context')};publisher=AtomicLidarPublisher(pubs)
        built=[]
        def factory():
            built.append(len(built)+1);return bundle(built[-1])
        for i in range(20):
            before=copy.deepcopy(publisher.counts)
            changed=publisher.publish_due(i*.05,factory)
            if not changed:self.assertEqual(publisher.counts,before)
            self.assertEqual(len(set(publisher.counts.values())),1)
        self.assertGreater(len(built),1);self.assertLess(len(built),20)
        self.assertEqual(len(pubs['cloud'].messages),len(built))
        for cloud,meta,context in zip(*(pubs[k].messages for k in ('cloud','meta','context'))):
            self.assertEqual(cloud.header.stamp,meta.measurement_stamp)
            self.assertEqual(cloud.header.stamp,context.evidence.measurement_stamp)
            self.assertEqual(meta.source_counter,context.evidence.source_counter)

    def test_malformed_or_regressed_bundle_never_partially_publishes(self):
        for attack in ('stamp','counter'):
            pubs={k:Publisher() for k in ('cloud','meta','context')};publisher=AtomicLidarPublisher(pubs)
            bad=bundle(1)
            if attack=='stamp':bad[2].evidence.measurement_stamp.sec=2
            else:bad[2].evidence.source_counter=2
            with self.assertRaisesRegex(ValueError,'IDENTITY_CONFLICT'):publisher.publish_due(0.,lambda:bad)
            self.assertTrue(all(not p.messages for p in pubs.values()))

    def test_product_strict_epoch_equality_is_preserved(self):
        line='if meta.measurement_stamp != cloud.header.stamp or context_msg.header.stamp != cloud.header.stamp:'
        self.assertIn(line,(ROOT/'src/chili_crane_perception/src/chili_crane_perception/surface_runtime.py').read_text())

    def test_normal_prefix_in_symlink_environment_is_rejected(self):
        with tempfile.TemporaryDirectory() as root:
            normal=Path(root)/'install';symlink=Path(root)/'symlink-install'
            assert_clean_prefixes({'AMENT_PREFIX_PATH':str(symlink)},[normal])
            for key in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH','PATH'):
                with self.assertRaisesRegex(ValueError,'MIXED_OVERLAY'):
                    assert_clean_prefixes({key:str(normal/'chili_crane_control')},[normal])

    def test_authority_stages_start_from_clean_underlay_and_keep_probes(self):
        main=(ROOT/'scripts/validation/ubuntu22_ros2_phase0_validate.sh').read_text()
        stage=(ROOT/'scripts/validation/ubuntu22_ros2_stage.sh').read_text()
        self.assertIn('env -i',main);self.assertIn('ROS_LOCALHOST_ONLY=1',main)
        self.assertNotIn('install/setup.bash',main)
        self.assertIn('. /opt/ros/humble/setup.sh',stage)
        self.assertIn('. "$install/local_setup.sh"',stage)
        self.assertIn('--probe 10',stage);self.assertIn('timeout 300s',stage)
        self.assertNotIn('validate_phase2c_synthetic.py',stage)


@unittest.skipUnless(os.name=='posix' and Path('/proc').is_dir(),'NOT_RUN: Linux process groups/subreaper unavailable')
class OwnedProcessTests(unittest.TestCase):
    def test_wrapper_exit_does_not_leave_its_child_in_same_or_new_session(self):
        for separate_session in (False,True):
            script=('import subprocess,sys; '
                'subprocess.Popen([sys.executable,"-c","import time;time.sleep(30)"],start_new_session='+str(separate_session)+')')
            p=start_owned([sys.executable,'-c',script],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            self.assertEqual(p.wait(timeout=3.),0)
            self.assertEqual(stop_owned(p,interrupt_timeout=.3,terminate_timeout=.5),0)
            self.assertEqual(owned_process_leak_count(),0)

    def test_sigterm_escalation_is_bounded_and_only_for_owned_group(self):
        p=start_owned([sys.executable,'-c','import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);print("ready",flush=True);time.sleep(30)'],
            stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True)
        self.assertEqual(p.stdout.readline().strip(),'ready')
        self.assertEqual(stop_owned(p,interrupt_timeout=.1,terminate_timeout=.5),0)
        p.stdout.close();self.assertEqual(owned_process_leak_count(),0)

    def test_unowned_process_group_is_rejected_before_signalling(self):
        with self.assertRaisesRegex(ValueError,'NOT_OWNED'):stop_owned(S(pid=os.getpid()))


if __name__=='__main__':unittest.main()
