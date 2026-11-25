"""Static plus simulated middleware tests. No ROS/Gazebo required."""
import ast
import os
from pathlib import Path
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'race/scripts/race_runtime.py'

class FakeDuration:
    def __init__(self,seconds=0):
        self.seconds=seconds

class FakePublisher:
    def __init__(self,acks,subscriptions=1):
        self.acks=list(acks)
        self.subscriptions=subscriptions
        self.sent=[]
        self.waits=[]
    def get_subscription_count(self):return self.subscriptions
    def publish(self,msg):self.sent.append(msg.data)
    def wait_for_all_acked(self,duration):
        self.waits.append(duration.seconds)
        return self.acks.pop(0) if self.acks else False

class FakeROS:
    def __init__(self):self.spins=0
    def spin_once(self,node,timeout_sec=0):self.spins+=1

class FakeBool:
    def __init__(self):self.data=None

class FakeTimingError(Exception):pass


def method_from_source():
    source=SOURCE.read_text(encoding='utf-8')
    tree=ast.parse(source)
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='StartGate')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='publish')
    method.decorator_list=[]
    mod=ast.Module(body=[method],type_ignores=[])
    ast.fix_missing_locations(mod)
    namespace={'TimingError':FakeTimingError,'time':__import__('time')}
    import sys,types
    fake=types.ModuleType('rclpy');fake_duration=types.ModuleType('rclpy.duration');fake_duration.Duration=FakeDuration
    with patch.dict(sys.modules,{'rclpy':fake,'rclpy.duration':fake_duration}):
        exec(compile(mod,str(SOURCE),'exec'),namespace)
    return namespace['publish']

class GateTest(unittest.TestCase):
    def setUp(self):
        self.fn=method_from_source()
        self.ros=FakeROS()
        self.gate=type('Gate',(),{})()
        self.gate.node=object()
        self.gate.rclpy=self.ros
        self.gate.Bool=FakeBool

    def call(self,ids,value):
        import sys,types
        fake=types.ModuleType('rclpy');fake_duration=types.ModuleType('rclpy.duration');fake_duration.Duration=FakeDuration
        with patch.dict(sys.modules,{'rclpy':fake,'rclpy.duration':fake_duration}), patch('time.sleep',return_value=None):
            return self.fn(self.gate,ids,value)

    def test_immediate_delivery(self):
        a=FakePublisher([True]);b=FakePublisher([True])
        self.gate.pubs={'team_01':a,'team_11':b}
        self.call(['team_01','team_11'],False)
        self.assertEqual(a.sent,[False]);self.assertEqual(b.sent,[False])
        self.assertEqual(a.waits,[3]);self.assertEqual(b.waits,[3])

    def test_retry_and_recover(self):
        a=FakePublisher([False,True]);self.gate.pubs={'team_11':a}
        self.call(['team_11'],True)
        self.assertEqual(a.sent,[True,True]);self.assertEqual(self.ros.spins,2)

    def test_persistent_failure_stops_race(self):
        a=FakePublisher([False]*4);self.gate.pubs={'team_11':a}
        with self.assertRaisesRegex(FakeTimingError,'4회 재시도'):
            self.call(['team_11'],True)
        self.assertEqual(len(a.sent),4)

    def test_absent_subscriber_never_releases(self):
        a=FakePublisher([],subscriptions=0);self.gate.pubs={'team_11':a}
        with self.assertRaises(FakeTimingError):
            self.call(['team_11'],True)
        self.assertFalse(a.sent)

    def test_empty_set_is_noop(self):
        self.gate.pubs={}
        self.call([],False)
        self.assertEqual(self.ros.spins,0)

    def test_marker_is_present(self):
        self.assertIn('# v11 ROS DDS ACK retry (fail closed)',SOURCE.read_text())

if __name__=='__main__':unittest.main()
