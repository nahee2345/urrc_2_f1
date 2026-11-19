#!/usr/bin/env python3
"""ROS-free regression tests for manual spawn and DNF mailbox and source policy."""
from __future__ import annotations
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

here=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(here))
from operator_v26 import OperatorSession, command
from operator_v26_runtime import adapt_run


class MailboxTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        (self.root/'race/results/runs/r1').mkdir(parents=True)
        (self.root/'race').mkdir(exist_ok=True)
        (self.root/'race/v24_profile.json').write_text(json.dumps({'mode':'official'}))
        self.session=OperatorSession(self.root,'semi_a','monza',
              [{'entry_id':'team_01'},{'entry_id':'team_02'}],
              self.root/'race/results/runs/r1')
        self.addCleanup(self.session.close)

    def test_operator_spawn_only_after_request(self):
        self.assertEqual(self.session.commands('team_01',{'spawn'}),[])
        self.assertEqual(command(self.root,'spawn','team_01',''),0)
        msg=self.session.consume('team_01',{'spawn'})
        self.assertEqual(msg['action'],'spawn')
        self.assertIsNone(self.session.consume('team_01',{'spawn'}))

    def test_manual_dnf_only_team_and_reason(self):
        command(self.root,'dnf','team_02','차량 충돌 후 주행 불가')
        self.assertIsNone(self.session.consume('team_01',{'dnf'}))
        self.assertEqual(self.session.requested_dnf('team_02'),'차량 충돌 후 주행 불가')
        self.assertIsNone(self.session.requested_dnf('team_02'))

    def test_dnf_priority_to_avoid_unwanted_spawn(self):
        command(self.root,'spawn','team_01','')
        command(self.root,'dnf','team_01','출발 전 주행 불가')
        msg=self.session.consume('team_01',{'spawn','dnf'})
        self.assertEqual(msg['action'],'dnf')

    def test_reject_unregistered_team(self):
        with self.assertRaisesRegex(ValueError,'현재 경기 출전팀'):
            command(self.root,'dnf','team_11','')

    def test_reject_after_finished(self):
        self.session.status('team_01','FINISHED')
        with self.assertRaisesRegex(ValueError,'이미 완주'):
            command(self.root,'dnf','team_01','')

    def test_old_run_command_not_reused(self):
        command(self.root,'spawn','team_02','')
        self.session.close()
        fresh=OperatorSession(self.root,'semi_b','baku',
                [{'entry_id':'team_02'}],self.root/'race/results/runs/r2')
        try:self.assertEqual(fresh.commands('team_02',{'spawn'}),[])
        finally:fresh.close()

    def test_practice_rejected(self):
        (self.root/'race/v24_profile.json').write_text(json.dumps({'mode':'practice'}))
        with self.assertRaisesRegex(ValueError,'공식 대회'):
            command(self.root,'spawn','team_01','')

class SourcePolicyTests(unittest.TestCase):
    def test_injected_policy_strings(self):
        fixture=Path(os.environ.get('URRC_V26_MANAGER_FIXTURE',''))
        if not fixture.is_file():self.skipTest('Upstream runner source fixture not supplied')
        import ast
        source=fixture.read_text(encoding='utf-8')
        manager=ast.parse(source)
        method=next(n for n in manager.body if isinstance(n,ast.FunctionDef) and n.name=='run_mode')
        source=ast.get_source_segment(source,method)
        result=adapt_run(source)
        ast.parse(result)
        self.assertIn('operator_session.wait_spawn',result)
        self.assertIn('operator_session.requested_dnf',result)
        self.assertIn('monitor.team_faults',result)
        self.assertNotIn('monitor.health([e',result)
        self.assertNotIn("reason=monitor.failure_reason(eid,'제한시간 초과')",result)
        self.assertNotIn("reason=monitor.failure_reason(eid,'알고리즘 종료')",result)
        self.assertIn('unadjudicated=unadjudicated',result)
        self.assertIn('monitor.wait_lidar',result)
        self.assertIn('monitor.watch_camera',result)

if __name__=='__main__':unittest.main()
