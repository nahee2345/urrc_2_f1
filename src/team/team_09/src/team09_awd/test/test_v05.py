import math
import os
from pathlib import Path
import runpy
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from team09_awd.core import MAX_CURVATURE,WHEELBASE,KINGPIN_WIDTH,MAX_WHEEL_ANGLE

ROOT=Path(__file__).resolve().parents[1]
class PackagingAndSteering(unittest.TestCase):
    def test_packaging_excludes_compiled_directory(self):
        # This directory exists after compileall / colcon imports the launch script.
        cache=ROOT/'launch/__pycache__';cache.mkdir(exist_ok=True)
        old=os.getcwd()
        try:
            os.chdir(ROOT)
            with patch('setuptools.setup') as setup:runpy.run_path('setup.py')
            files=[f for _,fs in setup.call_args.kwargs['data_files'] for f in fs]
            self.assertTrue(files);self.assertTrue(all(Path(f).is_file() for f in files))
            self.assertFalse(any('__pycache__' in f for f in files))
            self.assertIn('launch/experiment.launch.py',files)
        finally:os.chdir(old)
    def test_inner_wheel_command_bound(self):
        for sign in (-1,1):
            k=sign*MAX_CURVATURE
            for side in (-1,1):
                angle=math.atan(WHEELBASE*k/(1-side*KINGPIN_WIDTH*.5*k))
                self.assertLessEqual(abs(angle),MAX_WHEEL_ANGLE+1e-10)
    def test_model_joints_match_bound(self):
        tree=ET.parse(ROOT/'models/model.sdf')
        for name in ('front_left_steer','front_right_steer'):
            limit=tree.find(f'.//joint[@name="{name}"]/axis/limit')
            self.assertAlmostEqual(float(limit.findtext('upper')),MAX_WHEEL_ANGLE)
            self.assertAlmostEqual(float(limit.findtext('lower')),-MAX_WHEEL_ANGLE)
    def test_plugin_radius_limit_matches_planner(self):
        tree=ET.parse(ROOT/'models/model.sdf')
        value=float(tree.findtext('.//plugin/steering_limit'))
        self.assertAlmostEqual(math.sin(value)/WHEELBASE,MAX_CURVATURE,places=9)
