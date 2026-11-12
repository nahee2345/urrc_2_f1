import importlib.util
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

PATH=Path(__file__).parent/"tests"/"prepare_practice_world.py"
def load_generator():
    spec=importlib.util.spec_from_file_location("team11_prepare_practice",PATH)
    mod=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

class PracticeWorldTests(unittest.TestCase):
    def test_dynamic_signal_active_without_static_race_red(self):
        with tempfile.TemporaryDirectory() as temp:
            src=Path(temp)/"official"/"worlds"
            src.mkdir(parents=True)
            (src/"monza_race.sdf").write_text(
                '<sdf version="1.9"><world name="monza">'
                '<plugin filename="gz-sim-sensors-system" name="Sensors"/>'
                '<scene><shadows>true</shadows></scene>'
                '<model name="track"><link name="visuals">'
                '<sensor name="spectator" type="camera"/>'
                '</link></model></world></sdf>')
            (src/"monza_signal_active.sdf").write_text(
                '<sdf version="1.9"><model name="urrc_start_signal_active"/></sdf>')
            (src/"monza_grid.gui.config").write_text(
                '<window><width>640</width></window>'
                '<plugin filename="CameraTracking" name="Camera tracking"/>')
            out=Path(temp)/"practice"
            load_generator().prepare("monza",src.parent,out)
            w=ET.parse(out/"monza_practice.sdf").getroot()
            self.assertEqual(len(w.findall(".//sensor")),0)
            self.assertEqual(len(w.findall(".//plugin[@name='Sensors']")),1)
            self.assertTrue((out/"monza_practice.gui.config").is_file())
    def test_remove_spectator_only_keep_signals_and_follow(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            src=folder/"official"/"worlds"
            src.mkdir(parents=True)
            world='''<sdf version="1.9"><world name="monza">
             <plugin filename="gz-sim-physics-system" name="Physics"/>
             <plugin filename="spectator" name="spectator_plugin"/>
             <scene><shadows>true</shadows></scene>
             <light name="sun"><cast_shadows>true</cast_shadows></light>
             <light name="race_red_1"/><light name="race_red_2"/><light name="race_red_3"/><light name="race_red_4"/><light name="race_red_5"/>
             <model name="stand"><link name="frame">
             <sensor name="spectator_cam" type="camera"/>
             </link></model></world></sdf>'''
            gui='''<?xml version="1.0"?>
             <window><width>640</width></window>
             <plugin filename="MinimalScene" name="3D View"/>
             <plugin filename="CameraTracking" name="Camera tracking"/>
             <plugin filename="WorldStats" name="World statistics"/>'''
            (src/"monza_race.sdf").write_text(world)
            (src/"monza_grid.gui.config").write_text(gui)
            out=folder/"practice"
            load_generator().prepare("monza",src.parent,out)
            w=ET.parse(out/"monza_practice.sdf").getroot()
            self.assertEqual(len(w.findall(".//sensor")),0)
            self.assertEqual(len(w.findall(".//light[@name='race_red_1']")),1)
            self.assertEqual(len(w.findall(".//light[@name='race_red_5']")),1)
            self.assertEqual(w.find(".//scene/shadows").text,"false")
            self.assertEqual(len(w.findall(".//plugin[@name='spectator_plugin']")),0)
            g=(out/"monza_practice.gui.config").read_text()
            self.assertIn("CameraTracking",g)
            self.assertNotIn("WorldStats",g)
            self.assertIn("<window>",g)

if __name__=="__main__":unittest.main()
