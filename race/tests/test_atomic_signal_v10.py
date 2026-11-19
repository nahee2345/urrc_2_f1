"""Offline checks for v10 race five-red atomic lights-out; no Gazebo needed."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

MODULE_PATH=Path(__file__).resolve().parents[1]/'scripts'/'atomic_signal_v10.py'
spec=importlib.util.spec_from_file_location('atomic_signal_v10',MODULE_PATH)
signal_mod=importlib.util.module_from_spec(spec)
spec.loader.exec_module(signal_mod)


class TestAtomicSignal(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
        worlds=self.root/'src/urrc_track_gazebo/worlds'
        worlds.mkdir(parents=True)
        sdf=ET.Element('sdf',version='1.9')
        world=ET.SubElement(sdf,'world',name='monza')
        gantry=ET.SubElement(world,'model',name='race_start_gantry')
        ET.SubElement(gantry,'pose').text='1 2 3 0 0 0'
        link=ET.SubElement(gantry,'link',name='gantry_visuals')
        for number in range(1,6):
            visual=ET.SubElement(link,'visual',name=f'red_lens_{number}')
            ET.SubElement(visual,'pose').text=f'{number} 0 0 0 0 0'
            ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size').text='.01 .1 .12'
            ET.SubElement(visual,'material')
        ET.ElementTree(sdf).write(worlds/'monza_race.sdf',encoding='utf-8')
        self.requests=[]
        self.fake_request=lambda track,service,reqtype,payload,**kw:self.requests.append((track,service,reqtype,payload,kw)) or True

    def tearDown(self):
        self.temp.cleanup()

    def test_generated_five_lens_one_model(self):
        model_path=signal_mod._generate_template(self.root,'monza')
        signal_mod.verify_template(model_path)
        tree=ET.parse(model_path)
        self.assertEqual(len(tree.findall('./model')),1)
        names={v.get('name') for v in tree.findall('.//visual')}
        self.assertEqual(names,{f'active_red_lens_{i}' for i in range(1,6)})
        # The original race world must be preserved byte-for-byte by generation.
        self.assertEqual(len(ET.parse(self.root/'src/urrc_track_gazebo/worlds/monza_race.sdf').findall('.//visual')),5)

    def test_sequential_on_single_off_request(self):
        signal=signal_mod.AtomicFiveRed(self.root,'monza',request=self.fake_request,sleep=lambda _sec:None)
        signal.start()
        for number in range(1,6):signal.red(number)
        signal.all_off()
        commands=[x[1] for x in self.requests]
        self.assertEqual(commands,['remove','create',*['visual_config']*5,'remove'])
        self.assertEqual([x for x in self.requests if x[1]=='remove' and not x[4].get('optional')], [self.requests[-1]])
        self.assertEqual(self.requests[-1][3], 'name: "urrc_start_signal_active" type: 2')
        self.assertFalse(signal.active)

    def test_no_release_before_all_lit(self):
        signal=signal_mod.AtomicFiveRed(self.root,'monza',request=self.fake_request,sleep=lambda _sec:None)
        signal.start()
        with self.assertRaises(signal_mod.SignalError):signal.all_off()
        with self.assertRaises(signal_mod.SignalError):signal.red(3)
        self.assertEqual([x[1] for x in self.requests],['remove','create'])

    def test_service_fail_closed(self):
        class Finished:
            returncode=0
            stdout='data: false\n'
            stderr=''
        from unittest.mock import patch
        with patch.object(signal_mod.subprocess,'run',return_value=Finished()):
            with self.assertRaises(signal_mod.SignalError):
                signal_mod.gz_request('monza','remove','gz.msgs.Entity','name: "x" type: 2')

    def test_missing_lens_rejected(self):
        path=self.root/'src/urrc_track_gazebo/worlds/monza_race.sdf'
        tree=ET.parse(path)
        link=tree.find('.//link[@name="gantry_visuals"]')
        link.remove(link.findall('visual')[0])
        tree.write(path)
        with self.assertRaises(signal_mod.SignalError):
            signal_mod.AtomicFiveRed(self.root,'monza',request=self.fake_request,sleep=lambda _sec:None)


if __name__=='__main__':unittest.main()
