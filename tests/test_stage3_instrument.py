"""3단계 원본 맵 복사 및 접촉 감시/ROS 브리지 계약 검사."""
import tempfile
from pathlib import Path
from dataclasses import replace
import unittest
import xml.etree.ElementTree as ET
from vehicle_generator.model import Vehicle
from vehicle_generator.sensors import build
from stage3.instrument import instrument_model
from scripts.stage3_bridge import mappings


class MonzaInstrumentTests(unittest.TestCase):
    def test_all_ten_sensor_model_colliders_covered(self):
        for drive,sensors in (("rwd",["rgb"]),("awd",["rgb","depth","lidar3d"]),
                              ("awd",["stereo","lidar3d"])):
            with self.subTest(sensors=sensors):
                v=replace(Vehicle(),name="urrc_test_03",drive=drive)
                xml,_=build(v,sensors)
                out,topics=instrument_model(xml,v.name)
                m=ET.fromstring(out).find("model")
                self.assertIsNotNone(m.find("plugin[@name='gz::sim::systems::PosePublisher']"))
                self.assertEqual(len(topics),len(m.findall(".//link/collision")))
                self.assertEqual(len(topics),len(m.findall(".//link/sensor[@type='contact']")))
                self.assertGreater(len(topics),5)
                self.assertEqual(len(set(topics)),len(topics))
                for sensor in m.findall(".//link/sensor[@type='contact']"):
                    self.assertIsNotNone(sensor.findtext("contact/collision"))
                    self.assertIn(sensor.findtext("topic"),topics)

    def test_bridge_includes_image_pose_and_all_contacts(self):
        model="urrc_ai_01"
        _,ts=instrument_model(build(Vehicle(name=model),["rgb"])[0],model)
        yaml=mappings(model,"monza",ts)
        self.assertEqual(yaml.count("direction: GZ_TO_ROS"),len(ts)+3)
        self.assertIn("ros_gz_interfaces/msg/Contacts",yaml)
        self.assertIn("sensor_msgs/msg/Image",yaml)
        self.assertIn("tf2_msgs/msg/TFMessage",yaml)
        self.assertIn("rosgraph_msgs/msg/Clock",yaml)

    def test_reject_duplicate_topic(self):
        with self.assertRaises(ValueError):
            mappings("x","monza",["/clock"])

    def test_user_original_source_copy_not_mutated(self):
        # 가상 upstream fixture로 원본 보존과 별도 Contact 월드 생성 확인
        import json
        from stage3.instrument import make_monza_copy
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            p=root/"src/urrc_track_gazebo/worlds";p.mkdir(parents=True)
            d=root/"src/urrc_track_gazebo/tracks/processed";d.mkdir(parents=True)
            source='<sdf version="1.9"><world name="monza"><gravity>0 0 -9.81</gravity></world></sdf>'
            (p/"monza_race.sdf").write_text(source)
            (d/"monza.json").write_text(json.dumps(
                {"grid":[{"x":0.1,"y":-2.0,"road_z":1.2,"yaw":1.47}]}))
            v=Vehicle(name="urrc_test_car")
            car,t=instrument_model(build(v,["rgb"])[0],v.name)
            output=make_monza_copy(root,car,v.name)
            self.assertEqual((p/"monza_race.sdf").read_text(),source)
            world=ET.fromstring(output).find("world")
            self.assertIsNotNone(world.find("plugin[@name='gz::sim::systems::Contact']"))
            self.assertEqual(len(world.findall("model")),1)
            self.assertAlmostEqual(float(world.findtext("model/pose").split()[2]),1.24)


if __name__=="__main__":unittest.main()
