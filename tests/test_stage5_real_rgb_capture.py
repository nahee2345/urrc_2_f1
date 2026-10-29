"""실제 RGB 수집 프로그램의 파서 검증. 합성 데이터는 프로토콜 단위 시험만 사용."""
import unittest
from scripts.stage5_capture_actual_rgb import parse_gz_camera_image,inspect_rgb


def synthetic_proto(w=160,h=120):
    pixels=bytes((39,42,46))*(w*h)
    # 실제 gz topic -e protobuf TextFormat의 C-style escape string 재현.
    quoted='"'+''.join(chr(v) if 32<=v<127 and v not in (34,92) else f"\\{v:03o}" for v in pixels)+'"'
    return f'header {{ stamp {{ sec: 1 }} }}\nwidth: {w}\nheight: {h}\nstep: {w*3}\ndata: {quoted}\n'


class ActualGazeboImageDecoder(unittest.TestCase):
    def test_gazebo_text_bytes_decode(self):
        w,h,data=parse_gz_camera_image(synthetic_proto())
        self.assertEqual((w,h),(160,120))
        self.assertEqual(len(data),160*120*3)
        self.assertEqual(data[:3],b"'*.")
    def test_alternate_property_order(self):
        raw=synthetic_proto().replace("width: 160\nheight: 120","height: 120\nwidth: 160")
        self.assertEqual(parse_gz_camera_image(raw)[:2],(160,120))
    def test_garbled_data_not_silently_claimed(self):
        with self.assertRaises(ValueError):
            parse_gz_camera_image("width: 160\nheight: 120\ndata: \"not a full image\"")
    def test_missing_data_rejected(self):
        with self.assertRaises(ValueError):
            parse_gz_camera_image("width: 160\nheight: 120")
    def test_ground_truth_is_not_read_by_inspector(self):
        w,h,pixels=parse_gz_camera_image(synthetic_proto())
        result=inspect_rgb(w,h,pixels)
        self.assertEqual(result["actual_gazebo_rgb_bytes"],57600)
        self.assertIn("lane_valid",result)
        self.assertIn("asphalt_fraction_by_image_row",result)
        self.assertNotIn("ground_truth_pose",result)
        self.assertNotIn("centerline",result)


if __name__=="__main__":unittest.main()
