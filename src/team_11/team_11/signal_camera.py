"""Camera-only red-light start detector; emits Bool on /start_allowed_11.

Official competition mode also requires race/start/<id> true: this stage7
local trial only grants start after its own red-lamps-on then off detection.
"""
import time
import os
from pathlib import Path
import struct
import zlib
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from .signal_core import StartGate
from .red_lamps import count_red_lamps

class SignalNode(Node):
    def __init__(self):
        super().__init__("team11_signal_camera")
        self.gate=StartGate()
        self.zoom_gate=StartGate()
        self.wide_only=os.environ.get("TEAM11_SIGNAL_WIDE_ONLY","0")=="1"
        self.last_frame=-1.e10
        self.last_zoom_frame=-1.e10
        self.frames=0
        self.ready_sources=set()
        self.processed_at={"wide":-1.e10,"zoom":-1.e10}
        self.peak_lamps=0
        self.image_encoding="none"
        self.image_size="unknown"
        self.diag_dir=Path(os.environ["TEAM11_CAMERA_DIAG_DIR"]) if os.environ.get("TEAM11_CAMERA_DIAG_DIR") else None
        self.saved_before=False
        self.saved_candidate=False
        self.red_peak=0
        self.last_signal_capture=-1.e10
        self.signal_capture_count=0
        self.last_log=-1.e10
        self.prev_signal_counts={"wide":None,"zoom":None}
        self.armed_sources=set()
        self.start_source=None
        self.publisher=self.create_publisher(Bool,"/start_allowed_11",10)
        self.create_subscription(Image,"/camera_11",lambda msg:self.on_frame(msg,"wide"),qos_profile_sensor_data)
        if not self.wide_only:
            self.create_subscription(Image,"/camera_zoom_11",lambda msg:self.on_frame(msg,"zoom"),qos_profile_sensor_data)
        self.create_timer(.1,self.tick)
        self.get_logger().info("WAIT: require 4+ red lamps THEN all-off; initial dark is NOT start; mode="+("wide-only" if self.wide_only else "wide+zoom"))
    def on_frame(self,msg,source="wide"):
        now=time.monotonic()
        if source=="zoom":
            self.last_zoom_frame=now
        else:
            self.last_frame=now
        # Bound CPU use of two Python image detectors. Processing every camera
        # frame can starve rclpy callbacks and Gazebo sensor rendering.
        if now-self.processed_at[source]<.16:
            return
        self.processed_at[source]=now
        self.frames+=1
        self.image_encoding=msg.encoding
        self.image_size=f"{msg.width}x{msg.height}"
        try:
            lamps=count_red_lamps(msg.data,msg.width,msg.height,msg.step,msg.encoding)
            valid=True
        except (ValueError,TypeError) as exc:
            lamps=0
            valid=False
            if now-self.last_log>2.:
                self.get_logger().warning("Invalid camera image: "+str(exc))
        if valid and source not in self.ready_sources:
            self.ready_sources.add(source)
            self.get_logger().info(f'FRAME_READY source={source} encoding={msg.encoding} size={msg.width}x{msg.height}')
        self.peak_lamps=max(self.peak_lamps,lamps)
        red_pixels=self.sample_red(msg) if valid else 0
        self.red_peak=max(self.red_peak,red_pixels)
        if self.diag_dir and source=='wide' and not self.saved_before and valid:
            self.save_image(msg,'camera_before.png')
            self.saved_before=True
        if self.diag_dir and source=='wide' and not self.saved_candidate and red_pixels>5 and valid:
            self.save_image(msg,'camera_red_candidate.png')
            self.saved_candidate=True
        # Capture the ACTUAL official light sequence. Previous "red candidate"
        # frames were triggered by red crash barriers before lamps lit.
        marker=self.diag_dir/"signal_capture.active" if self.diag_dir else None
        if source=="wide" and marker and marker.is_file() and self.signal_capture_count<32 and now-self.last_signal_capture>=.30 and valid:
            self.save_image(msg,f"signal_{self.signal_capture_count:02d}.png")
            self.signal_capture_count+=1
            self.last_signal_capture=now
        before=self.gate.started or self.zoom_gate.started
        selected=self.zoom_gate if source=="zoom" else self.gate
        old_armed = selected.armed_at is not None
        selected.observe(lamps,now,valid)
        if not old_armed and selected.armed_at is not None:
            self.armed_sources.add(source)
            self.get_logger().info(f"ARMED source={source} count={lamps} frames={self.frames}")
        if old_armed and selected.armed_at is None and not selected.started:
            self.get_logger().warning(f"RESET source={source} count={lamps} frames={self.frames} reason=timeout_or_invalid")
        if (selected.armed_at is not None or lamps>=4 or self.prev_signal_counts[source]!=lamps):
            self.get_logger().info(
                f"SIGNAL source={source} count={lamps} lit_run={selected.lit_run}"
                f" dark_run={selected.dark_run} armed={selected.armed_at is not None}"
                f" started={selected.started}")
        self.prev_signal_counts[source]=lamps
        if (self.gate.started or self.zoom_gate.started) and not before:
            self.start_source=source
            self.get_logger().info("START: 4+ red lamps observed, now ALL OFF (source="+source+")")
        if now-self.last_log>1.:
            self.get_logger().info("camera red_lamps=%d armed=%s started=%s" % (
                lamps,selected.armed_at is not None,selected.started))
            self.get_logger().info(f"DIAG frames={self.frames} source={source} encoding={self.image_encoding} size={self.image_size} peak_lamps={self.peak_lamps} peak_red_sample={self.red_peak}")
            self.last_log=now
    @staticmethod
    def sample_red(msg):
        data=memoryview(msg.data)
        r_idx=0 if msg.encoding.lower()=="rgb8" else 2
        b_idx=2 if r_idx==0 else 0
        hits=0
        for y in range(0,int(msg.height*.85),8):
            for x in range(0,msg.width,8):
                i=y*msg.step+x*3
                r,g,b=data[i+r_idx],data[i+1],data[i+b_idx]
                if r>=65 and r-g>=20 and r>=1.45*max(1,g,b):
                    hits+=1
        return hits

    def save_image(self,msg,name):
        self.diag_dir.mkdir(parents=True,exist_ok=True)
        rows=[]
        data=memoryview(msg.data)
        for y in range(msg.height):
            row=bytes(data[y*msg.step:y*msg.step+msg.width*3])
            if msg.encoding.lower()=="bgr8":
                swapped=bytearray(len(row))
                for i in range(0,len(row),3):
                    swapped[i:i+3]=bytes((row[i+2],row[i+1],row[i]))
                row=bytes(swapped)
            rows.append(row)
        def chunk(kind,payload):
            return struct.pack("!I",len(payload))+kind+payload+struct.pack(
                "!I",zlib.crc32(kind+payload)&0xffffffff)
        image=b"".join(bytes([0])+row for row in rows)
        png=(b"\x89PNG\r\n\x1a\n"+
             chunk(b"IHDR",struct.pack("!2I5B",msg.width,msg.height,8,2,0,0,0))+
             chunk(b"IDAT",zlib.compress(image,3))+chunk(b"IEND",b""))
        dest=self.diag_dir/name
        dest.write_bytes(png)
        self.get_logger().info("Saved onboard camera: "+str(dest))

    def tick(self):
        # Even after a start, stop if camera stream dies.
        now=time.monotonic()
        wide_ready=self.gate.started and now-self.last_frame<=.65
        zoom_ready=self.zoom_gate.started and now-self.last_zoom_frame<=.65
        self.publisher.publish(Bool(data=wide_ready or zoom_ready))

def main(args=None):
    rclpy.init(args=args)
    n=SignalNode()
    try:rclpy.spin(n)
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__=="__main__":main()
