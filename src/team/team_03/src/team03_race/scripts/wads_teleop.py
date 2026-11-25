#!/usr/bin/env python3
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class WadsTeleop(Node):
    def __init__(self):
        super().__init__("wads_teleop")
        self.speed_topic = self.declare_parameter("speed_topic", "/team03/speed_03").value
        self.steering_topic = self.declare_parameter("steering_topic", "/team03/steering_03").value

        self.speed_pub = self.create_publisher(Float32, self.speed_topic, 10)
        self.steering_pub = self.create_publisher(Float32, self.steering_topic, 10)

    def publish_cmd(self, speed, steer):
        s_msg = Float32(data=float(speed))
        st_msg = Float32(data=float(steer))
        self.speed_pub.publish(s_msg)
        self.steering_pub.publish(st_msg)

def setup_gui(node):
    root = tk.Tk()
    root.title("URRC WADS Control")
    root.geometry("320x180")

    info = "W: 전진 / S: 후진\nA: 좌회전 / D: 우회전\nSpace: 정지 (영문 입력 상태 필수)"
    label = tk.Label(root, text=info, font=("Arial", 11), justify="center")
    label.pack(expand=True)

    keys = set()

    def update():
        speed = 0.0
        steer = 0.0
        if 'w' in keys: speed += 1.0
        if 's' in keys: speed -= 0.6
        if 'a' in keys: steer += 0.4
        if 'd' in keys: steer -= 0.4
        if 'space' in keys: speed = 0.0; steer = 0.0
        node.publish_cmd(speed, steer)
        root.after(50, update)

    def on_press(e): keys.add(e.keysym.lower())
    def on_release(e): keys.discard(e.keysym.lower())

    root.bind("<KeyPress>", on_press)
    root.bind("<KeyRelease>", on_release)
    root.after(50, update)
    return root

def main():
    rclpy.init()
    node = WadsTeleop()
    root = setup_gui(node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    