import os
import math
import time
import numpy as np
import rclpy

from pathlib import Path
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

from monaco_center_drive import Driver, D

class RacePlus(Driver):

    def __init__(self):
        super().__init__()

        self.mode = os.getenv("TEAM05_MODE", "practice")
        self.started = self.mode == "practice"

        self.tracks = []
        self.track_time = 0.
        self.camera_count = 0
        self.camera_time = 0.
        self.camera_objects = []
        self.detector = None

        self.create_subscription(
            Bool,
            "/race/start/team_05",
            self.start_signal,
            10
        )

        self.create_subscription(
            Image,
            "/team_05/camera/image_raw",
            self.camera,
            qos_profile_sensor_data
        )

        weights = os.getenv("TEAM05_CAMERA_MODEL", "")

        if weights and Path(weights).is_file():
            try:
                from ultralytics import YOLO
                self.detector = YOLO(weights)
                print("CAMERA DETECTOR READY", flush=True)
            except Exception as e:
                print("CAMERA DETECTOR UNAVAILABLE", e)

        print("RACE PLUS READY", flush=True)
        print("MODE:", self.mode, flush=True)

    def start_signal(self, msg):
        self.started = bool(msg.data)

        if not self.started:
            self.send(0, 0)

    def camera(self, msg):
        self.camera_count += 1

        now = time.monotonic()

        if self.detector is None:
            return

        if now - self.camera_time < 1.0:
            return

        self.camera_time = now

        try:
            if msg.encoding.lower() not in ("rgb8", "bgr8"):
                return

            a = np.frombuffer(
                msg.data, dtype=np.uint8
            ).reshape(msg.height, msg.step)

            image = a[:, :msg.width*3].reshape(
                msg.height, msg.width, 3)

            if msg.encoding.lower() == "rgb8":
                image = image[:, :, ::-1].copy()

            result = self.detector.predict(
                image, verbose=False, conf=0.35
            )[0]

            objects = []

            for box in result.boxes:
                name = result.names[int(box.cls.item())]

                if name in (
                    "car", "truck", "bus", "motorcycle"
                ):
                    objects.append(name)

            self.camera_objects = objects

        except Exception:
            self.camera_objects = []

    def scan(self, msg):
        super().scan(msg)

        if self.points is None or self.pos is None:
            return

        now = time.monotonic()

        if now - self.pose_time > .35:
            return

        p = self.points

        if len(p) < 3:
            self.tracks = []
            return

        # Group adjacent LiDAR returns.
        gaps = np.linalg.norm(
            np.diff(p, axis=0), axis=1
        )

        groups = np.split(
            p, np.where(gaps > .16)[0]+1
        )

        c = math.cos(self.heading)
        s = math.sin(self.heading)

        rotation = np.array([
            [c, -s],
            [s, c]
        ])

        detections = []

        for g in groups:

            if len(g) < 3:
                continue

            size = float(np.linalg.norm(
                np.ptp(g, axis=0)
            ))

            # Compact obstacles are vehicle candidates.
            # Long wall segments are not tracked as cars.
            if not .045 <= size <= .70:
                continue

            center = np.mean(g, axis=0)

            if np.linalg.norm(center) > 3.0:
                continue

            world = self.pos + rotation @ center

            _, lateral, _ = self.project(world)

            # Avoid tracking most roadside wall returns.
            if lateral > .55:
                continue

            detections.append((
                world,
                min(.22, max(.09, size*.5))
            ))

        updated = []
        used = set()

        for position, radius in detections:

            matches = []

            for i, old in enumerate(self.tracks):

                if i in used:
                    continue

                dt = now-old["time"]

                if not .025 <= dt <= .65:
                    continue

                predicted = (
                    old["pos"] + old["vel"]*dt
                )

                error = np.linalg.norm(
                    predicted-position
                )

                if error < .25 + dt:
                    matches.append((error, i, dt))

            velocity = np.zeros(2)
            hits = 1

            if matches:
                _, i, dt = min(matches)
                old = self.tracks[i]
                used.add(i)

                raw = (position-old["pos"])/dt

                speed = np.linalg.norm(raw)

                if speed > 2.5:
                    raw *= 2.5/speed

                velocity = (
                    .60*old["vel"] + .40*raw
                )

                hits = old["hits"]+1

            updated.append({
                "pos": position,
                "vel": velocity,
                "radius": radius,
                "hits": hits,
                "time": now
            })

        self.tracks = updated
        self.track_time = now

    def clearance(self, x, y, a):

        # Keep original wall and obstacle protection.
        free = super().clearance(x, y, a)

        if self.pos is None:
            return free

        if time.monotonic()-self.track_time > .5:
            return free

        c = math.cos(self.heading)
        s = math.sin(self.heading)

        # Future arrival times along candidate path.
        t = np.minimum(
            D / max(self.speed, .35),
            1.2
        )

        for obj in self.tracks:

            if obj["hits"] < 3:
                continue

            if np.linalg.norm(obj["vel"]) < .12:
                continue

            # Predict moving obstacle positions.
            future = (
                obj["pos"][None, :]
                + t[:, None]*obj["vel"][None, :]
            )

            relative = future-self.pos

            ox = (
                c*relative[:, 0]
                + s*relative[:, 1]
            )

            oy = (
                -s*relative[:, 0]
                + c*relative[:, 1]
            )

            dx = ox-x
            dy = oy-y

            px = np.cos(a)*dx+np.sin(a)*dy
            py = -np.sin(a)*dx+np.cos(a)*dy

            radius = obj["radius"]

            hit = (
                (np.abs(px) < .20+radius)
                & (np.abs(py) < .128+radius)
            )

            ids = np.flatnonzero(hit)

            if len(ids):
                free = min(
                    free,
                    max(0., float(D[ids[0]])-.05)
                )

        return free

    def report(self, state, extra=""):

        moving = sum(
            o["hits"] >= 3
            and np.linalg.norm(o["vel"]) >= .12
            for o in self.tracks
        )

        super().report(
            state,
            extra
            + " moving=" + str(moving)
            + " camera=" + str(self.camera_count)
            + " visual=" + str(len(self.camera_objects))
        )

    def drive(self):

        if not self.started:
            self.send(0, 0)
            self.report("WAIT OFFICIAL START")
            return

        super().drive()


def main():
    rclpy.init()
    node = RacePlus()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send(0, 0)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
