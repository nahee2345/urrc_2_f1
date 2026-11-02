#!/usr/bin/env python3
"""Stage 2 ROS 2 Jazzy bridge configuration for per-AI sensor topics.

Source topics come solely from generated manifest.json, no magic substitutions.
3D LiDAR LaserScan does NOT provide a proper PointCloud2 by itself.
An optional /points bridge is included ONLY when --include-points is requested.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path


def yaml_bridge(manifest,include_points=False):
    topics=manifest["topics"]
    lines=["# Generated URRC sensor bridges: Gazebo -> ROS ONLY."]
    for name,topic in topics.items():
        gz_topic=topic["gazebo"]
        if not gz_topic.startswith(f"/model/{manifest['vehicle']}/"):
            raise ValueError("sensor must use model-isolated topic")
        lines += [
            "- ros_topic_name: "+gz_topic,
            "  gz_topic_name: "+gz_topic,
            "  ros_type_name: "+topic["ros_type"],
            "  gz_type_name: "+topic["gz_type"],
            "  direction: GZ_TO_ROS",
        ]
        if include_points and name=="lidar3d":
            candidate=topic["pointcloud_topic_candidate"]
            lines += [
                "# Verify this optional topic exists using: gz topic -l",
                "- ros_topic_name: "+candidate,
                "  gz_topic_name: "+candidate,
                "  ros_type_name: sensor_msgs/msg/PointCloud2",
                "  gz_type_name: gz.msgs.PointCloudPacked",
                "  direction: GZ_TO_ROS",
            ]
    return "\n".join(lines)+"\n"


def main():
    p=argparse.ArgumentParser()
    p.add_argument("--manifest",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--include-points",action="store_true",
                   help="Only after verifying a Gazebo PointCloudPacked /points topic exists")
    args=p.parse_args()
    data=json.loads(args.manifest.read_text(encoding="utf-8"))
    data_out=yaml_bridge(data,args.include_points)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(data_out,encoding="utf-8")
    print(f"WROTE {args.output}")
    print("Run: ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="+str(args.output.absolute()))

if __name__=="__main__":main()
