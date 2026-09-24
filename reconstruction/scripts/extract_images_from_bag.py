#!/usr/bin/env python3
"""Extract sensor_msgs/Image frames from a ROS 2 bag.

Usage:
  python3 extract_images_from_bag.py BAG_DIR IMAGE_TOPIC OUTPUT_DIR

Requires: rosbag2_py, cv_bridge, OpenCV, sensor_msgs.
"""
import sys
from pathlib import Path
import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def main():
    if len(sys.argv) != 4:
        raise SystemExit("Usage: extract_images_from_bag.py BAG_DIR IMAGE_TOPIC OUTPUT_DIR")
    bag, image_topic, out = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', '')
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if image_topic not in topics:
        raise SystemExit(f"{image_topic} not in bag. Available: {list(topics)}")
    msg_type = get_message(topics[image_topic])
    bridge = CvBridge()
    i = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != image_topic:
            continue
        msg = deserialize_message(data, msg_type)
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imwrite(str(out/f"frame_{i:06d}.png"), frame)
        i += 1
    print(f"Extracted {i} images to {out}")

if __name__ == '__main__':
    main()
