#!/usr/bin/env python3

import rclpy
import message_filters
from rclpy.node import Node
from PoseDetector import PoseDetector
import json
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from pose_detector_interface.msg import DetectedObject, DetectedObjects
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation
import functools
import numpy as np
import cv_bridge

class PoseDetectorNode(Node):
    def __init__(self, detector):
        super().__init__('pose_detector')
        self.Detector = detector
        self.ColorSub = message_filters.Subscriber(self, Image, '/color')
        self.DepthSub = message_filters.Subscriber(self, Image, '/depth')
        self.CameraInfoSub = message_filters.Subscriber(self, CameraInfo, '/camera_info')

        self.TS = message_filters.TimeSynchronizer([self.ColorSub, self.DepthSub, self.CameraInfoSub], 1)
        self.TS.registerCallback(self.detect_poses)

        self.Publisher = self.create_publisher(DetectedObjects, '/detected_objects', rclpy.qos.HistoryPolicy(1))

    def detect_poses(self, color_msg, depth_msg, camera_info):
        
        K = camera_info.K
        bridge = cv_bridge.CvBridge()

        K = np.array(K).reshape(3,3).T
        detected_objects = self.Detector.detect(
            bridge.imgmsg_to_cv2(color_msg).astype(np.uint8),
            bridge.imgmsg_to_cv2(depth_msg).astype(np.uint8),
            K
        )

        objects_msg = []
        for o in detected_objects:
            msg = DetectedObject()
            msg.mesh_name = String(o.Name)
            msg.mesh_file = String(o.MeshFile)
            bbox = np.array(o.BoundingBox, dtype = np.int)
            mask = o.Mask[bbox[1]:bbox[3], bbox[0]:bbox[2]]
            mask_msg = bridge.cv2_to_imgmsg(mask.astype(np.uint8))
            msg.mask = mask_msg
            msg.mask_offset_x = Int32(bbox[0])
            msg.mask_offset_y = Int32(bbox[1])
            R = Rotation.from_matrix(o.R)
            pose = Pose()
            pose.position.x = o.o[0]
            pose.position.y = o.o[1]
            pose.position.z = o.o[2]
            q = R.as_quat()
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            msg.pose = pose

            objects_msg.append(msg)
        response = DetectedObjects()
        response.detected_objects = objects_msg
        self.Publisher.publish(response)


def main():
    rclpy.init()
    import sys
    config_fp = sys.argv[1]
    with open(config_fp) as fin:
        config = json.load(fin)
    detector = PoseDetector(**config)

    pose_detector = PoseDetectorNode(detector)
    rclpy.spin(pose_detector)
    
if __name__ == '__main__':
    main()