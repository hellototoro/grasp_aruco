#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import json

class RealsenseSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.color_image = None
        self.depth_image = None
        self.point_cloud = None

        rospy.init_node('realsense_subscriber', anonymous=True)

        rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except CvBridgeError as e:
            rospy.logerr(e)

    def point_cloud_callback(self, msg):
        try:
            self.point_cloud = msg
        except CvBridgeError as e:
            rospy.logerr(e)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def save_point_cloud(self, point_cloud, filename):
        pc = pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)
        pcl_data = np.array(list(pc), dtype=np.float64)
        np.save(filename, pcl_data)

    def save_camera_info(self, camera_info, filename):
        camera_info_dict = {
            'width': camera_info.width,
            'height': camera_info.height,
            'K': camera_info.K,
            'D': camera_info.D,
            'R': camera_info.R,
            'P': camera_info.P,
            'distortion_model': camera_info.distortion_model
        }
        with open(filename, 'w') as f:
            json.dump(camera_info_dict, f, indent=4)

    def display_images(self):
        save_dir = './data2'
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        while not rospy.is_shutdown():
            if self.color_image is not None and self.depth_image is not None and self.point_cloud is not None:
                aligned_depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
                cv2.imshow("Aligned Depth colormap", aligned_depth_colormap)
                cv2.imshow("Aligned Depth Image", self.depth_image)
                cv2.imshow("Color Image", self.color_image)

                cv2.imwrite(save_dir + '/depth.png', self.depth_image)
                cv2.imwrite(save_dir + '/color.png', self.color_image)
                
                point_cloud_path = os.path.join(save_dir, 'point_cloud.npy')
                self.save_point_cloud(self.point_cloud, point_cloud_path)
                rospy.loginfo(f"Point cloud saved to {point_cloud_path}")

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    realsense_subscriber = RealsenseSubscriber()
    realsense_subscriber.display_images()
