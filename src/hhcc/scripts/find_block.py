#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import geometry_msgs.msg
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


def find_max_rect_of_target_color(image, white=False):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]
    mask = np.zeros(h.shape, dtype=np.uint8)
    if white:
        mask[(s < 32) & (v > 192)] = 255
    else:
        mask[(((h < 20) | (h > 235)) | ((h > 32) & (h < 60)) | ((h > 140) & (h < 185))) 
             & (s > 128) & (v > 128)] = 255
    image, contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = []
    for contour in contours:
        approx = cv2.convexHull(contour)
        rect = cv2.boundingRect(approx)
        rects.append(np.array(rect))
    if len(rects) != 0:
        rect = min(rects, key=(lambda x: x[2] * x[3]))
        return rect
    else:
        return None


class FindObject:
    u"""find red object and publish tf"""

    def __init__(self):
        u"""Setup node"""
        self._bridge = CvBridge()
        self._rgb_sub = message_filters.Subscriber(
            "rgb_image", Image)
        self._depth_sub = message_filters.Subscriber(
            "depth_image", Image)
        self._debug_image_pub = rospy.Publisher(
            "debug_image", Image,
            queue_size=10)
        camera_info = rospy.wait_for_message(
            "depth_camera_info", CameraInfo)
        self._invK = np.linalg.inv(np.array(camera_info.K).reshape(3, 3))
        self._ts = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub], 30, 0.5)
        self._ts.registerCallback(self.callback)
        self._br = tf2_ros.TransformBroadcaster()

    def callback(self, rgb, depth):
        try:
            depth_image = self._bridge.imgmsg_to_cv2(depth, 'passthrough')
            rgb_image = self._bridge.imgmsg_to_cv2(rgb, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        rect = find_max_rect_of_target_color(rgb_image)
        if rect is None:
            rect = find_max_rect_of_target_color(rgb_image, white=True)           
        if rect is not None:
            cv2.rectangle(rgb_image,
                          tuple(rect[0:2]),
                          tuple(rect[0:2] + rect[2:4]),
                          (0, 0, 255),
                          thickness=2)
            self._debug_image_pub.publish(
                self._bridge.cv2_to_imgmsg(
                    rgb_image, encoding=rgb.encoding))
            depth_array = np.array(depth_image, dtype=np.float32)
            u = int(rect[0] + 0.5 * rect[2])
            v = int(rect[1] + 0.5 * rect[3])
            z = depth_array[v][u] * 1e-3
            # no valid point
            if z == 0.0:
                rospy.loginfo('no valid depth in region.')
                return
            image_point = np.array([u, v, 1])
            object_point = np.dot(self._invK, image_point) * z
            t = geometry_msgs.msg.TransformStamped()
            t.header = depth.header
            t.child_frame_id = 'object'
            t.transform.translation.x = object_point[0]
            t.transform.translation.y = object_point[1]
            t.transform.translation.z = object_point[2]
            t.transform.rotation.z = np.sin(-np.pi / 4)
            t.transform.rotation.w = np.cos(-np.pi / 4)
            self._br.sendTransform([t])
        else:
            rospy.loginfo('no valid regions.')
            return


if __name__ == "__main__":
    rospy.init_node('find')
    find = FindObject()
    rospy.spin()
