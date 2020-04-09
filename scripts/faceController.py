#!/usr/bin/env python

import os
import typing
import cv2
import numpy as np
import cv_bridge
import rospy
import actionlib
from sensor_msgs.msg import (
    Image,
    CompressedImage
)
from Face.Face import Face
from tbd_ros_msgs.msg import (
    faceAnimationResult,
    faceAnimationAction,
    faceAnimationGoal
)
import actionlib 
import yaml


class FaceServer:

    _animation_server: actionlib.SimpleActionServer
    _face: Face

    def __init__(self, topic, run_local=False, width=1280, height=720):

        self._pub_topic_name = topic
        self._face = Face(run_local, width, height)
        self._img_pub = rospy.Publisher(
            self._pub_topic_name, Image, latch=True, queue_size=1)
        # We use our own compression topic instead of ImageTransport because ImageTransport doesn't have a python API.
        self._compressed_img_pub = rospy.Publisher(
            self._pub_topic_name + '/compressed', CompressedImage, latch=True, queue_size=1)

        # To take advantage of stuff like progress, we would need to implement our own action server.
        self._animation_server = actionlib.SimpleActionServer("animation", faceAnimationAction, execute_cb=self._animation_cb, auto_start=False)
        self._animation_server.register_preempt_callback(self._animation_preempted_cb)
        self._animation_server.start()

    def _animation_preempted_cb(self):
        rospy.loginfo("Pre-empting face animation ...")
        self._face.stop_animation()

    def _animation_cb(self, goal: faceAnimationGoal):
        data = yaml.safe_load(str(goal))
        self._face.run_animation(data)
        result = faceAnimationResult()
        result.success = self._face.wait_for_animation()
        if self._animation_server.is_preempt_requested:
            self._animation_server.set_preempted(result)
        else:
            self._animation_server.set_succeeded(result)

    def _send_image(self, img_data):
        # send uncompressed
        # if (self._img_pub.get_num_connections() > 0):
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img_data, encoding="bgr8")
        self._img_pub.publish(msg)
        # send compressed
        if (self._compressed_img_pub.get_num_connections() > 0):
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]
            result, data = cv2.imencode('.jpg', img_data, encode_param)
            if result:
                compressed_msg.data = data.flatten().tolist()
                self._compressed_img_pub.publish(compressed_msg)

    def spin(self):
        refresh_rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            # Update the face
            self._face.update_once()
            # send the image to baxter's face
            self._send_image(self._face.get_screen_as_bmp())
            # refresh the screen
            refresh_rate.sleep()

def main():
    # initialize node
    rospy.init_node('baxter_face_controller')

    # get rosparam
    pub_topic = rospy.get_param("~topic_name", 'face_image')
    run_local = rospy.get_param("~run_local", True)

    server = FaceServer(pub_topic, run_local)
    # loop forever
    server.spin()


if __name__ == '__main__':
    main()
