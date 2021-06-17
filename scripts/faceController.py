#!/usr/bin/env python

import cv2
import cv_bridge
import rospy
import actionlib
from sensor_msgs.msg import (
    Image,
    CompressedImage
)
from tbd_screen_face.Face import Face
from tbd_ros_msgs.msg import (
    faceAnimationResult,
    faceAnimationAction,
    faceAnimationGoal
)
import yaml
import dynamic_reconfigure.server
from tbd_screen_face.cfg import FaceConfig


class FaceServer:

    _animation_server: actionlib.SimpleActionServer
    _face: Face
    _encoding: str
    _dyn_server: dynamic_reconfigure.server.Server

    def __init__(self, topic, run_local=False, width=1280, height=720, encoding="rgb8"):

        self._pub_topic_name = topic
        self._encoding = encoding
        self._face = Face(run_local, width, height)
        self._cv_bridge = cv_bridge.CvBridge()
        self._img_pub = rospy.Publisher(
            self._pub_topic_name, Image, latch=True, queue_size=1)
        # We use our own compression topic instead of ImageTransport because ImageTransport doesn't have a python API.
        self._compressed_img_pub = rospy.Publisher(
            self._pub_topic_name + '/compressed', CompressedImage, latch=True, queue_size=1)

        # start dynamic reconfigure client
        self._dyn_server = dynamic_reconfigure.server.Server(FaceConfig, self._reconfigure_cb)

        # To take advantage of stuff like progress, we would need to use the main our own action server.
        self._animation_server = actionlib.SimpleActionServer(
            "animation", faceAnimationAction, execute_cb=self._animation_cb, auto_start=False)
        self._animation_server.register_preempt_callback(self._animation_preempted_cb)
        self._animation_server.start()

        rospy.loginfo(f"{rospy.get_name()} started.")

    def _reconfigure_cb(self, config, level):
        # set the background color
        self._face.set_background_color([
            config["background_color_r"],
            config["background_color_g"],
            config["background_color_b"]
        ])
        self._face.set_feature_color([
            config["feature_color_r"],
            config["feature_color_g"],
            config["feature_color_b"]
        ])
        return config

    def _animation_preempted_cb(self):
        rospy.loginfo("Pre-empting face animation ...")
        self._face.stop_animation()

    def _animation_cb(self, goal: faceAnimationGoal):
        data = yaml.safe_load(str(goal))
        self._face.start_animation(data)
        result = faceAnimationResult()
        result.success = self._face.wait_for_animation()
        if self._animation_server.is_preempt_requested:
            self._animation_server.set_preempted(result)
        else:
            self._animation_server.set_succeeded(result)

    def _send_image(self, img_data, encoding):
        # send uncompressed
        # if (self._img_pub.get_num_connections() > 0):
        if (self._compressed_img_pub.get_num_connections() > 0 or self._img_pub.get_num_connections() > 0):
            msg = self._cv_bridge.cv2_to_imgmsg(img_data, encoding=encoding)

            # send uncompressed
            if (self._img_pub.get_num_connections() > 0):
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
        refresh_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # Update the face
            self._face.update_once()
            # send the image to baxter's face
            self._send_image(self._face.get_screen_as_bmp(self._encoding), self._encoding)
            # refresh the screen
            refresh_rate.sleep()


def main():
    # initialize node
    rospy.init_node('face_controller')

    # get rosparam
    pub_topic = rospy.get_param("~topic_name", 'face_image')
    run_local = rospy.get_param("~display_local", True)
    width = rospy.get_param("~width", 1280)
    height = rospy.get_param("~height", 720)
    encoding = rospy.get_param("~encoding", "rgb8")

    server = FaceServer(pub_topic, run_local, width=width, height=height, encoding=encoding)
    # loop forever
    server.spin()


if __name__ == '__main__':
    main()
