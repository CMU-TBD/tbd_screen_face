#!/usr/bin/python3

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
from .Face.Face import Face


class FaceServer:

    def __init__(self, topic, run_local=False, width=1280, height=720):

        self._face = Face(run_local, width, height)
        #TODO use ImageTranporter instead
        self._img_pub = rospy.Publisher(
            self._pub_topic_name, Image, latch=True, queue_size=1)
        self._compressed_img_pub = rospy.Publisher(
            self._pub_topic_name + '/compressed', CompressedImage, latch=True, queue_size=1)

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
            self.send_screen(self._face.get_screen_as_bmp())
            # refresh the screen
            refresh_rate.sleep()

def main():
    # initialize node
    rospy.init_node('baxter_face_controller')

    # get rosparam
    pub_topic = rospy.get_param("topic_name", 'face_image')
    run_local = rospy.get_param("run_local", True)

    server = FaceServer(pub_topic, run_local)
    # loop forever
    server.spin()


if __name__ == '__main__':
    main()
