#!/usr/bin/python3

import os
import typing
import cv2
import numpy as np
import cv_bridge
import rospy
import actionlib
import pygame
import pygame.gfxdraw
from sensor_msgs.msg import (
    Image,
    CompressedImage
)

PACKAGE_NAME = "tbd_robot_face"

BACKGROUND_COLOR = (255, 100, 100)


FACE_HEIGHT = 600
FACE_WIDTH = 1024


class FaceServer:

    _width: int
    _height: int

    def __init__(self, topic, run_local=False, width=1280, height=720):
        pygame.init()

        self._width = width
        self._height = height

        self._screen = pygame.Surface((self._width, self._height))
        self._pub_topic_name = topic

        self._run_local = run_local
        if self._run_local:
            self._local_window = pygame.display.set_mode(
                (self._width, self._height))

        self._img_pub = rospy.Publisher(
            self._pub_topic_name, Image, latch=True, queue_size=1)
        self._compressed_img_pub = rospy.Publisher(
            self._pub_topic_name + '/compressed', CompressedImage, latch=True, queue_size=1)

        self._face_states = {
            'left_eye': {
                'center': (0.25, 0.35),
                'width': 0.2,
                'height': 0.5,
                'eye_brow': {
                    'thickness': 40,
                    'distance': "50px",
                }
            },
            'right_eye': {
                'center': (0.75, 0.35),
                'width': 0.2,
                'height': 0.5,
            },
            'blink': {
                'duration': 0.5,  # second long
                'frequency_mean': 8,
                'frequency_std': 1,
                'last_blink': rospy.Time.now(),
                'in-progress': -1,
            }

        }

    def send_screen(self):
        # convert the output screen to numpy array
        ori_img_data = pygame.surfarray.array3d(self._screen)
        # change from rgb to bgr and flip the image
        img_data = np.swapaxes(np.roll(np.uint8(ori_img_data), 2, 2), 0, 1)
        # send the image data
        self._send_image(img_data)

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

    def _draw_face(self):
        """ Wrapper for the draw face method
        """
        self._screen.fill(BACKGROUND_COLOR)
        self.draw_face()

    def _get_eye_rect(self, eye_info):

        # figure out the width and height of the eye
        eye_width = self._parse_values(eye_info['width'], self._width)
        eye_height = self._parse_values(eye_info['height'], self._height)
        # figure out the top right corner
        eye_corner_x = self._parse_values(eye_info['center'][0], self._width) - int(eye_width)/2 
        eye_corner_y = self._parse_values(eye_info['center'][1], self._height)  - int(eye_height)/2 

        return pygame.Rect(eye_corner_x, eye_corner_y, eye_width, eye_height)

    def draw_face(self):
        """Draw Baxter's face on the surface
        """

        # draw the eyes
        # left eye
        left_eye_rect = self._get_eye_rect(self._face_states['left_eye'])
        pygame.draw.ellipse(self._screen, pygame.Color('black'), left_eye_rect)
        # right eye
        right_eye_rect = self._get_eye_rect(self._face_states['right_eye'])
        pygame.draw.ellipse(
            self._screen, pygame.Color('black'), right_eye_rect)

        # check if blinking is enable and whether we are blinking
        if (self._face_states['blink'] and self._face_states['blink']['in-progress'] != -1):
            # get time since last blink started
            self._face_states['blink']['in-progress'] = (rospy.Time.now() - self._face_states['blink']['last_blink']).to_sec()
            # if the time exceeded the blink duration, stop
            if self._face_states['blink']['in-progress'] > self._face_states['blink']['duration']:
                self._face_states['blink']['in-progress'] = -1
                self._face_states['blink']['last_blink'] = rospy.Time.now()
            else:
                # draw the eye lids
                progress = self._face_states['blink']['in-progress']/self._face_states['blink']['duration']
                # figure out the direction, moving up or down
                diff = (progress/0.5) if (progress <= 0.5) else (progress/0.5 - 2)*-1
                diff = 1 - diff
                # get the height
                left_eye_height = self._parse_values(self._face_states['left_eye']['height'], self._height)
                right_eye_height = self._parse_values(self._face_states['right_eye']['height'], self._height)
                # draw the eyelids
                left_eye_rect = left_eye_rect.move((0, np.floor(diff * -1 * left_eye_height)))
                pygame.draw.ellipse(self._screen, BACKGROUND_COLOR, left_eye_rect)

                right_eye_rect = right_eye_rect.move((0, np.floor(diff * -1 * right_eye_height)))
                pygame.draw.ellipse(self._screen, BACKGROUND_COLOR, right_eye_rect)

        # draw eye brows
        # which consisted of multiple arches
        self.draw_eye_brow(self._face_states["right_eye"], self._screen)
        self.draw_eye_brow(self._face_states["left_eye"], self._screen)

    def _parse_values(self, val: typing.Any, modifier: int) -> int:
        if type(val) == str:
            # check ending
            if val.endswith("px"):
                return int(val[:-2])
            elif val.endswith("%"):
                return float(val[:-1])/100.0
        elif type(val) == float:
            if val <= 1 and val >= 0:
                return int(val * modifier)
        elif type(val) == int:
            return val
        raise SyntaxError("Unable to parse {}".format(val))


    def draw_eye_brow(self, eye_state: dict, screen):
        if 'eye_brow' in eye_state:
            eye_center_x = self._parse_values(eye_state['center'][0], self._width) 
            eye_center_y = self._parse_values(eye_state['center'][1], self._height)

            eye_height =  self._parse_values(eye_state['height'], self._height)
            eye_brow_distance =  self._parse_values(eye_state['eye_brow']['distance'], self._height)

            for i in range(0, eye_state['eye_brow']['thickness']):
                pygame.gfxdraw.arc(screen, eye_center_x, eye_center_y,
                                   int(np.floor(eye_height/2) + eye_brow_distance - i), 240, 300, pygame.Color('black'))

    def _update_state(self):

        # determine if the face should blink or not
        if (self._face_states['blink'] and self._face_states['blink']['in-progress'] == -1):
            # check whether to blink
            # draw number from frequency distribution
            num = np.random.normal(
                self._face_states['blink']['frequency_mean'], self._face_states['blink']['frequency_std'])
            # check if its time to blink
            if rospy.Time.now() > self._face_states['blink']['last_blink'] + rospy.Duration.from_sec(num):
                # blink in progress:
                self._face_states['blink']['in-progress'] = 0
                self._face_states['blink']['last_blink'] = rospy.Time.now()

    
        # return

    def spin(self):
        refresh_rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            # Run any code to update the face state
            self._update_state()
            # draw the surface that makes baxter's face
            self._draw_face()
            # send the image to baxter's face
            self.send_screen()
            # display it on local machine if set to true
            if self._run_local:
                self._local_window.blit(self._screen, (0, 0))
                pygame.display.update()
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
