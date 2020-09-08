#!/usr/bin/env python

import os
import typing
import numpy as np
import pygame
import pygame.gfxdraw
import time
from enum import Enum

class AnimationTypes(Enum):
    LINEAR = 0
    ACCELERATE_DECCELERATE = 1

class Face:

    _width: int
    _height: int

    _display_local: bool # Whether the screen is shown on the local machine

    _running: bool # Whether we are running
    _animating: bool # Whether the face is undergoing animations
    _animation_info: dict # Information about dictionary
    _face_states: dict # Representing the information about the face

    def __init__(self, display_local: bool = False, width: int = 1280, height: int = 720):
        pygame.init()

        self._width = width
        self._height = height

        self._screen = pygame.Surface((self._width, self._height))

        self._display_local = display_local
        if self._display_local:
            self._local_window = pygame.display.set_mode(
                (self._width, self._height))
            self._local_window.fill((255,0,0))

        self._face_states = {
            'left_eye': {
                'center': [0.25, 0.35],
                'width': 0.2,
                'height': 0.5,
                'eye_brow': {
                    'thickness': 40,
                    'distance': "50px",
                }
            },
            'right_eye': {
                'center': [0.75, 0.35],
                'width': 0.2,
                'height': 0.5,
                'eye_brow': {
                    'thickness': 40,
                    'distance': "50px",
                }
            },
            'blink': {
                'duration': 0.5,  # second long
                'frequency_mean': 8,
                'frequency_std': 1,
                'last_blink': self.get_time_now(),
                'in-progress': -1,
            },
            'background_color': [255, 100, 100]
        }

        self._animating = False
        self._animation_info = {}

        self._running = True


    def get_face_width(self) -> int:
        return self._width

    def get_face_height(self) -> int:
        return self._height


    def _get_eye_rect(self, eye_info):

        # figure out the width and height of the eye
        eye_width = self._parse_values(eye_info['width'], self._width)
        eye_height = self._parse_values(eye_info['height'], self._height)
        # figure out the top right corner
        eye_corner_x = self._parse_values(eye_info['center'][0], self._width) - int(eye_width)/2 
        eye_corner_y = self._parse_values(eye_info['center'][1], self._height)  - int(eye_height)/2 

        return pygame.Rect(eye_corner_x, eye_corner_y, eye_width, eye_height)

    def get_time_now(self) -> float:
        """Return the current time (since epoch) in seconds.

        Returns
        -------
        float
            Current time in seconds
        """
        return time.time()

    def sleep(self, sleep_time: float) -> None:
        time.sleep(sleep_time)

    def draw_face(self):
        """Draw Baxter's face on the surface
        """

        # fill up the screen with the background color
        self._screen.fill(self._face_states['background_color'])

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
            self._face_states['blink']['in-progress'] = (self.get_time_now() - self._face_states['blink']['last_blink'])
            # if the time exceeded the blink duration, stop
            if self._face_states['blink']['in-progress'] > self._face_states['blink']['duration']:
                self._face_states['blink']['in-progress'] = -1
                self._face_states['blink']['last_blink'] = self.get_time_now()
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
                pygame.draw.ellipse(self._screen, self._face_states['background_color'], left_eye_rect)

                right_eye_rect = right_eye_rect.move((0, np.floor(diff * -1 * right_eye_height)))
                pygame.draw.ellipse(self._screen, self._face_states['background_color'], right_eye_rect)

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

    def _get_state_value(self, name:str):
         # split the name
        name_arg = name.split(".")
        curr_state = self._face_states
        for i in range(0, len(name_arg)):
            curr_state = curr_state[name_arg[i]]
        return curr_state

    def _set_state_value(self, name:str, val: typing.Any) -> None:
        """ Helper method to set the value in the face state.
        
        Parameters
        ----------
        name : str
            name in the dict in string and allow . access
        val : typing.Any
            The value to be placed into the dict
        """
        # split the name
        name_arg = name.split(".")
        curr_state = self._face_states
        for i in range(0, len(name_arg) - 1):
            curr_state = curr_state[name_arg[i]]
        # set the actual value
        curr_state[name_arg[-1]] = val


    def get_screen_as_bmp(self, encoding='bgr8'):
        # convert the output screen to numpy array
        ori_img_data = pygame.surfarray.array3d(self._screen)
        # change from rgb to bgr and flip the image
        if encoding == 'bgr8':
            return np.swapaxes(np.roll(np.uint8(ori_img_data), 2, 2), 0, 1)
        elif encoding == 'rgb8':
            return np.swapaxes(np.uint8(ori_img_data), 0, 1)
        else:
            raise NotImplementedError("encoding {} not supported".format(encoding))

    def _update_state(self):

        if self._animating:

            running = False
            # go through each object
            for obj in self._animation_info['objects']:
                # figure out the progress
                progress = (self.get_time_now() - self._animation_info['start_time'])/obj['duration']
                if progress >= 1:
                    progress = 1
                else:
                    running = True
                obj_name = obj['name']

                if AnimationTypes(obj['type']) == AnimationTypes.LINEAR:
                    if 'min' not in obj:
                        obj['min'] = self._get_state_value(obj_name)
                    if type(obj['target']) == list:
                        curr_val = [None] * len(obj['target'])
                        for i in range(0, len(obj['target'])):
                            curr_val[i] = (obj['target'][i] - obj['min'][i]) * progress + obj['min'][i]
                    else:
                        curr_val = (obj['target'] - obj['min']) * progress + obj['min']  
                    self._set_state_value(obj_name, curr_val)   
            
            # set if we are still runnin
            self._animating = running
        else:
            # run idle motion code
            # determine if the face should blink or not
            if (self._face_states['blink'] and self._face_states['blink']['in-progress'] == -1):
                # check whether to blink
                # draw number from frequency distribution
                num = np.random.normal(
                    self._face_states['blink']['frequency_mean'], self._face_states['blink']['frequency_std'])
                # check if its time to blink
                if self.get_time_now() > self._face_states['blink']['last_blink'] + num: 
                    # blink in progress:
                    self._face_states['blink']['in-progress'] = 0
                    self._face_states['blink']['last_blink'] = self.get_time_now()
        # return


    def update_once(self):
        # Run any code to update the face state
        self._update_state()
        # draw the surface that makes baxter's face
        self.draw_face()
        if self._display_local:
            self._local_window.blit(self._screen, (0, 0))
            pygame.display.update()

    def update(self, timeout: float = -1, hz: float = 60) -> None:
        
        start_time = self.get_time_now()
        cycle_time = 1/hz
        last_time = start_time
        expected_end_time = start_time + timeout
        while (timeout != -1 and expected_end_time > self.get_time_now()) and self._running:

            self.update_once()

            # sleep if not enough time passed
            curr_time = self.get_time_now()
            sleep_time = (self.get_time_now() - last_time) - cycle_time
            if sleep_time > 0:
                self.sleep(sleep_time)
            last_time = curr_time
            

        



    def start_animation(self, animation_info: dict, start_time: float = None) -> None:
        """Start animating the face based on the animation given by animation_info

        Parameters
        ----------
        animation_info : dict
            A dict in a specific format describing the animation to do
        start_time : float, optional
            starting time for the animation, could be in the future, by default None
        """
        self._animation_info = animation_info
        self._animation_info['start_time'] = start_time if start_time != None else self.get_time_now()
        # find the longest duration
        max_time = 0
        for obj in self._animation_info['objects']:
            max_time = np.max([obj['duration'], max_time])
        self._animation_info['duration'] = max_time
        self._animating = True

    def stop_animation(self):
        if self._animating:
            self._animating = False
            self._animation_info = None


    def wait_for_animation(self, timeout = None):
        wait_start_time = self.get_time_now()
        while self._animating:
            # sleep until timeout
            if timeout != None and (self.get_time_now() - wait_start_time) > timeout:
                return False
            time.sleep(0.01)
        return True