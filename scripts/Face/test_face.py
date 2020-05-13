from .Face import Face, AnimationTypes
import time
import pygame
from pygame.locals import *
import sys
import numpy as np
import pytest


def test_build():
    f = Face(False)


def test_width_height():
    f = Face(width=800, height=600)

    assert f.get_face_height() == 600
    assert f.get_face_width() == 800

    # get an image and check it's size
    face_bmp = f.get_screen_as_bmp()
    face_np_shape = np.shape(face_bmp)

    assert face_np_shape[0] == 600
    assert face_np_shape[1] == 800


def test_update():

    f = Face()
    start_time = time.time()
    f.update(0.5)
    assert pytest.approx(time.time() - start_time, rel=1e-2) == 0.5


def test_animation_single():

    f = Face(False)
    # simple animation
    animation_info = {
        'objects': [
            {
                'type': AnimationTypes.LINEAR,
                'name': 'left_eye.center',
                'target': [0.15, 0.35],
                'duration': 1,
            }]
    }
    f.start_animation(animation_info)
    # spin the face for half the duration animation and check the position
    f.update(0.5)
    assert pytest.approx(f._get_state_value('left_eye.center'), rel=1e-2) == [0.20, 0.35]
    # spin all the way and check
    f.update(0.6)
    assert pytest.approx(f._get_state_value('left_eye.center')) == [0.15, 0.35]


def test_animation_group():
    f = Face(False)
    # simple animation II
    animation_info = {
        'objects': [
            {
                'type': AnimationTypes.LINEAR,
                'name': 'left_eye.center',
                'target': [0.15, 0.35],
                'duration': 1,
            },
            {
                'type': AnimationTypes.LINEAR,
                'name': 'right_eye.center',
                'target': [0.65, 0.35],
                'duration': 2,
            }]
    }

    f.start_animation(animation_info)
    f.update(1.1)
    assert f._get_state_value('left_eye.center') == [0.15, 0.35]
    assert f._get_state_value('right_eye.center') != [
        0.65, 0.35]  # right eye should reach it yet
    f.update(1)
    assert f._get_state_value('right_eye.center') == [0.65, 0.35]
