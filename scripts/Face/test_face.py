from Face import Face, AnimationTypes
import time
import pygame
from pygame.locals import *
import sys
import numpy as np 

if __name__ == "__main__":
    f = Face(True)
    
    start_time = time.time()
    ran = False

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                sys.exit()
        f.update_once()

        if time.time() - start_time > 1 and not ran:
            ran = True
            animation_info = {
                'data':[
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
                        },
                    ]
            }
            f.run_animation(animation_info)
        
   