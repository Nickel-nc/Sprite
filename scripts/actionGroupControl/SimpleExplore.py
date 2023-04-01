from scripts.uSonic import HCSR04
from scripts.actionGroupControl import ActionGroupControl as AGC
from scripts.boardControl import BusCmd as cmd
import time
import numpy as np

import RPi.GPIO as GPIO

"""
Pretty dump space orientation task based on moving sequences presets (Action Groups) and distance from ultrasonic
"""


"""Action Group commands"""

MIN_DIST_THR = 40
MAX_DIST_THR = 100


# Define the robot class
class Actions():
    def __init__(self):
        # self.x = 0.0  # initial x position
        # self.y = 0.0  # initial y position
        # self.theta = 0.0  # initial orientation angle

        self.step_size = 10  # cm
        self.turn_rate = 11.25  # deg
        self.scan_steps = 8 # scan distance in 90 degrees area

    def prepare(self):
        AGC.runActionGroup('stand_low')
        AGC.runActionGroup('stand_middle')

    def shrink(self):
        AGC.runActionGroup('stand_low')

    def forward(self, distance):
        distance = min(distance, MAX_DIST_THR)
        n_steps = distance//self.step_size
        AGC.runActionGroup('go_forward_middle', times=n_steps)
        # self.x += meters * math.cos(math.radians(self.theta))
        # self.y += meters * math.sin(math.radians(self.theta))

    def backward(self, distance):
        distance = min(distance, MAX_DIST_THR)
        n_steps = distance//self.step_size
        AGC.runActionGroup('back_middle', times=n_steps)
        # self.x -= meters * math.cos(math.radians(self.theta))
        # self.y -= meters * math.sin(math.radians(self.theta))

    def turn_left(self, degrees):
        n_steps = degrees//self.turn_rate
        AGC.runActionGroup('turn_left_middle', times=n_steps)
        # self.theta += degrees

    def turn_right(self, degrees):
        n_steps = degrees//self.turn_rate
        AGC.runActionGroup('turn_right_middle', times=n_steps)
        # self.theta -= degrees

    def get_distance(self):
        dist = Usonic.get_usonic_dist()
        return dist

    def rotate_scan(self):

        dist_map = np.zeros(self.scan_steps)
        cur_angle = 0
        dist_map[0] = self.get_distance()
        print("dist_map", dist_map[0], end=' ')
        for i in range(1, self.scan_steps):
            self.turn_right(self.turn_rate)
            cur_angle += self.turn_rate
            dist_map[i] = self.get_distance()

            print("dist_map", dist_map[i], end=' ')

        max_dist = dist_map.max()
        max_idx = np.where(dist_map==max_dist)[0][-1] # pick last angle with max distance
        turn_angle = (self.scan_steps - 1 - max_idx) * self.turn_rate

        print("...turn_angle, max_dist", turn_angle, max_dist)
        return turn_angle, max_dist



if __name__ == "__main__":
    cmd.startSerialProcess()
    Act = Actions()
    time.sleep(0.1)
    Act.prepare()

    try:
        while True:
            # Get the initial distance to the wall/object in front of the robot

            # vision distances:
            # 11.25, 22.5, 33.75, 45, 56.25, 67.5, 78.75, 90
            dist = 0
            while dist < MIN_DIST_THR:
                ang, dist = Act.rotate_scan()

            Act.turn_left(ang)

            Act.forward(dist - MIN_DIST_THR)

    finally:
        Act.shrink()
        GPIO.cleanup()
        time.sleep(0.5)
        cmd.powerOffServos()
        time.sleep(0.5)



