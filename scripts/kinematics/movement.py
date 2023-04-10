
import os, sys
sys.path.append(os.getcwd())

import numpy as np

from scripts.boardControl.BusCmd import BusCmd
from scripts.kinematics.hexapod.solvers.walkSequenceSolver import getWalkSequence
from config.ik_settings import (BASE_POS, SERVO_ORDERED_LEG_NAMES,
                                LEGS_INCREMENT_CORRECTION_MATRIX, BASE_WALK_PARAMS)

class LegTranslator():
    """
    Translate leg angles (alpha, beta, gamma)
    to servo Pulse for each leg (servos 1-18)

    For current state it separated from end-effector translator
    """

    def __init__(self, cmd):

        self.cmd = cmd
        self.current_pos = BASE_POS  # zero state pos

    def translateToServos(self, sequence):

        sequence_len = len(sequence['leftFront']['alpha'])
        servo_commands_sequence = np.full((sequence_len, 18), 500)
        for leg_idx, leg_name in enumerate(SERVO_ORDERED_LEG_NAMES):
            for seq_idx in range(sequence_len):
                servo_commands_sequence[seq_idx][leg_idx*3 + 0] = round(sequence[leg_name]['alpha'][seq_idx],1)
                servo_commands_sequence[seq_idx][leg_idx*3 + 1] = round(sequence[leg_name]['beta'][seq_idx],1)
                servo_commands_sequence[seq_idx][leg_idx*3 + 2] = round(sequence[leg_name]['gamma'][seq_idx],1)
        # print(servo_commands_sequence)
        servo_commands_sequence = self.anglesToPositionsIncrements(servo_commands_sequence)

        servo_commands_sequence *= LEGS_INCREMENT_CORRECTION_MATRIX
        servo_commands_sequence += BASE_POS
        # print("servo_commands_sequence", servo_commands_sequence)

        return servo_commands_sequence

    # solver -> controller

    def anglesToPositionsIncrements(self, angles_bias: np.array) -> np.array:
        """Convert angle increment to position increment"""
        return (1000 / 240 * angles_bias).round().astype('int')

    def run_pos(self, command, rate):

        if np.any(command > 1000) or np.any(command < 0):
            print("Error Positions", command)
            return -1

        self.cmd.moveServos([i + 1 for i in range(18)], command, rate)
        return 0


    def run_sequence(self, sequence, n_times, rate, rev=1):
        servo_commands_sequence = self.translateToServos(sequence)
        for i in range(n_times):
            for pos in servo_commands_sequence[::rev]:
                self.run_pos(pos, rate)


class LegMovement():
    """
    Wrapper to control leg moves through simple encapsulated methods
    """

    def __init__(self, cmd, params = BASE_WALK_PARAMS):
        """
        available parameters:

            "dimensions": BASE_DIMENSIONS, (hexapod real measurments)
            "gaitParams": {
                "tx": 0,
                "tz": 0,
                "rx": 0,
                "ry": 0,
                "legStance": 0,
                "hipStance": 25,
                "stepCount": 4,
                "hipSwing": 25,
                "liftSwing": 40},
        "gaitType": 'tripod', # ripple (can be customized)
        'walkMode': 'walking', # rotating
        """

        self.x = 0
        self.y = 0
        self.face_direction = 0
        self.servo_translator = LegTranslator(cmd)

        self.params = params


    def update_params(self, new_params: dict):
        got_changes = True
        for key, val in new_params.items():
            if self.params[key] != val:
                self.params[key] = val
                got_changes = True
        if got_changes:
            self.update_sequence()

    def update_sequence(self):
        sequence, r_len = getWalkSequence(
            dimensions=self.params["dimensions"],
            params=self.params["gaitParams"],
            gaitType=self.params['gaitType'],
            walkMode=self.params['walkMode'])

        self.sequence = sequence

        alphas = self.sequence['rightMiddle']['alpha']
        angle_diff = abs(max(alphas) - min(alphas))

        l_projected = r_len.x*0.9

        self.step_angle = int(angle_diff)
        self.step_distance = int(l_projected*np.cos(np.radians(90-angle_diff/2))*2)*2  # double step

        print("Unit angle, distance", self.step_angle, self.step_distance)

    def forward(self, distance, rate):
        self.update_params({"walkMode": 'walking'})

        times = distance//self.step_distance
        self.servo_translator.run_sequence(self.sequence, times, rate, rev=1)

        self.x += self.step_distance*times * np.sin(np.radians(self.face_direction))
        self.y += self.step_distance*times * np.cos(np.radians(self.face_direction))


    def backward(self, distance, rate):
        self.update_params({"walkMode": 'walking'})

        times = distance//self.step_distance
        self.servo_translator.run_sequence(self.sequence, times, rate, rev=-1)

        self.x -= self.step_distance*times * np.sin(np.radians(self.face_direction))
        self.y -= self.step_distance*times * np.cos(np.radians(self.face_direction))


    def turn_left(self, degrees, rate):
        self.update_params({"walkMode": 'rotating'})

        times = degrees//self.step_angle
        self.servo_translator.run_sequence(self.sequence, times, rate, rev=1)

        result_angle_diff = self.step_angle * times
        self.face_direction = (self.face_direction + result_angle_diff) % 360

    def turn_right(self, degrees, rate):
        self.update_params({"walkMode": 'rotating'})

        times = degrees // self.step_angle
        self.servo_translator.run_sequence(self.sequence, times, rate, rev=-1)

        result_angle_diff = self.step_angle * times
        self.face_direction = (self.face_direction + result_angle_diff) % 360


"""Usage"""
if __name__ == "__main__":

    cmd = BusCmd()
    cmd.startSerialProcess()
    cmd.moveServos([i for i in range(1,19)],BASE_POS, 1000)

    engine = LegMovement(cmd)
    print("coords:", engine.x, engine.y, engine.face_direction)


    try:
        engine.forward(400,100)
        print("coords:", engine.x, engine.y, engine.face_direction)
        engine.backward(200, 100)

        engine.turn_right(45, 100)

        print("coords:", engine.x, engine.y, engine.face_direction)
        engine.turn_left(90, 100)
        print("coords:", engine.x, engine.y, engine.face_direction)

    finally:
        cmd.powerOffServos()