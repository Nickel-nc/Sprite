import os, sys
sys.path.append(os.getcwd())

import numpy as np
from time import sleep
import math

from scripts.kinematics.hexapod.solvers.walkSequenceSolver import getWalkSequence
from config.ik_settings import (BASE_POS, SERVO_ORDERED_LEG_NAMES,
                                LEGS_INCREMENT_CORRECTION_MATRIX, BASE_WALK_PARAMS, MAX_HIP_SWING, P_LEN)
from config.poses import (LEG_SERVO_IDS, POSE_STAND_BASE, POSE_STAND_LOW)
from scripts.boardControl.BusCmd import BusCmd


class ServoTranslateEngine:

    """
    Wrapper to translate motor angle move commands to servo controller instructions.
    Translate IK leg angles (alpha, beta, gamma) to servo Pulse for each leg (servos 1-18).
    Also contains some preset basic functions

    Featured for ik model data structure
    """

    def __init__(self, min_rate=60, max_rate=300, default_rate=500):
        self.min_rate = min_rate
        self.base_rate = default_rate
        self.max_rate = max_rate

        self.cmd = BusCmd()
        self.cmd.startSerialProcess()


    def translate_to_servos(self, sequence):

        sequence_len = len(sequence['leftFront']['alpha'])
        servo_commands_sequence = np.full((sequence_len, 18), 500)
        for leg_idx, leg_name in enumerate(SERVO_ORDERED_LEG_NAMES):
            for seq_idx in range(sequence_len):
                servo_commands_sequence[seq_idx][leg_idx*3 + 0] = round(sequence[leg_name]['alpha'][seq_idx],1)
                servo_commands_sequence[seq_idx][leg_idx*3 + 1] = round(sequence[leg_name]['beta'][seq_idx],1)
                servo_commands_sequence[seq_idx][leg_idx*3 + 2] = round(sequence[leg_name]['gamma'][seq_idx],1)

        servo_commands_sequence = self.angles_to_pos_increments(servo_commands_sequence)
        servo_commands_sequence *= LEGS_INCREMENT_CORRECTION_MATRIX
        servo_commands_sequence += BASE_POS

        return servo_commands_sequence

    # solver -> controller

    def angles_to_pos_increments(self, angles_bias: np.array) -> np.array:
        """Convert angle increment to position increment"""
        return (1000 / 240 * angles_bias).round().astype('int')

    def run_pos(self, command, rate):
        if np.any(command > 1000) or np.any(command < 0):  # Violate servo rotation angles
            return -1

        self.cmd.moveServos(LEG_SERVO_IDS, command, rate)
        return 0

    def run_sequence(self, sequence, n_times, rate, direction=1):
        servo_commands_sequence = self.translate_to_servos(sequence)
        for i in range(n_times):
            for pos in servo_commands_sequence[::direction]:
                self.run_pos(pos, rate)

    def shutdown_servos(self):
        self.cmd.moveServos(LEG_SERVO_IDS, POSE_STAND_LOW, 1000)
        sleep(0.1)
        self.cmd.powerOffServos()


class IKMove(ServoTranslateEngine):
    """
    Wrapper to control leg moves through simple encapsulated methods

    Move straight line on fixed distance or rotate for fixed angle
    """

    def __init__(self, params=BASE_WALK_PARAMS):
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
        "gaitType": 'tripod', # or ripple (not well tested)
        'walkMode': 'walking', # or rotating
        """
        super().__init__()

        self.precalc_setup()
        self.ik_params = params
        self.stand()

    def precalc_setup(self):
        """
        Precalculated params for base stance to ease steps estimation for earlier stages
        Separated in distinct function
        """
        self.max_hip_swing = MAX_HIP_SWING
        self.max_hip_angle = MAX_HIP_SWING * 2
        self.p_len = P_LEN
        self.max_step_dist = (self.p_len * math.cos(math.radians(90 - self.max_hip_angle / 2)) * 4)
        self.deg_rate_coef = 1.5

    def update_sequence(self):
        sequence, r_len = getWalkSequence(
            dimensions=self.ik_params["dimensions"],
            params=self.ik_params["gaitParams"],
            gaitType=self.ik_params['gaitType'],
            walkMode=self.ik_params['walkMode'])

        self.sequence = sequence

    def forward(self, distance):

        n_steps = math.ceil(distance / self.max_step_dist)

        estimated_unit_dist = distance / n_steps

        hip_swing = 0.5 * math.degrees(
            math.pi - (math.acos(estimated_unit_dist / (4 * self.p_len))) * 2
        )

        self.ik_params['walkMode'] = 'walking'
        self.ik_params['gaitParams']['hipSwing'] = round(hip_swing, 1)
        rate = int(max(self.min_rate, hip_swing / self.max_hip_swing * self.max_rate))

        self.update_sequence()
        self.run_sequence(self.sequence, n_steps, rate)

    def backward(self, distance):

        n_steps = math.ceil(distance / self.max_step_dist)

        estimated_unit_dist = distance / n_steps

        hip_swing = 0.5 * math.degrees(
            math.pi - (math.acos(estimated_unit_dist / (4 * self.p_len))) * 2
            )

        self.ik_params['walkMode'] = 'walking'
        self.ik_params['gaitParams']['hipSwing'] = round(hip_swing, 1)
        rate = int(max(self.min_rate, hip_swing / self.max_hip_swing * self.max_rate))

        self.update_sequence()
        self.run_sequence(self.sequence, n_steps, rate, -1)

    def turn_left(self, degrees):
        degrees = degrees * self.deg_rate_coef

        n_steps = math.ceil(degrees / self.max_hip_angle)
        hip_swing = degrees / (n_steps*2)

        self.ik_params['walkMode'] = 'rotating'
        self.ik_params['gaitParams']['hipSwing'] = round(hip_swing, 1)

        rate = int(max(self.min_rate, hip_swing / self.max_hip_swing * self.max_rate))
        self.update_sequence()

        self.run_sequence(self.sequence, n_steps, rate)

    def turn_right(self, degrees):
        degrees = degrees * self.deg_rate_coef
        n_steps = math.ceil(degrees / self.max_hip_angle)
        hip_swing = degrees / (n_steps*2)

        self.ik_params['walkMode'] = 'rotating'
        self.ik_params['gaitParams']['hipSwing'] = round(hip_swing, 1)

        rate = int(max(self.min_rate, hip_swing / self.max_hip_swing * self.max_rate))
        self.update_sequence()
        self.run_sequence(self.sequence, n_steps, rate, -1)

    def stand(self, pose=POSE_STAND_BASE, rate=1000):
        self.run_pos(pose,rate)



if __name__ == "__main__":

    """Usage test"""

    ik = IKMove()

    try:
        # walk around square and check loop accuracy
        for _ in range(4):
            ik.forward(500)
            ik.turn_left(90)

        for _ in range(4):
            ik.turn_right(90)
            ik.backward(500)

    finally:
        ik.shutdown_servos()