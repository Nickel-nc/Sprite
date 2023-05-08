
import os, sys
sys.path.append(os.getcwd())

import numpy as np

from scripts.kinematics.hexapod.solvers.walkSequenceSolver import getWalkSequence
from config.ik_settings import (BASE_POS, SERVO_ORDERED_LEG_NAMES,
                                LEGS_INCREMENT_CORRECTION_MATRIX, BASE_WALK_PARAMS)

from config.poses import (LEG_SERVO_IDS)


global CMD, LIDAR

# CMD = BusCmd()
# CMD.startSerialProcess()
# LIDAR = Lidar()


class LegTranslator():
    """
    Translate leg angles (alpha, beta, gamma)
    to servo Pulse for each leg (servos 1-18)

    Featured for current ik model data structure
    """

    def __init__(self):
        self.cmd = CMD


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

        # TODO: add sanity check
        if np.any(command > 1000) or np.any(command < 0):
            print("Error Positions", command)
            return -1

        self.cmd.moveServos(LEG_SERVO_IDS, command, rate)
        return 0


    def run_sequence(self, sequence, n_times, rate):
        servo_commands_sequence = self.translateToServos(sequence)
        for i in range(n_times):
            for pos in servo_commands_sequence:
                self.run_pos(pos, rate)


class PositionTracker():
    """

    """
    def __init__(self):
        self.x = 0
        self.y = 0
        self.face_direction = 0

        self.prev_lidar_points = None
        self.walkable = None


    def match_points(self, new_cloud):
        """Matching lidar point clouds"""

        # process magic

        self.prev_lidar_points = new_cloud

        xy_diff = (None, None)
        rotation_angle = None


    def check_bounds(self):
        pass

    def update_space_position(self):
        new_cloud = LIDAR.get_circle_batch()

        x_diff, y_diff, r_angle = self.match_points(new_cloud)




class LegMovement():
    """
    Wrapper to control leg moves through simple encapsulated methods

    Move straight line on fixed distance or rotate for fixed angle
    """

    def __init__(self, params = BASE_WALK_PARAMS):
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
        super().__init__()

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


        l_projected = r_len.x*0.9  # coef of constructional properties

        self.step_angle = int(angle_diff)
        self.step_distance = int(l_projected*np.cos(np.radians(90-angle_diff/2))*2)*2  # double step

        # print("Angles: max, min, diff", max(alphas), min(alphas), abs(max(alphas) - min(alphas)), "Double step", self.step_distance)

        # print("Unit angle, distance", self.step_angle, self.step_distance)

    def forward(self, distance, rate):
        for hip_swg in range(1,10):
            print("hip swg", hip_swg)
            self.update_params({"walkMode": 'walking'})

        # times = distance//self.step_distance
        # self.run_sequence(self.sequence, times, rate)
        #
        # self.x += self.step_distance*times * np.sin(np.radians(self.face_direction))
        # self.y += self.step_distance*times * np.cos(np.radians(self.face_direction))


    def backward(self, distance, rate):
        self.update_params({"walkMode": 'walking'})

        # times = distance//self.step_distance
        # self.run_sequence(self.sequence, times, rate)
        #
        # self.x -= self.step_distance*times * np.sin(np.radians(self.face_direction))
        # self.y -= self.step_distance*times * np.cos(np.radians(self.face_direction))


    def turn_left(self, degrees, rate):
        self.update_params({"walkMode": 'rotating'})

        # times = degrees//self.step_angle
        # self.run_sequence(self.sequence, times, rate)
        #
        # result_angle_diff = self.step_angle * times
        # self.face_direction = (self.face_direction + result_angle_diff) % 360

    def turn_right(self, degrees, rate):
        self.update_params({"walkMode": 'rotating'})

        # times = degrees // self.step_angle
        # self.run_sequence(self.sequence, times, rate)
        #
        # result_angle_diff = self.step_angle * times
        # self.face_direction = (self.face_direction + result_angle_diff) % 360

    def process_action(self, remain_amt, rate, is_rotating, rev=1):

        while remain_amt > 0 and self.walkable:
            if is_rotating:
                step_size = self.step_angle
            else:
                step_size = self.step_distance

            if remain_amt >= step_size:
                self.run_sequence(self.sequence[::rev], 1, rate)
                self.update_space_position()
            else:
                for pos in self.sequence[::rev]:
                    self.run_pos(pos, rate)
                    self.update_space_position()
                    if not self.walkable:
                        break

"""Usage"""
if __name__ == "__main__":

    move = LegMovement()

    # move.run_pos(POSE_STAND_BASE, 1000)

    # print("coords:", move.x, move.y, move.face_direction)


    try:
        move.forward(400, 100)
        print("coords:", move.x, move.y, move.face_direction)
        move.backward(200, 100)

        move.turn_right(45, 100)

        print("coords:", move.x, move.y, move.face_direction)
        move.turn_left(90, 100)
        print("coords:", move.x, move.y, move.face_direction)

    finally:
        pass
        # move.run_pos(POSE_STAND_LOW, 1000)
        # CMD.powerOffServos()
        # LIDAR.close_port()