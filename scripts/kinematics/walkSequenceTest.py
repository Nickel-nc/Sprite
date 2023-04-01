import os, sys
sys.path.append(os.getcwd())

from scripts.kinematics.hexapod.solvers.walkSequenceSolver import getWalkSequence
from config.ik_settings import BASE_DIMENSIONS, BASE_POS, SERVO_ORDERED_LEG_NAMES

from scripts.boardControl import BusCmd as cmd
import time

import numpy as np

def translateToServos(sequence):
    # TODO MOVE EXTERNAL FUNC

    sequence_len = len(sequence['leftFront']['alpha'])
    servo_commands_sequence = np.full((sequence_len, 18), 500)
    for leg_idx, leg_name in enumerate(SERVO_ORDERED_LEG_NAMES):
        for seq_idx in range(sequence_len):
            servo_commands_sequence[seq_idx][leg_idx*3 + 0] = round(sequence[leg_name]['alpha'][seq_idx],1)
            servo_commands_sequence[seq_idx][leg_idx*3 + 1] = round(sequence[leg_name]['beta'][seq_idx],1)
            servo_commands_sequence[seq_idx][leg_idx*3 + 2] = round(sequence[leg_name]['gamma'][seq_idx],1)
    # print(servo_commands_sequence)
    servo_commands_sequence = anglesToPositionsIncrements(servo_commands_sequence)

    servo_commands_sequence *= INCREMENT_CORRECTION_DIMENTIONS
    servo_commands_sequence += BASE_POS
    # print("servo_commands_sequence", servo_commands_sequence)

    return servo_commands_sequence



# solver -> controller
TRANSLATE_POS_ORDER = [4,3,2,5,0,1]


def anglesToPositionsIncrements(angles_bias: np.array) -> np.array:
    """Convert angle increment to position increment"""
    return (1000 / 240 * angles_bias).round().astype('int')


def convert_poses_order(poses):
    converted = np.zeros(18)
    for i, leg_num in enumerate(TRANSLATE_POS_ORDER):
        converted[i * 3] = poses[leg_num]['coxia']
        converted[i * 3 + 1] = poses[leg_num]['femur']
        converted[i * 3 + 2] = poses[leg_num]['tibia']

    return converted

INCREMENT_CORRECTION_DIMENTIONS = np.array([
    1, -1, 1,
    1, -1, 1,
    1, -1, 1,
    1, 1, -1,
    1, 1, -1,
    1, 1, -1
])



def transform_rotation_base(positions):
    # transform matrix
    return positions * INCREMENT_CORRECTION_DIMENTIONS


def translate_poses(angle_increments):
    """Translate angle increment to final servo positions"""
    raw_pos_increments = anglesToPositionsIncrements(angle_increments)
    correct_pos_increments = transform_rotation_base(raw_pos_increments)
    new_positions = BASE_POS + correct_pos_increments
    return new_positions


def update_pos(poses, rate):
    angle_increments = convert_poses_order(poses)
    new_pos = translate_poses(angle_increments)

    if np.any(new_pos > 1000) or np.any(new_pos < 0):
        print("Error Positions", new_pos)
        return 0

    cmd.moveServos([i + 1 for i in range(18)], new_pos, rate)
    return new_pos

def run_pos(command, rate):

    if np.any(command > 1000) or np.any(command < 0):
        print("Error Positions", command)
        return -1

    cmd.moveServos([i + 1 for i in range(18)], command, rate)
    return 0


def get_initial_stance():
    cmd.moveServos([i + 1 for i in range(18)], BASE_POS, 500)
    time.sleep(0.5)

cases = [
    {
        "params": {
            "dimensions": BASE_DIMENSIONS,
            "gaitParams": {
                "tx": 0,
                "tz": 0,
                "rx": 0,
                "ry": 0,
                "legStance": 0,
                "hipStance": 25,
                "stepCount": 4,
                "hipSwing": 25,
                "liftSwing": 40,
            },
        },
        "result": {"answer": True},
        "description": "first sequence",
    },
]

if __name__ == "__main__":
    try:
        # prepare hexapod
        cmd.startSerialProcess()
        get_initial_stance()


        example = cases[0]
        stepCount = example["params"]["gaitParams"]["stepCount"]

        sequence = getWalkSequence(
            dimensions=example["params"]["dimensions"],
            params=example["params"]["gaitParams"],
            gaitType="tripod", # "tripod"
            walkMode='rotating' # rotating
        )
        # print(sequence['leftBack']["alpha"])

        # leftFront: {alpha: [....], ..., leftRight:}
        command_sequence = translateToServos(sequence)
        for _ in range(4):
            for seq in command_sequence:
                run_pos(seq, 100)



    except Exception as e:
        print(e)
    finally:
        time.sleep(0.5)
        cmd.powerOffServos()
        time.sleep(0.5)