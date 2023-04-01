# ***************************
# Settings
# ***************************
from numpy import array as np_array

# hexapod physical dimensions
BASE_DIMENSIONS = {
    "front": 118//2,
    "side": 240//2,
    "middle": 184//2,
    "coxia": 45,
    "femur": 75,
    "tibia": 135,
}

BASE_POS = np_array([
    500, 500, 125,               # left-back
    500, 500, 125,               # left-middle
    500, 500, 125,               # left-front
    500, 500, 875,               # right-back
    500, 500, 875,               # right-middle
    500, 500, 875                # right-front
    ])

SERVO_ORDERED_LEG_NAMES = [
    "leftBack",
    "leftMiddle",
    "leftFront",
    "rightBack",
    "rightMiddle",
    "rightFront"
]

LEG_NAMES = [
    "rightMiddle",
    "rightFront",
    "leftFront",
    "leftMiddle",
    "leftBack",
    "rightBack",
]

LEG_POINT_TYPES_LIST = [
    "bodyContactPoint",
    "coxiaPoint",
    "femurPoint",
    "footTipPoint",
]

BASE_IK_PARAMS = {
    "hip_stance": 0,
    "leg_stance": 0,
    "percent_x": 0,
    "percent_y": 0,
    "percent_z": 0,
    "rot_x": 0,
    "rot_y": 0,
    "rot_z": 0,
}

DEFAULT_POSE = {
    0: {"coxia": 0, "femur": 0, "tibia": 0, "name": "right-middle", "id": 0},
    1: {"coxia": 0, "femur": 0, "tibia": 0, "name": "right-front", "id": 1},
    2: {"coxia": 0, "femur": 0, "tibia": 0, "name": "left-front", "id": 2},
    3: {"coxia": 0, "femur": 0, "tibia": 0, "name": "left-middle", "id": 3},
    4: {"coxia": 0, "femur": 0, "tibia": 0, "name": "left-back", "id": 4},
    5: {"coxia": 0, "femur": 0, "tibia": 0, "name": "right-back", "id": 5},
}

POSITION_NAME_TO_AXIS_ANGLE_MAP = {
    "rightMiddle": 0,
    "rightFront": 45,
    "leftFront": 135,
    "leftMiddle": 180,
    "leftBack": 225,
    "rightBack": 315,
}

POSITION_NAME_TO_ID_MAP = {
    "rightMiddle": 0,
    "rightFront": 1,
    "leftFront": 2,
    "leftMiddle": 3,
    "leftBack": 4,
    "rightBack": 5,
}

MAX_ANGLES = {
    "alpha": 90,
    "beta": 180,
    "gamma": 180,
}

POSITION_NAME_TO_IS_LEFT_MAP = {
    "rightMiddle": False,
    "rightFront": False,
    "leftFront": True,
    "leftMiddle": True,
    "leftBack": True,
    "rightBack": False,
}

NUMBER_OF_LEGS = 6

# BASE_HEXAPOD = VirtualHexapod(BASE_DIMENSIONS)

# The range of each leg joint in degrees
# ALPHA_MAX_ANGLE = 90
# BETA_MAX_ANGLE = 120
# GAMMA_MAX_ANGLE = 120
BODY_MAX_ANGLE = 20

# LEG STANCE
# would define the starting leg position used to compute
# the target ground contact for inverse kinematics poses
# femur/ beta = -leg_stance
# tibia/ gamma = leg_stance
LEG_STANCE_MAX_ANGLE = 90

# HIP STANCE
# would defined the starting hip position used to compute
# the target ground contact for inverse kinematics poses
# coxia/alpha angle of
#  right_front = -hip_stance
#   left_front = +hip_stance
#    left_back = -hip_stance
#   right_back = +hip_stance
#  left_middle = 0
# right_middle = 0
HIP_STANCE_MAX_ANGLE = 20

DEBUG_MODE = False
ASSERTION_ENABLED = False

# The inverse kinematics solver already updates the points of the hexapod
# But there is no guarantee that this pose is correct
# So better update a fresh hexapod with the resulting poses
RECOMPUTE_HEXAPOD = True

PRINT_IK_LOCAL_LEG = False
PRINT_IK = False
PRINT_MODEL_ON_UPDATE = False
