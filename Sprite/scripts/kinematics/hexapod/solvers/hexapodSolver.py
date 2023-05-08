from Sprite.config.ik_settings import LEG_NAMES

from Sprite.scripts.kinematics.hexapod.utils.geometry import (
    tRotZmatrix,
    tRotXYZmatrix,
    vectorFromTo,
    angleBetween,
    isCounterClockwise,
)

from Sprite.scripts.kinematics.hexapod.utils.vector import Vector

from Sprite.scripts.kinematics.hexapod.Hexapod import Hexapod
from Sprite.scripts.kinematics.hexapod.solvers.IKSolver import IKSolver
from typing import List, Dict, Tuple

def solveInverseKinematics(
    dimensions: Dict[str, float],
    rawIKparams: Dict[str, float],
    flags: Dict[str, bool] = {"rotateThenShift": True},
) -> Dict[str, any]:
    ikSolver, target_groundContactPoints, _ = solveHexapodParams(
        dimensions, rawIKparams, flags["rotateThenShift"]
    )

    if not ikSolver.foundSolution:
        return {
            "pose": None,
            "obtainedSolution": False,
            "message": ikSolver.message,
            "hexapod": None,
        }

    currentHexapod = Hexapod(dimensions, ikSolver.pose)
    excludedPositions = ikSolver.legPositionsOffGround

    pivots = findTwoPivotPoints(
        currentHexapod.groundContactPoints,
        target_groundContactPoints,
        excludedPositions,
    )

    hexapod = (
        rotateShiftHexapodgivenPivots(currentHexapod, pivots.points1, pivots.points2)
        if pivots.foundTwoPoints
        else currentHexapod
    )

    return {
        "pose": ikSolver.pose,
        "obtainedSolution": True,
        "message": ikSolver.message,
        "hexapod": hexapod,
    }

def solveHexapodParams(
    dimensions: Dict[str, float],
    rawIKparams: Dict[str, float],
    rotateThenShift: bool) -> Tuple[IKSolver, List[Vector]]:

    tVec, startPose, rotMatrix = convertIKparams(dimensions, rawIKparams)

    startHexapod = Hexapod(dimensions, startPose)

    targets = buildHexapodTargets(startHexapod, rotMatrix, tVec, rotateThenShift)


    ikSolver = IKSolver().solve(
        startHexapod.legDimensions,
        targets["bodyContactPoints"],
        targets["groundContactPoints"],
        targets["axes"])

    return ikSolver, targets["groundContactPoints"], ikSolver.r_len


def rawParamsToNumbers(rawParams: Dict[str, any]) -> Dict[str, float]:
    return {key: float(val) for key, val in rawParams.items()}


def convertFromPercentToTranslateValues(
    tx: float, ty: float, tz: float, middle: float, side: float, tibia: float
) -> Vector:
    shiftX = tx * middle
    shiftY = ty * side
    shiftZ = tz * tibia
    return Vector(shiftX, shiftY, shiftZ)

"""
startPose:
    - The pose of the hexapod before we
        rotate and translate the hexapod
    - The body (hexagon) is flat at this point
    - At the very end, we want the hexapod
        to step on the same place as at this pose
        (ie same ground contact points)
"""

def buildStartPose(hipStance: float, legStance: float) -> Dict[str, Dict[str, float]]:
    betaAndGamma = {"beta": legStance, "gamma": -legStance}
    alphas = [0, -hipStance, hipStance, 0, -hipStance, hipStance]

    return {
        positionName: {"alpha": alpha, **betaAndGamma}
        for positionName, alpha in zip(LEG_NAMES, alphas)
    }




def convertIKparams(dimensions, rawIKparams):
    """
    compute for the following:
    startPose:
        - The pose of the hexapod before we
            rotate and translate the hexapod
        - see function buildStartPose() for details
    rotateMatrix:
        - The transformation matrix we would use to
            rotate the hexapod's body
    tVec
        - The translation vector we would use to
            shift the hexapod's body
    """

    IKparams = rawParamsToNumbers(rawIKparams)

    middle, side, tibia = dimensions['middle'], dimensions['side'], dimensions['tibia']
    tx, ty, tz = IKparams['tx'], IKparams['ty'], IKparams['tz']

    tVec = convertFromPercentToTranslateValues(tx, ty, tz, middle, side, tibia)
    hipStance, legStance = IKparams['hipStance'], IKparams['legStance']
    startPose = buildStartPose(hipStance, legStance)

    rx, ry, rz = IKparams['rx'], IKparams['ry'], IKparams['rz']
    rotMatrix = tRotXYZmatrix(rx, ry, rz)

    return tVec, startPose, rotMatrix



def buildHexapodTargets(hexapod, rotMatrix, tVec, rotateThenShift):
    """
    compute the parameters required to solve
    for the hexapod's inverse kinematics
    see IKSolver() class for details.
    """
    # print("hexapod.legs[0]", hexapod.legs[0].ground_contacts)
    groundContactPoints = [leg.maybeGroundContactPoint for leg in hexapod.legs] # [leg.maybeGroundContactPoint for leg in hexapod.legs]

    if rotateThenShift:
        bodyContactPoints = hexapod.body.cloneTrot(rotMatrix).cloneShift(tVec.x, tVec.y, tVec.z).verticesList
    else:
        bodyContactPoints = hexapod.body.cloneShift(tVec.x, tVec.y, tVec.z).cloneTrot(rotMatrix).verticesList

    xAxis = Vector(1, 0, 0).cloneTrot(rotMatrix)
    zAxis = Vector(0, 0, 1).cloneTrot(rotMatrix)

    axes = {'xAxis': xAxis, 'zAxis': zAxis}

    return {'groundContactPoints': groundContactPoints, 'bodyContactPoints': bodyContactPoints, 'axes': axes}


"""
We know 2 point positions that we know are
foot tip ground contact points
(position ie "rightMiddle" etc)
The given `hexapod` is stepping at the `current` points
We want to return a hexapod that is
shifted and rotated it so that those
two points would be stepping at their
respective `target` points
"""

def rotateShiftHexapodgivenPivots(hexapod, points1, points2):
    targetVector = vectorFromTo(points1['target'], points2['target'])
    currentVector = vectorFromTo(points1['current'], points2['current'])

    twistAngleAbsolute = angleBetween(currentVector, targetVector)
    isCCW = isCounterClockwise(currentVector, targetVector, [0, 0, 1])
    twistAngle = twistAngleAbsolute if isCCW else -twistAngleAbsolute
    twistMatrix = tRotZmatrix(twistAngle)

    twistedCurrentPoint1 = points1['current'].get_point_wrt(twistMatrix)
    translateVector = vectorFromTo(twistedCurrentPoint1, points1['target'])

    pivotedHexapod = hexapod.get_point_wrt(twistMatrix).move_xyz(translateVector[0], translateVector[1], 0)

    return pivotedHexapod

"""
given the points where the hexapod should step on
Find two foot tips as pivot points
that we can use to shift and twist the current Hexapod
"""

def findTwoPivotPoints(currentPoints, targetPoints, excludedPositions):
    targetPointsMap = {point['name']: point for point in targetPoints}

    targetPointNames = targetPointsMap.keys()

    currentPoint1, currentPoint2 = None, None
    targetPoint1, targetPoint2 = None, None

    for i in range(len(currentPoints)):
        currentPoint = currentPoints[i]
        currentName = currentPoint['name']
        if currentName in excludedPositions:
            continue

        if currentName in targetPointNames:
            if currentPoint1 is None:
                currentPoint1 = currentPoint
                targetPoint1 = targetPointsMap[currentName]
            else:
                currentPoint2 = currentPoint
                targetPoint2 = targetPointsMap[currentName]
                break

    if currentPoint2 is None:
        return {'foundTwoPoints': False}

    return {
        'points1': {'target': targetPoint1, 'current': currentPoint1},
        'points2': {'target': targetPoint2, 'current': currentPoint2},
        'foundTwoPoints': True,
    }