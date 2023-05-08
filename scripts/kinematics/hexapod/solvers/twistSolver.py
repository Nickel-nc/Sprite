import numpy as np

from config.ik_settings import POSITION_NAME_TO_ID_MAP
from scripts.kinematics.hexapod.utils.geometry import degrees


def mightTwist(legsOnGround):
    negativeAlphaCount = 0
    positiveAlphaCount = 0

    for leg in legsOnGround:
        pointType = leg.maybeGroundContactPoint.name.split("-")[1]

        footTipIsOnGround = pointType == "footTipPoint"
        changedAlpha = leg.pose['alpha'] != 0

        if footTipIsOnGround and changedAlpha:
            if leg.pose['alpha'] > 0:
                positiveAlphaCount += 1
            else:
                negativeAlphaCount += 1

    return positiveAlphaCount >= 3 or negativeAlphaCount >= 3


def complexTwist(currentPoints, defaultPoints):
    currentSamePoint = next((point for point in currentPoints if point.name.split("-")[1] == "footTipPoint"), None)

    if currentSamePoint is None:
        return 0

    samePointPosition = currentSamePoint.name.split("-")[0]
    samePointIndex = POSITION_NAME_TO_ID_MAP[samePointPosition]
    defaultSamePoint = defaultPoints[samePointIndex]

    thetaRadians = np.arctan2(defaultSamePoint.y, defaultSamePoint.x) - np.arctan2(currentSamePoint.y, currentSamePoint.x)

    return degrees(thetaRadians)


def simpleTwist(groundLegsNoGravity):
    firstLeg = groundLegsNoGravity[0]

    allSameAlpha = all(leg.pose['alpha'] == firstLeg.pose['alpha'] for leg in groundLegsNoGravity)


    if not allSameAlpha:
        return 0

    allPointTypes = [leg.maybeGroundContactPoint.name.split("-")[1] for leg in groundLegsNoGravity]
    firstPointType = allPointTypes[0]

    allPointsSameType = all(pointType == firstPointType for pointType in allPointTypes)

    if not allPointsSameType:
        return 0

    if firstPointType in ["coxiaPoint", "bodyContactPoint"]:
        return 0

    if firstPointType == "femurPoint":
        bodyContactPoint = next((leg.maybeGroundContactPoint for leg in groundLegsNoGravity if leg.maybeGroundContactPoint.name.split("-")[1] == "bodyContactPoint"), None)
        if bodyContactPoint is None or bodyContactPoint.z == groundLegsNoGravity[0].maybeGroundContactPoint.z:
            return 0

    # at this point, all ground points are of type footTipPoint
    return mightTwist(groundLegsNoGravity)