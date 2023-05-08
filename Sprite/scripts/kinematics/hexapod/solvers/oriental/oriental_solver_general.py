"""
A more general algorithm to account for the cases
that are not handled correctly by the orientSolverSpecific.
- Only used by the kinematics-page of the app.
- Can be optimized or replaced if a more elegant
algorithm is available.
............
  OVERVIEW
............
Find:
- The orientation of the hexapod (normal axis of the hexapod body plane)
- distance of hexapod center of gravity to the ground plane (height)
- All the legs which are in contact in the ground
How?
We have 18 points total.
(6 legs, three possible points per leg (femurPoint))
We have a total of 540 combinations
- get three legs out of six (20 combinations)
  - we have three possible points for each leg,
        (coxiaPoint, femurPoint, footTip),
        that's 27 (3^3) combinations
  -  27 * 20 is 540
For each combination:
    1. Check if stable. If not, try the next combination
      - Check if the projection of the center of gravity to the plane
        defined by the three points lies inside the triangle,
        if not stable, try the next combination
    2. Get the HEIGHT and normal of the height and normal of the triangle plane
        (We need this for the next part)
    3. For each of the three legs, check if the two other points on the leg is not
        lower than HEIGHT, (6 points total)
        if condition if broken, try the next combination.
    4. For each of the three other legs, check if all points (3 points of each leg)
        are not lower than HEIGHT
        if this condition is broken, try the next combination. (9 points total)
    5. If no condition is violated, then this is good, return this!
"""
import random

from Sprite.scripts.kinematics.hexapod.solvers.oriental.oriental_helper import (
    SOME_LEG_ID_TRIOS, ADJACENT_LEG_ID_TRIOS, isStable, isLower, findLegsOnGround
)
from Sprite.scripts.kinematics.hexapod.utils.geometry import dot, get_normal_of_three_points

# def make_joint_index_trios():
#     return [[i, j, k] for i in range(3, 0, -1) for j in range(3, 0, -1) for k in range(3, 0, -1)]

JOINT_INDEX_TRIOS = [[i, j, k] for i in range(3, 0, -1) for j in range(3, 0, -1) for k in range(3, 0, -1)]

def shuffleArray(array):
    for i in range(len(array)-1, 0, -1):
        j = random.randint(0, i)
        array[i], array[j] = array[j], array[i]
    return array

def computeOrientationProperties(legsNoGravity, flags = { "shuffle": False }):
    someLegTrios = shuffleArray(SOME_LEG_ID_TRIOS.copy()) if flags["shuffle"] else SOME_LEG_ID_TRIOS.copy()
    legIndexTrios = someLegTrios + ADJACENT_LEG_ID_TRIOS
    fallback = None


    for threeLegIndices in legIndexTrios:
        threeLegs, otherThreeLegs = getTwoLegSets(threeLegIndices, legsNoGravity)
        # print("threeLegs, otherThreeLegs", threeLegs, otherThreeLegs)

        for threeJointIndices in JOINT_INDEX_TRIOS:
            p0, p1, p2 = getThreePoints(threeLegs, threeJointIndices)

            if not isStable(p0, p1, p2):
                continue

            normal = get_normal_of_three_points(p0, p1, p2) # "normalVector"
            height = -dot(normal, p0)

            if anotherPointOfSameLegIsLower(threeLegs, threeJointIndices, normal, height):
                continue

            if anotherPointofOtherLegsIsLower(otherThreeLegs, normal, height):
                continue

            if height == 0:
                if fallback is None:
                    fallback = {"p0": p0, "p1": p1, "p2": p2, "normal": normal, "height": height}
                continue

            groundLegsNoGravity = findLegsOnGround(legsNoGravity, normal, height)
            return {"nAxis": normal, "height": height, "groundLegsNoGravity": groundLegsNoGravity}

    if fallback is None:
        return None

    return {
        "nAxis": fallback["normal"],
        "height": fallback["height"],
        "groundLegsNoGravity": findLegsOnGround(legsNoGravity, fallback["normal"], fallback["height"]),
    }

def getThreePoints(threeLegs, threeJointIndices):
    return [leg.allPointsList[jointId] for leg, jointId in zip(threeLegs, threeJointIndices)]

def getTwoLegSets(threeLegIndices, sixLegs):
    threeLegs = [sixLegs[n] for n in threeLegIndices]
    otherThreeLegIndices = [n for n in range(6) if n not in threeLegIndices]
    otherThreeLegs = [sixLegs[n] for n in otherThreeLegIndices]
    return threeLegs, otherThreeLegs # {"threeLegs": threeLegs, "otherThreeLegs": otherThreeLegs}

def anotherPointOfSameLegIsLower(threeLegs, threeJointIndices, normal, height):
    for leg, jointIndex in zip(threeLegs, threeJointIndices):
        for otherPointIndex, otherPoint in enumerate(leg.allPointsList):
            notBodyContact = otherPointIndex != 0
            notItself = otherPointIndex != jointIndex
            if notBodyContact and notItself and isLower(otherPoint, normal, height):
                return True
    return False

def anotherPointofOtherLegsIsLower(otherThreeLegs, normal, height):
    for leg in otherThreeLegs:
        has_lower = any(isLower(point, normal, height) for point in leg.allPointsList[1:])
        if has_lower:
            return True
    return False