from scripts.kinematics.hexapod.utils.geometry import dot, get_normal_of_three_points
from scripts.kinematics.hexapod.solvers.oriental.oriental_helper import (
    SOME_LEG_ID_TRIOS, ADJACENT_LEG_ID_TRIOS,
    isStable, findLegsOnGround, isLower)

LEG_ID_TRIOS = SOME_LEG_ID_TRIOS + ADJACENT_LEG_ID_TRIOS

"""
  .................
  COMPUTE ORIENTATION PROPERTIES (TYPE: SPECIFIC)
  .................
  Given: 1. A list of legs with known pose and
            its points wrt the hexapod body is known
         2. The legs which are in contact with the ground
            is known
  Find: 1. Normal vector of the plane defined by foot tip of
           legs on the ground wrt the hexapod body plane
        2. Distance of the hexapod body plane to the plane
           defined by the foot tips on the ground
        3. Which legs are on the ground
"""
def computeOrientationProperties(legsNoGravity):
    result = computePlaneProperties(legsNoGravity)
    if result is None:
        return None

    groundLegsNoGravity = findLegsOnGround(
        legsNoGravity,
        result['normal'],
        result['height']
    )

    return {
        'nAxis': result['normal'],
        'height': result['height'],
        'groundLegsNoGravity': groundLegsNoGravity
    }

"""
 .................
 COMPUTE PLANE PROPERTIES
 .................
"""
def computePlaneProperties(legs):
    maybeGroundContactPoints = [leg.maybeGroundContactPoint for leg in legs]

    for legTrio in LEG_ID_TRIOS:
        p0, p1, p2 = [maybeGroundContactPoints[j] for j in legTrio]

        if not isStable(p0, p1, p2):
            continue

        normal = get_normal_of_three_points(p0, p1, p2)  # , "normalVector"

        """
           cog *   ^ (normal_vector) ----
                \  |                  |
                 \ |               -height
                  \|                  |
                   V p0 (foot_tip) ---v--
         
           using p0, p1 or p2 should yield the same height
         
         """
        height = -dot(normal, p0)

        otherTrio = [j for j in range(6) if j not in legTrio]
        otherFootTips = [maybeGroundContactPoints[j] for j in otherTrio]

        noOtherLegLower = all(not isLower(footTip, normal, height) for footTip in otherFootTips)

        if noOtherLegLower:
            return {'normal': normal, 'height': height}

    return None