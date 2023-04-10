from scripts.kinematics.hexapod.solvers.LinkageIKSolver import LinkageIKSolver
from scripts.kinematics.hexapod.solvers.HexapodSupportCheck import HexapodSupportCheck
from scripts.kinematics.hexapod.solvers.IKInfo import IKMessage

from config.ik_settings import (LEG_NAMES, NUMBER_OF_LEGS, POSITION_NAME_TO_AXIS_ANGLE_MAP, MAX_ANGLES)

from scripts.kinematics.hexapod.utils.geometry import (
    vectorFromTo,
    projectedVectorOntoPlane,
    getUnitVector,
    scaleVector,
    addVectors,
    angleBetween,
    vectorLength,
    isCounterClockwise
    )

class IKSolver():

    """
    .......
    Given:
    .......
    {}. Dimensions of each leg { femur, tibia, gamma }
    [] bodyContactPoints
    [] groundContactPoints
       - there are two lists which contains six items each. One item for each leg.
    {} axes { xAxis, zAxis }
       xAxis and zAxis of the rotated hexapod's local coordinate frame wrt
       world coordinate frame.
    .......
    Definition:
    .......
    bodyContactPoint (x, y, z)
    - The point in 3d space which is a vertex of the hexagon.
      This is where the leg is in contact with the body of the hexapod.
    groundContactPoint (x, y, z)
    - The point in 3d space which we want the foot tip of
      the leg to be. Where the leg is in contact with the ground plane of the world.
    .......
    Find:
    .......
    18 angles that represent the pose of the hexapod. Three angles for each leg.
        {
          rightMiddle: { alpha, beta, gamma },
          ...
          rightBack: { alpha, beta, gamma },
        }
        If no solution is found, make sure to explain why.
    .......
    Algorithm:
    .......
    If one of the vertices is below the ground z < 0,
    then there is no solution. Early exit.
    For each leg:
        1. Derive a few properties about the leg given what you already know
           which you'd later (see computeInitialProperties() for details )
           This includes the coxiaPoint. If this coxiaPoint is below the ground
            - then there is no solution. Early exit.
        2. Compute the alpha of this leg. see (computeAlpha())
           If alpha is not within range, then there is no solution. Early exit.
        3. Solve for beta and gamma of this leg (see LegIKSolver module)
          If a problem was encountered within this module, then there is no solution. Early exit.
          If the beta and gamma are not within range, then there is no solution, early exit.
        4. Sometimes the LegIKSolver module would return a solution where the leg
           would not reach the target ground contact point. (this leg would be on the air)
           If the combination of the legs in the air would produce an unstable pose
           (e.g 4 legs are in the air or all left legs are in the air)
           Then there is no solution. Early exit.
           (see also HexapodSupportChecker)
    If no problems are encountered, we have found a solution! Return!
    """
    def __init__(self):
        self.params = {}
        self.partialPose = {}
        self.pose = {}
        self.foundSolution = False
        self.legPositionsOffGround = []
        self.message = IKMessage.initialized


    def solve(self, legDimensions, bodyContactPoints, groundContactPoints, axes):
        self.params = {
            "bodyContactPoints": bodyContactPoints,
            "groundContactPoints": groundContactPoints,
            "axes": axes,
            "legDimensions": legDimensions,
        }

        if self._hasBadVertex(bodyContactPoints):
            return self

        coxia, femur, tibia = legDimensions.values()

        self.r_len = self.getLegCoxiaProjectionLenght(bodyContactPoints[0], groundContactPoints[0], axes["zAxis"])

        for i in range(NUMBER_OF_LEGS):
            legPosition = LEG_NAMES[i]

            known = computeInitialLegProperties(
                bodyContactPoints[i], groundContactPoints[i], axes["zAxis"], coxia)

            # print("known", known)

            if known["coxiaPoint"].z < 0:
                self._handleBadPoint(known["coxiaPoint"])
                return self

            legXaxisAngle = POSITION_NAME_TO_AXIS_ANGLE_MAP[legPosition]

            alpha = computeAlpha(
                known["coxiaUnitVector"],
                legXaxisAngle,
                axes["xAxis"],
                axes["zAxis"],
            )

            if abs(alpha) > MAX_ANGLES["alpha"]:
                self._finalizeFailure(
                    IKMessage.alphaNotInRange(
                        legPosition, alpha, MAX_ANGLES["alpha"]
                    )
                )
                return self

            solvedLegParams = LinkageIKSolver(legPosition).solve(
                coxia, femur, tibia, known["summa"], known["rho"]
            )

            if not solvedLegParams.obtainedSolution:
                self._finalizeFailure(IKMessage.badLeg(solvedLegParams["message"]))
                return self

            if not solvedLegParams.reachedTarget:
                if self._hasNoMoreSupport(legPosition):
                    return self

            self.partialPose[legPosition] = {
                "alpha": alpha,
                "beta": solvedLegParams.beta,
                "gamma": solvedLegParams.gamma,
            }

        self._finalizeSuccess()
        return self


    def getLegCoxiaProjectionLenght(self, bodyContactPoint, groundContactPoint, zAxis):
        bodyToFootVector = vectorFromTo(bodyContactPoint, groundContactPoint)
        coxiaDirectionVector = projectedVectorOntoPlane(bodyToFootVector, zAxis)
        return coxiaDirectionVector

    def computeInitialLegProperties(self, bodyContactPoint, groundContactPoint, zAxis, coxia):
        bodyToFootVector = vectorFromTo(bodyContactPoint, groundContactPoint)
        coxiaDirectionVector = projectedVectorOntoPlane(bodyToFootVector, zAxis)

        coxiaUnitVector = getUnitVector(coxiaDirectionVector)
        coxiaVector = scaleVector(coxiaUnitVector, coxia)

        coxiaPoint = addVectors(bodyContactPoint, coxiaVector)

        rho = angleBetween(coxiaUnitVector, bodyToFootVector)
        summa = vectorLength(bodyToFootVector)

        return {
            "coxiaUnitVector": coxiaUnitVector,
            "coxiaVector": coxiaVector,
            "coxiaPoint": coxiaPoint,
            "rho": rho,
            "summa": summa,
        }

    @property
    def hasLegsOffGround(self):
        return bool(self.legPositionsOffGround)

    def _hasNoMoreSupport(self, legPosition):
        self.legPositionsOffGround.append(legPosition)
        noSupport, reason = HexapodSupportCheck.checkSupport(self.legPositionsOffGround)
        if noSupport:
            message = IKMessage.noSupport(reason, self.legPositionsOffGround)
            self._finalizeFailure(message)
            return True
        return False

    def _handleBadPoint(self, point):
        self._finalizeFailure(IKMessage.badPoint(point))

    def _hasBadVertex(self, bodyContactPoints):
        for i in range(NUMBER_OF_LEGS):
            vertex = bodyContactPoints[i]
            if vertex.z < 0:
                self._handleBadPoint(vertex)
                return True
        return False

    def _finalizeFailure(self, message):
        self.message = message
        self.foundSolution = False

    def _finalizeSuccess(self):
        self.pose = self.partialPose
        self.foundSolution = True
        if not self.hasLegsOffGround:
            self.message = IKMessage.success
            return

        self.message = IKMessage.successLegsOnAir(self.legPositionsOffGround)

def computeInitialLegProperties(bodyContactPoint, groundContactPoint, zAxis, coxia):
    bodyToFootVector = vectorFromTo(bodyContactPoint, groundContactPoint)
    coxiaDirectionVector = projectedVectorOntoPlane(bodyToFootVector, zAxis)

    coxiaUnitVector = getUnitVector(coxiaDirectionVector)
    coxiaVector = scaleVector(coxiaUnitVector, coxia)

    coxiaPoint = addVectors(bodyContactPoint, coxiaVector)

    rho = angleBetween(coxiaUnitVector, bodyToFootVector)
    summa = vectorLength(bodyToFootVector)

    return {
        "coxiaUnitVector": coxiaUnitVector,
        "coxiaVector": coxiaVector,
        "coxiaPoint": coxiaPoint,
        "rho": rho,
        "summa": summa,
    }

def computeAlpha(coxiaVector, legXaxisAngle, xAxis, zAxis):
    sign = -1 if isCounterClockwise(coxiaVector, xAxis, zAxis) else 1
    alphaWrtHexapod = sign * angleBetween(coxiaVector, xAxis)
    alpha = (alphaWrtHexapod - legXaxisAngle) % 360

    if alpha > 180:
        return alpha - 360
    if alpha < -180:
        return alpha + 360

    # THERE IS A BUG HERE SOMEWHERE
    if alpha == 180 or alpha == -180:
        return 0

    return alpha