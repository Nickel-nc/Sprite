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

        for i in range(NUMBER_OF_LEGS):
            legPosition = LEG_NAMES[i]

            known = computeInitialLegProperties(
                bodyContactPoints[i], groundContactPoints[i], axes["zAxis"], coxia)

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

    # ❗❗❗THIS IS A HACK ❗❗❗
    # THERE IS A BUG HERE SOMEWHERE, FIND IT
    if alpha == 180 or alpha == -180:
        return 0

    return alpha