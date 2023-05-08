from Sprite.scripts.kinematics.hexapod.utils.vector import Vector
from Sprite.scripts.kinematics.hexapod.utils.geometry import (
    vectorFromTo, angleBetween, radians, vectorLength, angleOppositeOfLastSide, isTriangle
)
import math

from Sprite.scripts.kinematics.hexapod.solvers.IKInfo import LegIKInfo

class LinkageIKSolver:
    def __init__(self, legPosition):
        self.info = LegIKInfo.initialized(legPosition)
        self.vectors = {
            "legXaxis": Vector(1, 0, 0, "legXaxis"),
            "parsVector": None,
        }
        self.points = {
            "bodyContactPoint": None,
            "coxiaPoint": None,
            "targetFootTipPoint": None,
        }
        self.dimensions = {
            "coxia": 0,
            "femur": 0,
            "tibia": 0,
            "summa": 0,
            "pars": 0,
        }
        self.angles = {
            "beta": None,
            "gamma": None,
            "rho": None,
        }

    def solve(self, coxia, femur, tibia, summa, rho):
        self.angles["rho"] = rho
        self.dimensions = {"coxia": coxia, "femur": femur, "tibia": tibia, "summa": summa}
        coxiaPoint = Vector(coxia, 0, 0, "coxiaPoint")
        targetFootTipPoint = self._computeTargetFootTipPoint()

        parsVector = vectorFromTo(coxiaPoint, targetFootTipPoint)
        pars = vectorLength(parsVector)

        self.dimensions["pars"] = pars
        self.points.update({"coxiaPoint": coxiaPoint, "targetFootTipPoint": targetFootTipPoint})
        self.vectors.update({"parsVector": parsVector})

        if isTriangle(pars, femur, tibia):
            self._handleCaseTriangleCanForm()
        else:
            self._handleEdgeCase()

        return self

    @property
    def legPosition(self):
        return self.info['legPosition']

    @property
    def beta(self):
        return self.angles["beta"]

    @property
    def gamma(self):
        return self.angles["gamma"]

    @property
    def obtainedSolution(self):
        return self.info['obtainedSolution']

    @property
    def reachedTarget(self):
        return self.info['reachedTarget']

    @property
    def message(self):
        return self.info['message']

    def _computeTargetFootTipPoint(self):
        summa, rho = self.dimensions["summa"], self.angles["rho"]
        px = summa * math.cos(radians(rho))
        pz = -summa * math.sin(radians(rho))
        return Vector(px, 0, pz, "targetLocalFootTipPoint")

    def _handleCaseTriangleCanForm(self):
        femur, pars, tibia = self.dimensions["femur"], self.dimensions["pars"], self.dimensions["tibia"]
        parsVector, legXaxis = self.vectors["parsVector"], self.vectors["legXaxis"]
        targetFootTipPoint = self.points["targetFootTipPoint"]

        theta = angleOppositeOfLastSide(femur, pars, tibia)
        phi = angleBetween(parsVector, legXaxis)
        beta = theta - phi if targetFootTipPoint.z < 0 else theta + phi

        epsi = angleOppositeOfLastSide(femur, tibia, pars)
        femurPointZ = femur * math.sin(radians(beta))

        self.angles['beta'] = beta
        if targetFootTipPoint.z > femurPointZ:
            self.info = LegIKInfo.blocked(self.legPosition)
            return

        self.angles['gamma'] = epsi - 90
        self.info = LegIKInfo.targetReached(self.legPosition)

    def _handleEdgeCase(self):
        pars, tibia, femur = self.dimensions

        if pars + tibia < femur:
            self.info = LegIKInfo.femurTooLong(self.legPosition)
            return
        if pars + femur < tibia:
            # console.log(this.info.legPosition)
            self.info = LegIKInfo.tibiaTooLong(self.legPosition)
            return
        parsVector, legXaxis = self.vectors
        self.angles = {
            **self.angles,
            'beta': -angleBetween(parsVector, legXaxis),
            'gamma': 90
        }
        self.info = LegIKInfo.targetNotReached(self.legPosition)