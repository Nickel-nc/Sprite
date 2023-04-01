"""
  ..................
   LINKAGE
  ..................
     p0 *----* p1
              \       * p0 = origin / bodyContactPoint
               * p2   * p1 = coxiaPoint
               |      * p2 = femurPoint
               * p3   * p3 = tibiaPoint / footTipPoint
                      * coxiaVector = vector from p0 to p1
     localZ           * femurVector = vector from p1 to p2
     |  localY        * tibiaVector = vector from p2 to p3
     | /
     |/
     |------ localX   * LegPointId = {legId}-{pointId}
             2           1       * legId - legName     - localXaxisAngle
              \   head  /        *  0    - rightMiddle - 0
               *---*---*         *  1    - rightFront  - 45
              /    |    \        *  2    - leftFront   - 135
             /     |     \       *  3    - leftMiddle  - 180
         3 -*-----cog-----*- 0   *  4    - leftBack    - 225
             \     |     /       *  5    - rightBack   - 315
              \    |    /
               *---*---*          ^ hexapodY
              /         \         |
             4           5        *---> hexapodX
                                 /
                                * hexapodZ
                     * localXaxisAngle = angle made by hexapodXaxis and localXaxis
                     * alpha = angle made by coxia Vector and localXaxis
             p2      * beta = angle made by coxiaVector and femurVector
             *              = angle made by points p2, p1 and pxPrime
            / \
       *---*---\---> pxPrime
      p0   p1   * p3
      p0   p1         * gamma = angle made by vector perpendicular to
       *---*                    coxiaVector and tibiaVector
           | \                = angle made by points pzPrime, p1, p3
           |  \
           V   * p3
          pzPrime
  ..................
   LINKAGE PROPERTIES
  ..................
  {} this.dimensions: { coxia, femur, tibia }
  {} this.pose: { alpha, beta, gamma }
  "" this.position: "rightMiddle" from POSITION_NAMES_LIST or "linkage-position-not-defined"
  [] this.allPointsList: A list pointing to each of the four points in the map
      which the first element being the bodyContactPoint, the last element being the footTipPoint
      [
          {x, y, z, id: "5-0", name: "rightBack-bodyContactPoint"},
          {x, y, z, id: "5-1", name: "rightBack-coxiaPoint"},
          {x, y, z, id: "5-2", name: "rightBack-femurPoint"},
          {x, y, z, id: "5-3", name: "rightBack-footTipPoint"},
      ]
      each id is prefixed with 5 because the leg point id corresponding to "rightBack"
      position is 5.
  ....................
  (linkage derived properties)
  ....................
  {} this.maybeGroundContactPoint: The point which probably is the one in contact
      with the ground, but not necessarily the case (no guarantees)
  "" this.name: "{position}Leg" e.g. "rightMiddleLeg"
  "" this.id : a number from 0 to 5 corresponding to a particular position
"""
import numpy as np
from functools import reduce
from scripts.kinematics.hexapod.utils.geometry import tRotYmatrix, tRotZmatrix
from config.ik_settings import LEG_POINT_TYPES_LIST, POSITION_NAME_TO_ID_MAP, POSITION_NAME_TO_AXIS_ANGLE_MAP
from scripts.kinematics.hexapod.utils.vector import Vector

class Linkage():
    def __init__(self, dimensions, position, originPoint = Vector(0,0,0), pose = {"alpha": 0, "beta": 0, "gamma": 0}, flags = {"hasNoPoints": False}):
        self.dimensions = dimensions
        self.pose = pose
        self.position = position

        if flags["hasNoPoints"]:
            return

        self.allPointsList = self._computePoints(pose, originPoint)

    @property
    def bodyContactPoint(self):
        return self.allPointsList[0]

    @property
    def coxiaPoint(self):
        return self.allPointsList[1]

    @property
    def femurPoint(self):
        return self.allPointsList[2]

    @property
    def footTipPoint(self):
        return self.allPointsList[3]

    @property
    def id(self):
        return POSITION_NAME_TO_ID_MAP[self.position]

    @property
    def name(self):
        return f"{self.position}Leg"

    @property
    def maybeGroundContactPoint(self):
        reversedList = self.allPointsList[::-1]
        testPoint = reversedList[0]
        maybeGroundContactPoint = reduce(lambda testPoint, point: point if point.z < testPoint.z else testPoint, reversedList, testPoint)
        return maybeGroundContactPoint

    def cloneTrotShift(self, transformMatrix, tx, ty, tz):
        return self._doTransform("cloneTrotShift", transformMatrix, tx, ty, tz)

    def cloneTrot(self, transformMatrix):
        return self._doTransform("cloneTrot", transformMatrix)

    def cloneShift(self, tx, ty, tz):
        return self._doTransform("cloneShift", tx, ty, tz)

    def _doTransform(self, transformFunction, *args):
        newPointsList = [oldPoint.__getattribute__(transformFunction)(*args) for oldPoint in self.allPointsList]
        return self._buildClone(newPointsList)

    def _buildClone(self, allPointsList):
        clone = Linkage(
            self.dimensions,
            self.position,
            self.bodyContactPoint,
            self.pose,
            {"hasNoPoints": True}
        )

        clone.allPointsList = allPointsList
        return clone

    """
    structure of pointNameIds
         pointNameIds = [
           { name: "{legPosition}-bodyContactPoint", id: "{legId}-0" },
           { name: "{legPosition}-coxiaPoint", id: "{legId}-1" },
           { name: "{legPosition}-femurPoint", id: "{legId}-2" },
           { name: "{legPosition}-footTipPoint", id: "{legId}-3" }
           ]
    """

    def _buildNameId(self, pointName, id):
        return {"name": f"{self.position}-{pointName}", "id": f"{self.id}-{id}"}

    def _buildPointNameIds(self):
        return [self._buildNameId(pointType, index) for index, pointType in enumerate(LEG_POINT_TYPES_LIST)]

    def _computePointsWrtBodyContact(self, beta, gamma):
        matrix01 = tRotYmatrix(-beta, self.dimensions['coxia'], 0, 0)
        matrix12 = tRotYmatrix(90 - gamma, self.dimensions['femur'], 0, 0)
        matrix23 = tRotYmatrix(0, self.dimensions['tibia'], 0, 0)
        matrix02 = np.dot(matrix01, matrix12)
        matrix03 = np.dot(matrix02, matrix23)

        originPoint = Vector(0, 0, 0)

        localPoints = [
            originPoint, # bodyContactPoint
            originPoint.cloneTrot(matrix01), # coxiaPoint
            originPoint.cloneTrot(matrix02),
            originPoint.cloneTrot(matrix03)
            ]
        return localPoints

    def _computePointsWrtHexapodCog(self, alpha, originPoint, localPoints, pointNameIds):
        zAngle = POSITION_NAME_TO_AXIS_ANGLE_MAP[self.position] + alpha

        twistMatrix = tRotZmatrix(
            zAngle,
            originPoint.x,
            originPoint.y,
            originPoint.z
        )

        allPointsList = []
        for index, localPoint in enumerate(localPoints):
            name = pointNameIds[index]["name"]
            id = pointNameIds[index]["id"]
            point = localPoint.newTrot(twistMatrix, name, id)
            allPointsList.append(point)

        return allPointsList


    """
    Example of allPointsList =  [
        {"x": x, "y": y, "z": z, "id": "5-0", "name": "rightBack-bodyContactPoint"},
        {"x": x, "y": y, "z": z, "id": "5-1", "name": "rightBack-coxiaPoint"},
        {"x": x, "y": y, "z": z, "id": "5-2", "name": "rightBack-femurPoint"},
        {"x": x, "y": y, "z": z, "id": "5-3", "name": "rightBack-footTipPoint"},
    ]
    x, y, z are numbers
    """
    def _computePoints(self, pose, originPoint):
        alpha, beta, gamma = pose["alpha"], pose["beta"], pose["gamma"]
        pointNameIds = self._buildPointNameIds()

        localPoints = self._computePointsWrtBodyContact(beta, gamma)

        allPointsList = self._computePointsWrtHexapodCog(
            alpha, originPoint, localPoints, pointNameIds
        )
        # for i in range(len(allPointsList)):
        #     print(allPointsList[i].x,allPointsList[i].id, allPointsList[i].name)
        return allPointsList