import sys
sys.path.append("/")

from config.ik_settings import LEG_NAMES, POSITION_NAME_TO_ID_MAP, DEFAULT_POSE
from scripts.kinematics.hexapod.utils.geometry import matrixToAlignVectorAtoB, tRotZmatrix

from scripts.kinematics.hexapod.utils.vector import Vector
from scripts.kinematics.hexapod.Hexagon import Hexagon
from scripts.kinematics.hexapod.Linkage import Linkage

from scripts.kinematics.hexapod.solvers.oriental import oriental_solver_specific as oSolverSpecific, \
    oriental_solver_general as oSolverGeneral
from scripts.kinematics.hexapod.solvers.twistSolver import simpleTwist, mightTwist, complexTwist



DEFAULT_LOCAL_AXES = {
    "xAxis": Vector(1, 0, 0, "hexapodXaxis"),
    "yAxis": Vector(0, 1, 0, "hexapodYaxis"),
    "zAxis": Vector(0, 0, 1, "hexapodZaxis"),
}

def transformLocalAxes(localAxes, twistMatrix):
    return {
        "xAxis": localAxes["xAxis"].cloneTrot(twistMatrix),
        "yAxis": localAxes["yAxis"].cloneTrot(twistMatrix),
        "zAxis": localAxes["zAxis"].cloneTrot(twistMatrix),
    }

def buildLegsList(bodyContactPoints, pose, legDimensions):
    """
    bodyContactPoints:  (6)

    :param bodyContactPoints:
        Vector class objects (with params x,y,z, name, id) for each of 6 legs
    :param pose:
        {'rightMiddle': {'alpha': 0, 'beta': 0.0, 'gamma': -0.0}, 'rightFront': {'alpha': -25.0, 'beta': 0.0, 'gamma': -0.0} ...}
    :param legDimensions:
        Leg Dimentions (i.e. L1, L2, L3)
        {'coxia': 45, 'femur': 75, 'tibia': 135}
    :return:
    """

    legsList = []
    for position in LEG_NAMES:
        index = POSITION_NAME_TO_ID_MAP[position]
        legsList.append(Linkage(legDimensions, position, bodyContactPoints[index], pose[position]))
    return legsList

"""const buildLegsList = (bodyContactPoints, pose, legDimensions) =>
    POSITION_NAMES_LIST.map(
        (position, index) =>
            new Linkage(legDimensions, position, bodyContactPoints[index], pose[position])
    )"""

def hexapodErrorInfo():
    return {
        "isAlert": True,
        "subject": "Unstable position.",
        "body": "error in solving for orientation ",
    }

def hexapodSuccessInfo():
    return {
        "isAlert": False,
        "subject": "Success!",
        "body": "Stable orientation found.",
    }


"""
............................
 Virtual Hexapod properties
............................
Property types:
{}: hash map / object / dictionary
[]: array / list
##: number
"": string
{} this.dimensions: {front, side, middle, coxia, femur, tibia}
{} this.pose: A hash mapping the position name to a hash map of three angles
    which define the pose of the hexapod
    i.e. { rightMiddle: {alpha, beta, gamma },
           leftBack: { alpha, betam gamma },
             ...
         }
[] this.body: A hexagon object
    which contains all the info of the 8 points defining the hexapod body
    (6 vertices, 1 head, 1 center of gravity)
[] this.legs: A list which has elements that point to six Linkage objects.
    The order goes counter clockwise starting from the first element
    which is the rightMiddle leg up until the last element which is rightBack leg.
    Each leg contains the points that define that leg
    as well as other properties pertaining it (see Linkage class)
[] this.legPositionsOnGround: A list of the leg positions (strings)
    that are known to be in contact with the ground
{} this.localAxes: A hash containing three vectors defining the local
    coordinate frame of the hexapod wrt the world coordinate frame
    i.e. {
        xAxis: {x, y, z, name="hexapodXaxis", id="no-id"},
        yAxis: {x, y, z, name="hexapodYaxis", id="no-id"},
        zAxis: {x, y, z, name="hexapodZaxis", id="no-id"},
    }
....................
(virtual hexapod derived properties)
....................
{} this.bodyDimensions: { front, side, middle }
{} this.legDimensions: { coxia, femur, tibia }
## this.distanceFromGround: A number which is the perpendicular distance
    from the hexapod's center of gravity to the ground plane
{} this.cogProjection: a point that represents the projection
    of the hexapod's center of gravity point to the ground plane
    i.e { x, y, z, name="centerOfGravityProjectionPoint", id="no-id"}
[] this.groundContactPoints: a list whose elements point to points
    from the leg which contacts the ground.
    This list can contain 6 or less elements.
    (It can have a length of 3, 4, 5 or 6)
    i.e. [
        { x, y, z, name="rightMiddle-femurPoint", id="0-2"},
        { x, y, z, name="leftBack-footTipPoint", id=4-3},
         ...
    ]
"""


class Hexapod:
    def __init__(self, dimensions, pose, flags={"hasNoPoints": False, "assumeKnownGroundPoints": False, "wontRotate": False}):

        self.dimensions = dimensions
        self.pose = pose
        self.body = None
        self.legs = None
        self.legPositionsOnGround = None
        self.localAxes = None
        self.foundSolution = None

        if flags["hasNoPoints"]:
            return

        flat_hexagon = Hexagon(self.bodyDimensions)
        legs_no_gravity = buildLegsList(flat_hexagon.verticesList, self.pose, self.legDimensions)

        if flags["assumeKnownGroundPoints"]:
            solved = oSolverSpecific.computeOrientationProperties(legs_no_gravity)
        else:
            solved = oSolverGeneral.computeOrientationProperties(legs_no_gravity)

        if solved is None:
            self.foundSolution = False
            return

        self.foundSolution = True
        self.legPositionsOnGround = [leg.position for leg in solved['groundLegsNoGravity']]

        transform_matrix = matrixToAlignVectorAtoB(solved['nAxis'], DEFAULT_LOCAL_AXES['zAxis'])

        self.legs = [leg.cloneTrotShift(transform_matrix, 0, 0, solved['height']) for leg in legs_no_gravity]
        self.body = flat_hexagon.cloneTrotShift(transform_matrix, 0, 0, solved['height'])
        self.localAxes = transformLocalAxes(DEFAULT_LOCAL_AXES, transform_matrix)

        if flags["wontRotate"]:
            return
        if all([leg.pose['alpha'] == 0 for leg in self.legs]):
            return

        twist_angle = simpleTwist(solved['groundLegsNoGravity'])
        if twist_angle != 0:
            self._twist(twist_angle)
            return

        if mightTwist(solved['groundLegsNoGravity']):
            self._handleComplexTwist(flat_hexagon.verticesList)

    @property
    def distanceFromGround(self):
        return self.body.cog.z

    @property
    def cogProjection(self):
        return Vector(self.body.cog.x, self.body.cog.y, 0, "centerOfGravityProjectionPoint")

    @property
    def info(self):
        return hexapodSuccessInfo() if self.foundSolution else hexapodErrorInfo()

    @property
    def bodyDimensions(self):
        front, middle, side = self.dimensions["front"], self.dimensions["middle"], self.dimensions["side"]
        return {"front": front, "middle": middle, "side": side}

    @property
    def legDimensions(self):
        coxia, femur, tibia = self.dimensions["coxia"], self.dimensions["femur"], self.dimensions["tibia"]
        return {"coxia": coxia, "femur": femur, "tibia": tibia}

    @property
    def groundContactPoints(self):
        return [self.legs[POSITION_NAME_TO_ID_MAP[position]].maybeGroundContactPoint for position in self.legPositionsOnGround]

    def cloneTrot(self, transform_matrix):
        body = self.body.cloneTrot(transform_matrix)
        legs = [leg.cloneTrot(transform_matrix) for leg in self.legs]
        local_axes = transformLocalAxes(self.localAxes, transform_matrix)
        return self._buildClone(body, legs, local_axes)

    def cloneShift(self, tx, ty, tz):
        body = self.body.cloneShift(tx, ty, tz)
        legs = [leg.cloneShift(tx,ty,tz) for leg in self.legs]
        return self._buildClone(body, legs, self.localAxes)

    def _buildClone(self, body, legs, localAxes):
        clone = Hexapod(self.dimensions, self.pose, {"hasNoPoints": True})
        clone.body = body
        clone.legs = legs
        clone.localAxes = localAxes
        clone.legPositionsOnGround = self.legPositionsOnGround
        clone.foundSolution = self.foundSolution
        return clone

    def _handleComplexTwist(self, verticesList):
        defaultLegs = buildLegsList(verticesList, DEFAULT_POSE, self.legDimensions)
        defaultPoints = [leg.cloneShift(0, 0, self.dimensions.tibia).maybeGroundContactPoint for leg in defaultLegs]

        currentPoints = self.groundContactPoints
        twistAngle = complexTwist(currentPoints, defaultPoints)

        if twistAngle != 0:
            self._twist(twistAngle)

    def _twist(self, twistAngle):
        twistMatrix = tRotZmatrix(twistAngle)
        self.body = self.body.cloneTrot(twistMatrix)
        self.legs = [leg.cloneTrot(twistMatrix) for leg in self.legs]
        self.localAxes = transformLocalAxes(self.localAxes, twistMatrix)