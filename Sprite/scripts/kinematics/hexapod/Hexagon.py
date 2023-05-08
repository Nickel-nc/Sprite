from Sprite.config.ik_settings import LEG_NAMES
from Sprite.scripts.kinematics.hexapod.utils.vector import Vector


class Hexagon:
    def __init__(self, dimensions, flags={"hasNoPoints": False}):
        self.dimensions = dimensions

        if flags["hasNoPoints"]:
            return

        front, middle, side = dimensions["front"], dimensions["middle"], dimensions["side"]
        vertexX = [middle, front, -front, -middle, -front, front]
        vertexY = [0, side, side, 0, -side, -side]

        self.verticesList = [
            Vector(vertexX[i], vertexY[i], 0, f"{LEG_NAMES[i]}Vertex", i)
            for i in range(len(LEG_NAMES))
        ]
        self.head = Vector(0, side, 0, "headPoint", 7)
        self.cog = Vector(0, 0, 0, "centerOfGravityPoint", 6)

    @property
    def closedPointsList(self):
        return self.verticesList + [self.verticesList[0]]

    @property
    def allPointsList(self):
        return self.verticesList + [self.cog, self.head]

    def cloneTrotShift(self, transformMatrix, tx, ty, tz):
        return self._doTransform("cloneTrotShift", transformMatrix, tx, ty, tz)

    def cloneTrot(self, transformMatrix):
        return self._doTransform("cloneTrot", transformMatrix)

    def cloneShift(self, tx, ty, tz):
        return self._doTransform("cloneShift", tx, ty, tz)

    def _doTransform(self, transformFunction, *args):
        clone = Hexagon(self.dimensions, { "hasNoPoints": True })
        clone.cog = getattr(self.cog, transformFunction)(*args)
        clone.head = getattr(self.head, transformFunction)(*args)
        clone.verticesList = [getattr(point, transformFunction)(*args) for point in self.verticesList]
        return clone