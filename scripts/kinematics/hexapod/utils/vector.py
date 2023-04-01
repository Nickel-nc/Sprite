
class Vector:
    def __init__(self, x, y, z, name="no-name-point", id="no-id-point"):
        self.x = x
        self.y = y
        self.z = z
        self.name = name
        self.id = id

    def newTrot(self, transformMatrix, name="unnamed-point", id="no-id"):
        # given point `point` location wrt a local axes
        # coordinate frame
        # find point in a global axes coordinate frame
        # where the local axes wrt the global frame is defined by
        # parameter transformMatrix
        r0, r1, r2 = transformMatrix[:3]
        r00, r01, r02, tx = r0
        r10, r11, r12, ty = r1
        r20, r21, r22, tz = r2

        newX = self.x * r00 + self.y * r01 + self.z * r02 + tx
        newY = self.x * r10 + self.y * r11 + self.z * r12 + ty
        newZ = self.x * r20 + self.y * r21 + self.z * r22 + tz
        return Vector(newX, newY, newZ, name, id)

    def cloneTrot(self, transformMatrix):
        return self.newTrot(transformMatrix, self.name, self.id)

    def cloneShift(self, tx, ty, tz):
        return Vector(self.x + tx, self.y + ty, self.z + tz, self.name, self.id)

    def cloneTrotShift(self, transformMatrix, tx, ty, tz):
        return self.cloneTrot(transformMatrix).cloneShift(tx, ty, tz)

    def toMarkdownString(self):
        x = "{:.2f}".format(self.x)
        y = "{:.2f}".format(self.y)
        z = "{:.2f}".format(self.z)
        markdownString = f"{self.name}\n\n(x: {x}, y: {y}, z: {z})"
        return markdownString