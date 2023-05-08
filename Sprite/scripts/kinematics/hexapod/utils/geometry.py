import numpy as np
from Sprite.scripts.kinematics.hexapod.utils.vector import Vector

def vectorFromTo(a,b):
    return Vector(b.x - a.x, b. y - a.y, b.z - a.z)

def radians(thetaDegrees):
    return (thetaDegrees * np.pi) / 180

def angleOppositeOfLastSide(a, b, c):
    if a == 0 or b == 0:
        return None
    return np.degrees(np.arccos((a*a + b*b - c*c) / (2*a*b)))

def isTriangle(a, b, c):
    return a + b > c and a + c > b and b + c > a

def getUnitVector(v):
    return scaleVector(v, 1 / vectorLength(v))

def addVectors(a, b):
    return Vector(a.x + b.x, a.y + b.y, a.z + b.z)

def cross(a, b):
    x = a.y * b.z - a.z * b.y
    y = a.z * b.x - a.x * b.z
    z = a.x * b.y - a.y * b.x
    return Vector(x, y, z)

def scaleVector(v,d):
    return Vector(d * v.x, d * v.y, d * v.z)

def get_normal_of_three_points(a, b, c):
    ab = vectorFromTo(a, b)
    ac = vectorFromTo(a, c)
    n = cross(ab, ac)
    len_n = vectorLength(n)
    unit_n = scaleVector(n, 1 / len_n)

    return unit_n

def isCounterClockwise(a, b, n):
    return dot(a, cross(b, n)) > 0

def vectorLength(v):
    return np.sqrt(dot(v, v))

def dot(a, b):
    return a.x * b.x + a.y * b.y + a.z * b.z

def degrees(thetaRadians):
    return thetaRadians * 180 / np.pi

def getSinCos(theta):
    return [np.sin(np.radians(theta)), np.cos(np.radians(theta))]

def tRotXmatrix(theta, tx = 0, ty = 0, tz = 0) -> np.array:
    s, c = getSinCos(theta)
    return np.array([
        [1, 0, 0, tx],
        [0, c, -s, ty],
        [0, s, c, tz],
        [0, 0, 0, 1]
    ])


def tRotYmatrix(theta, tx = 0, ty = 0, tz = 0):
    s, c = getSinCos(theta)
    return np.array([
        [c, 0, s, tx],
        [0, 1, 0, ty],
        [-s, 0, c, tz],
        [0, 0, 0, 1]
    ])


def tRotZmatrix(theta, tx = 0, ty = 0, tz = 0):
    s, c = getSinCos(theta)
    return np.array([
        [c, -s, 0, tx],
        [s, c, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ])

def tRotXYZmatrix(xTheta, yTheta, zTheta):

    rx = tRotXmatrix(xTheta)
    ry = tRotYmatrix(yTheta)
    rz = tRotZmatrix(zTheta)
    rxy = np.dot(rx, ry)  #  multiply4x4(rx, ry)
    rxyz = np.dot(rxy, rz)  #  multiply4x4(rxy, rz)
    return rxyz

def acosDegrees(ratio):
    thetaRadians = np.arccos(ratio)
    if np.isnan(thetaRadians):
        return 0
    return degrees(thetaRadians)

def angleBetween(a, b):
    if (vectorLength(a) == 0 or vectorLength(b) == 0):
        return 0
    cosTheta = dot(a, b) / np.sqrt(dot(a, a) * dot(b, b))
    return acosDegrees(cosTheta)

def skew(p):
    return np.array([[0, -p.z, p.y], [p.z, 0, -p.x], [-p.y, p.x, 0]])


def projectedVectorOntoPlane(u, n):
    s = dot(u, n) / dot(n, n)
    tempVector = scaleVector(n, s)
    return vectorFromTo(tempVector, u)


def multiply_4x4(matrixA, matrixB):
    result_matrix = np.zeros((4, 4))

    for i in range(4):
        for j in range(4):
            result_matrix[i][j] = (
                matrixA[i][0] * matrixB[0][j] +
                matrixA[i][1] * matrixB[1][j] +
                matrixA[i][2] * matrixB[2][j] +
                matrixA[i][3] * matrixB[3][j]
            )

    return result_matrix

def matrixToAlignVectorAtoB(a,b):
    v = cross(a, b)
    s = vectorLength(v)
    # When angle between a and b is zero or 180 degrees
    # cross product is 0, R = I
    if (s == 0):
        return np.identity(4)

    c = dot(a, b)
    vx = skew(v)
    d = (1 - c) / (s * s)
    vx2 = np.dot(vx, vx) # multiply matrix 4x4
    dMatrix = np.full((4,4), d)
    dvx2 = np.multiply(vx2, dMatrix)
    temp = np.add(np.identity(4), vx)
    transformMatrix = np.add(temp, dvx2)

    return transformMatrix


