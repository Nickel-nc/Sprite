from scripts.kinematics.hexapod.utils.geometry import dot, cross, vectorFromTo
from scripts.kinematics.hexapod.utils.points import Vector

def isLower(point, normal, height, tol=1):
    return -dot(normal, point) > height + tol

def findLegsOnGround(legs, normal, height):
    def sameHeight(point, normal, height, tol=1):
        _height = -dot(normal, point)
        return abs(height - _height) <= tol

    legsOnGround = []
    for leg in legs:
        reversedPoints = leg.allPointsList[1:][::-1]
        onGround = any(sameHeight(point, normal, height) for point in reversedPoints)
        if onGround:
            legsOnGround.append(leg)

    return legsOnGround

SOME_LEG_ID_TRIOS = [
    [0, 1, 3],
    [0, 1, 4],
    [0, 2, 3],
    [0, 2, 4],
    [0, 2, 5],
    [0, 3, 4],
    [0, 3, 5],
    [1, 2, 4],
    [1, 2, 5],
    [1, 3, 4],
    [1, 3, 5],
    [1, 4, 5],
    [2, 3, 5],
    [2, 4, 5],
]

ADJACENT_LEG_ID_TRIOS = [
    [0, 1, 2],
    [1, 2, 3],
    [2, 3, 4],
    [3, 4, 5],
    [0, 4, 5],
    [0, 1, 5],
]

def isStable(p0, p1, p2, tol=0.001):
    cog = Vector(0, 0, 0)

    u = vectorFromTo(p0, p1)
    v = vectorFromTo(p0, p2)
    w = vectorFromTo(p0, cog)
    n = cross(u, v)
    n2 = dot(n, n)

    # cogProjected = alpha * p0 + beta * p1 + gamma * p2
    beta = dot(cross(u, w), n) / n2
    gamma = dot(cross(w, v), n) / n2
    alpha = 1 - beta - gamma

    minVal = -tol
    maxVal = 1 + tol

    cond0 = minVal <= alpha <= maxVal
    cond1 = minVal <= beta <= maxVal
    cond2 = minVal <= gamma <= maxVal

    return cond0 and cond1 and cond2