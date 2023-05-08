import math

# sizes in millimeters
initMapXLength = 10000
initMapYLength = 10000
initXY = {'x':0.0, 'y':0.0, 'theta': 0.0}
unitGridSize = 20
lidarFOV = math.radians(240)
lidarMaxRange = 10000
numSamplesPerRev = 240

numParticles = 1

scanMatchSearchRadius = 1400
scanMatchSearchHalfRad = 0.25
scanSigmaInNumGrid = 2
wallThickness = 5 * unitGridSize
moveRSigma = 100
maxMoveDeviation = 200
turnSigma = 0.3
missMatchProbAtCoarse = 0.15
coarseFactor = 5

ogParameters = [initMapXLength, initMapYLength, initXY, unitGridSize,
                lidarFOV, lidarMaxRange, numSamplesPerRev, wallThickness]

smParameters = [scanMatchSearchRadius, scanMatchSearchHalfRad, scanSigmaInNumGrid,
                moveRSigma, maxMoveDeviation, turnSigma, missMatchProbAtCoarse, coarseFactor]
