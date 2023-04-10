from scripts.kinematics.hexapod.solvers.hexapodSolver import solveHexapodParams

def getWalkSequence(dimensions,
                    params = {'tx': 0, 'tz': 0, 'rx': 0, 'ry': 0, 'legStance': 0, 'hipStance': 25, 'stepCount': 5, 'hipSwing': 25, 'liftSwing': 40},
                    gaitType = "tripod",
                    walkMode = "walking"):

    hipStance, rx, ry, tx, tz, legStance = params['hipStance'], params['rx'], params['ry'], params['tx'], params['tz'], params['legStance']
    rawIKparams = {'tx': tx, 'ty': 0, 'tz': tz, 'legStance': legStance, 'hipStance': hipStance, 'rx': rx, 'ry': ry, 'rz': 0}

    # solver
    ikSolver, _, r_len = solveHexapodParams(dimensions, rawIKparams, True)

    if not ikSolver.foundSolution or ikSolver.hasLegsOffGround:
        return None

    hipSwing, liftSwing, stepCount = params['hipSwing'], params['liftSwing'], params['stepCount']
    aHipSwing, aLiftSwing = abs(hipSwing), abs(liftSwing)

    hipSwings = getHipSwingRotate(aHipSwing) if walkMode == "rotating" else getHipSwingForward(aHipSwing)

    if gaitType == "ripple":
        return rippleSequence(ikSolver.pose, aLiftSwing, hipSwings, stepCount), r_len
    else:
        return tripodSequence(ikSolver.pose, aLiftSwing, hipSwings, stepCount, walkMode), r_len


def tripodSequence(pose, aLiftSwing, hipSwings, stepCount, walkMode = None):
    forwardAlphaSeqs, liftBetaSeqs, liftGammaSeqs = buildTripodSequences(pose, aLiftSwing, hipSwings, stepCount, walkMode).values()
    doubleStepCount = 2 * stepCount

    tripodA = tripodASequence(forwardAlphaSeqs, liftGammaSeqs, liftBetaSeqs, doubleStepCount)
    tripodB = tripodBSequence(forwardAlphaSeqs, liftGammaSeqs, liftBetaSeqs, doubleStepCount)

    return {**tripodA, **tripodB}


def tripodASequence(forwardAlphaSeqs, liftGammaSeqs, liftBetaSeqs, doubleStepCount):
    sequences = {}
    for legPosition in ["leftFront", "rightMiddle", "leftBack"]:
        forward = forwardAlphaSeqs[legPosition]
        gammaLiftUp = liftGammaSeqs[legPosition]
        betaLiftUp = liftBetaSeqs[legPosition]

        gammaSeq = gammaLiftUp + gammaLiftUp[::-1] + fill_array(gammaLiftUp[0], doubleStepCount)
        betaSeq = betaLiftUp + betaLiftUp[::-1] + fill_array(betaLiftUp[0], doubleStepCount)

        sequences[legPosition] = {'alpha': forward + forward[::-1], 'gamma': gammaSeq, 'beta': betaSeq}

    return sequences


def tripodBSequence(forwardAlphaSeqs, liftGammaSeqs, liftBetaSeqs, doubleStepCount):
    sequences = {}
    for legPosition in ["rightFront", "leftMiddle", "rightBack"]:
        forward = forwardAlphaSeqs[legPosition]
        gammaLiftUp = liftGammaSeqs[legPosition]
        betaLiftUp = liftBetaSeqs[legPosition]
        gammaSeq = fill_array(gammaLiftUp[0], doubleStepCount) + gammaLiftUp + gammaLiftUp[::-1]
        betaSeq = fill_array(betaLiftUp[0], doubleStepCount) + betaLiftUp + betaLiftUp[::-1]
        sequences[legPosition] = {"alpha": forward[::-1] + forward, "gamma": gammaSeq, "beta": betaSeq}
    return sequences


def buildTripodSequences (start_pose, a_lift_swing, hip_swings, step_count, walk_mode):
    double_step_count = 2 * step_count
    leg_positions = list(start_pose.keys())

    forward_alpha_seqs = {}
    lift_beta_seqs = {}
    lift_gamma_seqs = {}


    for leg_position in leg_positions:
        alpha, beta, gamma = start_pose[leg_position].values()
        delta_alpha = hip_swings[leg_position]
        forward_alpha_seqs[leg_position] = buildSequence(
            alpha - delta_alpha,
            2 * delta_alpha,
            double_step_count
        )
        lift_beta_seqs[leg_position] = buildSequence(beta, a_lift_swing, step_count)
        lift_gamma_seqs[leg_position] = buildSequence(gamma, -a_lift_swing / 2, step_count)

    return {
        'forward_alpha_seqs': forward_alpha_seqs,
        'lift_beta_seqs': lift_beta_seqs,
        'lift_gamma_seqs': lift_gamma_seqs
    }

"""
RIPPLE SEQUENCE
a - lift-up
b - shove-down
[1, 2, 3, 4] - retract / power stroke sequence
left-back     |-- a --|-- b --|   1   |   2   |   3   |   4   |
left-middle   |   3   |   4   |-- a --|-- b --|   1   |   2   |
left-front    |   1   |   2   |   3   |   4   |-- a --|-- b --|
right-front   |   4   |-- a --|-- b --|   1   |   2   |   3   |
right-back    |   1   |   2   |   3   |-- a --|-- b --|   4   |
right-middle  |-- b --|   1   |   2   |   3   |   4   |-- a --|
"""

def rippleSequence(startPose, aLiftSwing, hipSwings, stepCount):
    legPositions = startPose.keys()

    sequences = {}
    for position in legPositions:
        alpha, beta, gamma = startPose[position].values()
        betaLift = buildSequence(beta, aLiftSwing, stepCount)
        gammaLift = buildSequence(gamma, -aLiftSwing / 2, stepCount)

        delta = hipSwings[position]
        fw1 = buildSequence(alpha - delta, delta, stepCount)
        fw2 = buildSequence(alpha, delta, stepCount)

        halfDelta = delta / 2
        bk1 = buildSequence(alpha + delta, -halfDelta, stepCount)
        bk2 = buildSequence(alpha + halfDelta, -halfDelta, stepCount)
        bk3 = buildSequence(alpha, -halfDelta, stepCount)
        bk4 = buildSequence(alpha - halfDelta, -halfDelta, stepCount)

        sequences[position] = buildRippleLegSequence(
            position, betaLift, gammaLift, fw1, fw2, bk1, bk2, bk3, bk4
        )

    return sequences

def buildRippleLegSequence(position, bLift, gLift, fw1, fw2, bk1, bk2, bk3, bk4):
    stepCount = len(fw1)
    revGLift = gLift[::-1]
    revBLift = bLift[::-1]
    b0 = bLift[0]
    g0 = gLift[0]
    bN = fill_array(b0, stepCount)
    gN = fill_array(g0, stepCount)

    alphaSeq = [fw1, fw2, bk1, bk2, bk3, bk4]
    betaSeq = [bLift, revBLift, bN, bN, bN, bN]
    gammaSeq = [gLift, revGLift, gN, gN, gN, gN]

    moduloMap = {
        "leftBack": 0,
        "rightFront": 1,
        "leftMiddle": 2,
        "rightBack": 3,
        "leftFront": 4,
        "rightMiddle": 5,
    }

    return {
        "alpha": modSequence(moduloMap[position], alphaSeq),
        "beta": modSequence(moduloMap[position], betaSeq),
        "gamma": modSequence(moduloMap[position], gammaSeq),
    }

def modSequence(mod, seq):
    sequence = seq + seq
    return sequence[mod : mod + 6]



def buildSequence(start_val, delta, step_count):
    step = delta / step_count
    current_item = start_val
    array = []
    for i in range(step_count):
        current_item += step
        array.append(current_item)
    return array

def getHipSwingForward(a_hip_swing):
    return {
        'leftFront': -a_hip_swing,
        'rightMiddle': a_hip_swing,
        'leftBack': -a_hip_swing,
        'rightFront': a_hip_swing,
        'leftMiddle': -a_hip_swing,
        'rightBack': a_hip_swing
    }


def getHipSwingRotate(a_hip_swing):
    return {
        'leftFront': a_hip_swing,
        'rightMiddle': a_hip_swing,
        'leftBack': a_hip_swing,
        'rightFront': a_hip_swing,
        'leftMiddle': a_hip_swing,
        'rightBack': a_hip_swing
    }


def fill_array(value, length):
    if length == 0:
        return []
    a = [value]
    while len(a) * 2 <= length:
        a = a + a
    if len(a) < length:
        a = a + a[:length - len(a)]
    return a
