

class IKMessage:
    success = {
        "subject": "Success.",
        "body": "All legs are on the floor."
    }

    @staticmethod
    def successLegsOnAir(legs):
        return {
            "subject": "Success.",
            "body": f"But some legs won't reach target points on the ground:\n{IKMessage.bulletPoints(legs)}"
        }

    @staticmethod
    def noSupport(reason, legs, flags = {"listLegs": False}):
        return {
            "subject": "Failure: No Support.",
            "body": f"{reason}\n{IKMessage.bulletPoints(legs) if flags['listLegs'] else ''}"
        }

    @staticmethod
    def badPoint(point):
        return {
            "subject": "Failure: Bad Point.",
            "body": f"At least one point would be shoved to the ground:\n{point.toMarkdownString()}"
        }

    @staticmethod
    def bulletPoints(elements):
        return "".join([f" - {position}\n" for position in elements])

    @staticmethod
    def badLeg(message):
        return {
            "subject": "Failure: Bad leg.",
            "body": message
        }

    @staticmethod
    def alphaNotInRange(position, alpha, maxAngle):
        return {
            "subject": "Failure: Alpha not within range",
            "body": f"The alpha ({alpha}) computed for {position} leg is not within -{maxAngle} < alpha < {maxAngle}"
        }

    initialized = {
        "subject": "Initialized",
        "body": "Has not solved for anything yet."
    }


class LegIKInfo:
    @staticmethod
    def targetReached(position):
        return {
            "legPosition": position,
            "message": f"Success! ({position})",
            "obtainedSolution": True,
            "reachedTarget": True
        }

    @staticmethod
    def targetNotReached(position):
        return {
            "legPosition": position,
            "message": f"Success! But this leg won't reach the target ground point. ({position})",
            "obtainedSolution": True,
            "reachedTarget": False
        }

    @staticmethod
    def blocked(position):
        return {
            "legPosition": position,
            "message": f"Failure. The ground is blocking the path. The target point can only be reached it by digging the ground. ({position})",
            "obtainedSolution": False,
            "reachedTarget": True
        }

    @staticmethod
    def femurTooLong(position):
        return {
            "legPosition": position,
            "message": f"Failure. Femur length too long. ({position})",
            "obtainedSolution": False,
            "reachedTarget": False
        }
    @staticmethod
    def tibiaTooLong(position):
        return {
            "legPosition": position,
            "message":  f"Failure. Tibia length too long. ({position})",
            "obtainedSolution": False,
            "reachedTarget": False

        }

    @staticmethod
    def initialized(position):
        return {
            "legPosition": position,
            "obtainedSolution": False,
            "reachedTarget": False,
            "message": f"Haven't solved anything yet. ({position})"
        }
