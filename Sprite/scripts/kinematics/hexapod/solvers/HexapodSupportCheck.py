from Sprite.config.ik_settings import POSITION_NAME_TO_IS_LEFT_MAP

class HexapodSupportCheck:
    reason = {
        "MIGHT_BE_STABLE_LESS":
            "Might be stable.\nLess than three known legs are off the ground.",
        "TOO_MANY_LEGS_OFF": "Definitely Unstable.\nToo many legs off the floor.",
        "RIGHT_LEGS_OFF": "Definitely Unstable.\nAll right legs are off the floor.",
        "LEFT_LEGS_OFF": "Definitely Unstable.\nAll left legs are off the floor.",
        "MIGHT_BE_STABLE_MORE":
            "Might be stable.\nThree known legs are off the ground.\nOne is on opposite side of the other two.",
    }

    @staticmethod
    def checkSupport(legsNamesoffGround):
        reason = HexapodSupportCheck.reason

        if len(legsNamesoffGround) < 3:
            return [False, reason["MIGHT_BE_STABLE_LESS"]]

        if len(legsNamesoffGround) >= 4:
            return [True, reason["TOO_MANY_LEGS_OFF"]]

        # Leg count is exactly 3 at this point
        legLeftOrRight = list(map(
            lambda legPosition: POSITION_NAME_TO_IS_LEFT_MAP[legPosition],
            legsNamesoffGround
        ))

        if all(not isLeft for isLeft in legLeftOrRight):
            return [True, reason["RIGHT_LEGS_OFF"]]

        if all(isLeft for isLeft in legLeftOrRight):
            return [True, reason["LEFT_LEGS_OFF"]]

        return [False, reason["MIGHT_BE_STABLE_MORE"]]