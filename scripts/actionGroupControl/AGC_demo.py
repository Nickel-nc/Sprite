import time
import ActionGroupControl as AGC

"""
# the action group needs to be saved in the /scripts/actionGroupControl/ActionGroups
# AGC.runActionGroup('stand_low')  # Parameter is action group, without a suffix and passed in as a character
# AGC.runActionGroup('go_forward_low', times=2)  # The second parameter running actions times, the default is 1. When 0, it indicates cyclic running. The third parameter indicates whether to stop at attention at the end
"""


if __name__ == "__main__":
    startSerialProcess()
    time.sleep(0.5)
    AGC.runActionGroup('stand_low', times=1)
    AGC.runActionGroup('stand_middle', times=1)
    AGC.runActionGroup('go_forward_middle', times=1)
    time.sleep(0.5)
    AGC.runActionGroup('turn_right_middle', times=8)
    AGC.runActionGroup('turn_left_middle', times=3)
    time.sleep(0.5)
    AGC.runActionGroup('go_forward_middle', times=4)
    time.sleep(0.5)
    AGC.runActionGroup('back_middle', times=2)
    time.sleep(0.5)
    AGC.runActionGroup('stand_low', times=1)
    time.sleep(0.5)
    powerOffServos()
    time.sleep(0.5)
