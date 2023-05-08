#!/usr/bin/env python3
# encoding: utf-8
import os
import sqlite3 as sql

# from BusServoCmd import *
# from Board import setBusServoPulse, stopBusServo

# PC software editor action call library

runningAction = False
stop_action = False
stop_action_group = False

def stopAction():
    global stop_action
    
    stop_action = True

def runActionGroup(actName, times=1):
    global stop_action
    global stop_action_group

    stop_action = False

    temp = times
    while True:
        if temp != 0:
            times -= 1
            if times < 0 or stop_action_group: # Out of the loop
                stop_action_group = False
                break
            runAction(actName)
        elif temp == 0:
            if stop_action_group: # Out of the loop
                stop_action_group = False
                break
            runAction(actName)

def runAction(actNum, lock_servos=''):
    '''
    Running action group, can not send stop signal
    :param actNum: action group name, character string stype
    :param times:  run times
    :return:
    '''
    global runningAction
    global stop_action
    global stop_action_group
    if actNum is None:
        return

    actNum = os.path.join(os.getcwd(), "Sprite1/cur_test/ActionGroups/" + actNum + ".d6a")

    if os.path.exists(actNum):
        if runningAction is False:
            runningAction = True
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")
            while True:
                act = cu.fetchone()
                if stop_action:
                    stop_action_group = True
                    break
                if act is not None:
                    moveServos([i+1 for i in range(N_SERVOS)], list(act[2:]), act[1])
                    """
                    for i in range(0, len(act) - 2, 1):
                        if str(i + 1) in lock_servos:

                            # setBusServoPulse(i + 1, lock_servos[str(i + 1)], act[1])
                            # setBusServoPulse(id, pulse, use_time)
                            moveServos([i+1], [lock_servos[str(i + 1)]], act[1])
                        else:
                            pass
                            moveServos([i + 1], [act[2 + i]], act[1])
                            # setBusServoPulse(i + 1, act[2 + i], act[1])
                    for j in range(int(act[1]/50)):
                        if stop_action:
                            stop_action_group = True
                            break
                        time.sleep(0.05)
                    time.sleep(0.001 + act[1]/1000.0 - 0.05*int(act[1]/50))
                    """
                else:   # run complete exit
                    break
            runningAction = False
            
            cu.close()
            ag.close()
    else:
        runningAction = False



# if __name__ == "__main__":
#     runActionGroup('stand_low', times=1)
