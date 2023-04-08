#!/usr/bin/env python
from time import sleep
import serial
from multiprocessing import Process, Queue

from config.cmd_settings import *



class BusCmd():
    def __init__(self):
        self.rQueue = Queue()
        self.wQueue = Queue()

        # self.serialData = self.openSerialPort()
        self.startSerialProcess()


    def openSerialPort(self):
        """Call to begin listining to the serial port. Uses openSerialPort"""
        serialData = serial.Serial(
            port='/dev/ttyS0',
            baudrate=9600,
            bytesize=serial.EIGHTBITS,
            timeout=None
        )

        try:
            while True:
                if (serialData.inWaiting()):
                    byte = serialData.read(1)
                    self.readQueue.put(byte)
                if (not self.writeQueue.empty()):
                    byte = self.writeQueue.get()
                    serialData.write(byte)
        finally:
            serialData.close()

    def startSerialProcess(self):
        p = Process(target=self.openSerialPort, args=(self.wQueue, self.rQueue))
        p.daemon = True
        p.start()

    @staticmethod
    def anglesToPositions(angles):
        """Converts -120/120 to 0-1000 (servo pulse)"""
        positions = []
        for angle in angles:
            positions.append(int(round((angle + 120) / 240 * 1000)))
        return positions

    @staticmethod
    def positionsToAngles(positions):
        """converts 0-1000 (servo position unit as integer) to -120/120"""
        angles = []
        for pos in positions:
            angles.append((pos / 1000 * 240) - 120)
        return angles

    def moveServos(self, servoIDs, positions, time):
        numOfServos = len(servoIDs)
        # check if each servo has a corresponding time and angle
        if (len(positions) != numOfServos):
            print("WARNING. pos|time lenghts mismatch")
            return

        packetLength = 5 + 3 * numOfServos
        # create packet header
        cmdbytes = bytearray(4)
        cmdbytes[0] = FRAME_HEADER  # header 1
        cmdbytes[1] = FRAME_HEADER  # header 2
        cmdbytes[2] = packetLength  # packet length
        cmdbytes[3] = CMD_SERVO_MOVE  # command number
        cmdbytes.append(numOfServos)  # number of servos that will move
        timeLSB = time & 0xFF  # least significant bit
        timeMSB = (time >> 8) & 0xFF  # most significant bit
        cmdbytes.append(timeLSB)
        cmdbytes.append(timeMSB)

        for i in range(numOfServos):
            # split angle into 2 bytes
            posLSB = positions[i] & 0xFF
            posMSB = (positions[i] >> 8) & 0xFF
            # per servo parameters
            cmdbytes.append(servoIDs[i])
            cmdbytes.append(posLSB)
            cmdbytes.append(posMSB)

        # bytesAsHex = ' '.join(format(x, '02x') for x in cmdbytes)
        # print(bytesAsHex)
        self.wQueue.put(cmdbytes)
        sleep(time / 1000)

    def powerOffServos(self):
        """
        Unload all servos
        i.e. id servos from 1 to 23 include six legs and end-effector gripper
        """
        sleep(0.5)
        buf = bytearray(b'\x55\x55')
        buf.append(N_SERVOS+3)
        buf.append(CMD_MULT_SERVO_UNLOAD)
        buf.append(N_SERVOS)
        for i in range(1, N_SERVOS+1):
            buf.append(i)
        self.wQueue.put(buf)
        sleep(0.5)
        # serialData.write(buf)

    def runActionGroup(self, groupNumber, iterations=1):
        # create packet header
        cmdbytes = bytearray(7)
        cmdbytes[0] = FRAME_HEADER  # header 1
        cmdbytes[1] = FRAME_HEADER  # header 2
        cmdbytes[2] = 0x05  # packet length
        cmdbytes[3] = CMD_ACTION_GROUP_RUN  # command number
        cmdbytes[4] = groupNumber

        itrLSB = iterations & 0xFF  # least significant bit
        itrMSB = (iterations >> 8) & 0xFF  # most significant bit
        cmdbytes[5] = itrLSB
        cmdbytes[6] = itrMSB

        # bytesAsHex = ' '.join(format(x, '02x') for x in cmdbytes)
        # print(bytesAsHex)
        self.wQueue.put(cmdbytes)


    # Cancel running action group
    def stopActionGroup(self):
        # create packet header
        cmdbytes = bytearray(4)
        cmdbytes[0] = FRAME_HEADER  # header 1
        cmdbytes[1] = FRAME_HEADER  # header 2
        cmdbytes[2] = 0x02  # packet length
        cmdbytes[3] = CMD_ACTION_GROUP_STOP  # command number
        # bytesAsHex = ' '.join(format(x, '02x') for x in cmdbytes)
        # print(bytesAsHex)
        self.wQueue.put(cmdbytes)


    # get current servo positions
    def readPositions(self):
        # create packet header
        cmdbytes = bytearray(11)
        cmdbytes[0] = FRAME_HEADER  # header 1
        cmdbytes[1] = FRAME_HEADER  # header 2
        cmdbytes[2] = 0x09  # packet length
        cmdbytes[3] = CMD_GET_POSITIONS  # command number
        cmdbytes[4] = 0x06
        cmdbytes[5] = 0x01
        cmdbytes[6] = 0x02
        cmdbytes[7] = 0x03
        cmdbytes[8] = 0x04
        cmdbytes[9] = 0x05
        cmdbytes[10] = 0x06
        # bytesAsHex = ' '.join(format(x, '02x') for x in cmdbytes)
        # print(bytesAsHex)
        self.wQueue.put(cmdbytes)



#######
# Usage Test
#######

if __name__ == '__main__':
    cmd = BusCmd()
    
    cmd.moveServos([1,4,7,10,13,16], [400]*6, 500)
    cmd.moveServos([1,4,7,10,13,16], [500]*6, 500)

    cmd.powerOffServos()





