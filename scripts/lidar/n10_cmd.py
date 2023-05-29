import serial


class Lidar():
    """
    Command wrapper for N10 Leishen lidar datagen
    """

    def __init__(self):
        self.init_port()

    def init_port(self):
        # self.ser = serial.Serial("COM3", 230400, timeout=5)  # for desktop debug
        self.ser = serial.Serial("/dev/ttyUSB0", 230400, timeout=5)


    def close_port(self):
        self.ser.close()

    def init_state(self):
        self.listdata = []
        self.lastangle = 0
        self.cur_angle = 0
        self.rpm = []

    def reset(self):
        self.close_port()
        self.init_port()

    def update_buffered_angle(self, start_angle, end_angle):
        """
        Update angle by new angle difference.
        Handle circular difference
        """
        angle_diff = abs(end_angle - start_angle)

        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        self.angle_diff = angle_diff
        self.buffered_angle += self.angle_diff


    def get_circle_batch(self):
        """
        Gather data for one revolute degrees cycle

        clearing buffer like self.ser.flushOutput() Won't work if device DTR Line disabled
        """
        self.reset()
        self.init_state()

        self.buffered_angle = 0

        while True:
            data = self.ser.read(3)  # Read 3 bytes of models

            if data[0] == 0xA5 and data[1] == 0x5A and data[2] == 0x3A:  # Determine whether it is a models frame header
                data = self.ser.read(55)  # If it is a models frame header, read the whole frame, and after removing the frame header, it will be 55 bytes

                self.rpm.append((data[0] * 256 + data[1]) * 144 / 1000)

                start_angle = (data[2] * 256 + data[3]) / 100.0
                end_angle = (data[52] * 256 + data[53]) / 100.0
                self.update_buffered_angle(start_angle, end_angle)

                if self.buffered_angle >= 360:
                    return self.listdata

                # 16 pts

                angle_increment = self.angle_diff / 15
                cnt = 0
                for x in range(4, 50, 3): # 16 points
                    # angle = start_angle+(angle_increment*cnt)
                    # dist = data[x] * 256 + data[x + 1]
                    # signal_str = data[x + 2]

                    self.listdata.append([start_angle+angle_increment*cnt, data[x] * 256 + data[x + 1], data[x + 2]])
                    # angle
                    # dist
                    # signal strenght
                    cnt+=1


    def get_limited_angle_batch(self, angle_min=240, angle_max=120):
        """
        Gather data for one revolute degrees cycle
        Collect angles within constrained values (filter robot blind zone)
        """
        self.reset()
        self.init_state()

        self.buffered_angle = 0

        while True:
            data = self.ser.read(3)  # Read 3 bytes of models

            if data[0] == 0xA5 and data[1] == 0x5A and data[2] == 0x3A:  # Determine whether it is a models frame header
                data = self.ser.read(55)  # If it is a models frame header, read the whole frame, and after removing the frame header, it will be 55 bytes

                self.rpm.append((data[0] * 256 + data[1]) * 144 / 1000)

                start_angle = (data[2] * 256 + data[3]) / 100.0
                end_angle = (data[52] * 256 + data[53]) / 100.0
                self.update_buffered_angle(start_angle, end_angle)

                if self.buffered_angle >= 360:
                    return self.listdata

                # 16 pts

                angle_increment = self.angle_diff / 15
                cnt = 0
                for x in range(4, 50, 3): # 16 points
                    angle = start_angle+(angle_increment*cnt)
                    distance = data[x] * 256 + data[x + 1]
                    signal_strength = data[x + 2]
                    if angle_min > angle_max:
                        if angle <= angle_max \
                        or angle >= angle_min:
                            self.listdata.append([angle, distance, signal_strength])
                    else:
                        if angle_max >= angle >= angle_min:
                            self.listdata.append([angle, distance, signal_strength])
                    cnt+=1
