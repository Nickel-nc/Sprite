import serial


class N10():
    """
    Command wrapper for N10 Leishen Lidar datagen
    """

    def __init__(self):
        self.init_port()

    def init_port(self):
        # self.ser = serial.Serial("COM3", 230400, timeout=5)
        self.ser = serial.Serial("/dev/ttyUSB0", 230400, timeout=5)


    def close_port(self):
        self.ser.close()

    def init_state(self):
        self.listdata = []
        self.lastangle = 0
        self.cur_angle = 0
        self.rpm = []


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
        Gather models for one 360 degrees cycle
        """
        self.init_state()

        self.buffered_angle = 0

        while True:

            data = self.ser.read(3)  # Read 3 bytes of models

            if data[0] == 0xA5 and data[1] == 0x5A and data[2] == 0x3A:  # Determine whether it is a models frame header
                data = self.ser.read(55)  # If it is a models frame header, read the whole frame, and after removing the frame header, it will be 55 bytes

                self.rpm.append((data[0] * 256 + data[1]) * 144 / 1000)

                start_angle = (data[2] * 256 + data[3]) / 100.0
                end_angle = (data[52] * 256 + data[53]) / 100.0
                # print("angles", start_angle, end_angle)
                self.update_buffered_angle(start_angle, end_angle)

                if self.buffered_angle >= 360:
                    return self.listdata

                # 16 pts

                angle_increment = self.angle_diff / 15
                print("end_angle - start_angle",end_angle, start_angle, end_angle - start_angle, angle_increment)
                cnt = 0
                for x in range(4, 50, 3): # 16 points
                    angle = start_angle+(angle_increment*cnt)
                    dist = data[x] * 256 + data[x + 1]
                    signal_str = data[x + 2]
                    # print("a, d, ss", angle, dist, signal_str)

                    self.listdata.append([start_angle+angle_increment*cnt, data[x] * 256 + data[x + 1], data[x + 2]])
                    # angle
                    # dist
                    # signal strenght
                    cnt+=1




if __name__ == '__main__':
    lidar = N10()
    data = lidar.get_circle_batch()
    print(data)

      


      





  
  




  
