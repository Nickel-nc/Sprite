import math
import os, sys
sys.path.append(os.getcwd())

from scripts.SLAM.preprocess import LidarDataProcessor
from scripts.SLAM.fast_slam import ParticleFilter

from config.slam_settings import (OG_PARAMETERS, SM_PARAMETERS, NUM_PARTICLES)
from scripts.path_finding.rrt_star import RRTStar
from scripts.path_finding.test_move_engine import IKMove


class PathPlanner():
    """
    This driven by following simple pattern:
        initial look around -> scan
        while goal is reachable and destination goal is not reached:
            search path to goal
            if exists, move for some short segment of the path and rescan path checking changes in environment
            else look around and rescan
            Repeat
    """

    def __init__(self):
        self.ldp = LidarDataProcessor()  # create initial lidar scan frame
        self.ik = IKMove()

        self.frame_cnt = 0
        self.pf = ParticleFilter(NUM_PARTICLES, OG_PARAMETERS, SM_PARAMETERS)

        self.x = 0
        self.y = 0
        self.theta = 0

        self.update_map_state()

    def update_map_state(self):
        """Update SLAM with fresh lidar processed scan"""

        self.frame_cnt += 1
        self.pf.updateParticles(self.ldp.data, self.frame_cnt)
        if self.pf.weightUnbalanced():
            self.pf.resample()

        self.state = self.pf.particles[0]

    def search_path(self, goal_global_xy):
        """Perform path finding for current map state"""

        rrt = RRTStar(state=self.state, real_goal=goal_global_xy)
        found_path = rrt.planning()
        if found_path is not None:
            found_path = rrt.map_path_to_real_position(found_path)


        self.x = rrt.real_pos_x
        self.y = rrt.real_pos_y
        self.theta = rrt.map_theta

        return found_path

    def update_localize(self, goal_global_xy, calc_path=True):

        """Perform localization and search path pipeline for one more step"""

        # update lidar scan
        self.ldp.scan_frame()
        # update grid map
        self.update_map_state()
        # update path
        if calc_path:
            return self.search_path(goal_global_xy)

    def locate_around(self):
        """
        Take initial several scans of environment
        Assume it has some free space on the start
        """
        for deg in range(45, 360, 45):
            self.ik.turn_left(deg)
            self.update_localize(goal_global_xy=None, calc_path=False)
        self.ik.turn_left(45)


    def get_dist_and_angle(self, destination):

        """Get robot move instruction to destination point"""

        x_dest, y_dest = destination
        dx = x_dest - self.x
        dy = y_dest - self.y

        distance = math.sqrt(dx ** 2 + dy ** 2)

        angle = math.atan2(dy, dx) - math.radians(self.theta)
        # adjust angle to be in the range [-pi, pi] to ensure closest rotate direction
        if angle > math.pi:
            angle -= 2 * math.pi
        elif angle < -math.pi:
            angle += 2 * math.pi

        return distance, math.degrees(angle)

    def move_to(self, goal_global_xy):
        is_on_path = True
        self.locate_around()

        while is_on_path:  # TO DO: process limits for "unable to reach goal" condition
            path = self.update_localize(goal_global_xy=goal_global_xy, calc_path=True)
            if path is not None:
                if len(path) < 2:
                    is_on_path = False

                distance, angle = self.get_dist_and_angle(path[1])
                if angle < 0:
                    self.ik.turn_right(angle)
                else:
                    self.ik.turn_left(angle)
                self.ik.forward(distance)
            else:
                self.ik.turn_left(30)

    def close_processes(self):
        # close translator
        self.ik.shutdown_servos()
        self.ldp.close_lidar_port()



if __name__ == "__main__":
    planner = PathPlanner()
    try:
        destination = (-2000, -2000)
        planner.move_to(destination)
    finally:
        planner.close_processes()
