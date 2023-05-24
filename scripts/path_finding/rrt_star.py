import random
import math
import copy

import numpy as np
from config.slam_settings import unitGridSize
from config.ik_settings import MOVING_FRONT_SAFE_DISTANCE, MOVING_REAR_SAFE_DISTANCE
from matplotlib import pyplot as plt


class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None
        self.leaf = True

class RRTStar():

    """Path finding RRT* og-based routine"""

    def __init__(self,
                 state,
                 real_goal,
                 expand_dist=10,
                 goal_sample_rate=15,
                 max_iter=1000,
                 near_search_rcoef=4):
        """
        Parameters
        ----------

            state: ParticleFilter class object
                Contains SLAM calculated parameters such as occupation grid map, trajectory...
                ParticleFilter class object

            real_goal: list
                Goal point in global coordinates in millimeters
                Position [x,y]

            expand_dist: int
                Distance increment in map pixels for Boundiung box collision algorithm.
                Should be in range of wall thickness(OGrig parameter)

            goal_sample_rate: int
                Rate to perform sampling from goal point

            max_iter: int
                Number of sampling iterations

            near_search_rcoef: int
                coefficient of searching radius for nearest nodes.
                Result search radius is: expandDis * near_search_rcoef

        """

        # Robot body parameters
        # assumes robot bbox as rectangle
        self.robot_half_length = MOVING_FRONT_SAFE_DISTANCE//unitGridSize
        self.robot_half_width = MOVING_REAR_SAFE_DISTANCE//unitGridSize

        self.obstacle_map = np.flipud(state.og.occupancyGridVisited / state.og.occupancyGridTotal)
        self.obstacle_map = np.where(self.obstacle_map > 0.5, 1, 0)

        self.x_limit = (self.obstacle_map.shape[1] - self.robot_half_width-20)
        self.y_limit = (self.obstacle_map.shape[0] - self.robot_half_width-20)

        self.map_xrange = state.og.mapXLim
        self.map_yrange = state.og.mapYLim
        self.unit_grid_size = unitGridSize

        self.real_pos_x = state.xTrajectory[-1]
        self.real_pos_y = state.yTrajectory[-1]

        self.map_pos_x, self.map_pos_y = self.real_to_map_position(self.real_pos_x, -self.real_pos_y)
        self.map_theta = state.thetaTrajectory[-1]

        self.map_goal_x, self.map_goal_y = self.real_to_map_position(real_goal[0], -real_goal[1])
        self.start = Node(self.map_pos_x, self.map_pos_y)
        self.end = Node(self.map_goal_x, self.map_goal_y)

        self.expand_dist = expand_dist
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.near_search_rcoef = near_search_rcoef
        self.get_bbox_body_contour()

    def get_bbox_body_contour(self):
        """
        Calculate robot body contour points as rectangular bbox bias around center point
        Rotation applied in future steps
        """

        corners = np.array([
            [-self.robot_half_length, -self.robot_half_width],
            [self.robot_half_length, -self.robot_half_width],
            [self.robot_half_length, self.robot_half_width],
            [-self.robot_half_length, self.robot_half_width]
        ])

        x1, y1, x2, y2, x3, y3, x4, y4 = corners.flatten()
        # Calculate integer points on the edges of the rectangle
        x_top = np.arange(x1, x2 + 1)
        y_top = np.full_like(x_top, y1)
        x_right = np.full(y3 - y2 + 1, x2)
        y_right = np.arange(y2, y3 + 1)
        x_bottom = np.arange(x3, x4 - 1, -1)
        y_bottom = np.full_like(x_bottom, y3)
        x_left = np.full(y4 - y1 + 1, x4)
        y_left = np.arange(y4, y1 - 1, -1)

        x = np.concatenate((x_top, x_right, x_bottom, x_left))
        y = np.concatenate((y_top, y_right, y_bottom, y_left))

        # Remove duplicate points
        points = np.column_stack((x, y))
        self.bbox_contour = np.unique(points, axis=0)

    def get_bbox_anchor_rotation_bias(self, estimated_theta):
        """
        Get bbox corners as offset from zero center point in map coordinates
        Further this precalc used to calculate bbox points around moving center point
        """

        rotation_matrix = np.array([[np.cos(estimated_theta), -np.sin(estimated_theta)],
                                    [np.sin(estimated_theta), np.cos(estimated_theta)]])

        rotated_bbox_bias = np.dot(self.bbox_contour, rotation_matrix).astype(int)

        return rotated_bbox_bias

    def get_bbox_anchor_points(self, estimated_x, estimated_y, bbox_bias):
        """
        Calculate anchor points of bbox for center point in map space base on raw biases
        see get_bbox_anchor_rotation_bias()

        input and return format: [(x1,y1),..., (xn, yn)]
        """

        bbox_anchors = bbox_bias + [estimated_x, estimated_y]
        return bbox_anchors



    def real_to_map_position(self, x,y):
        map_x = int((x - self.map_xrange[0]) / self.unit_grid_size)
        map_y = int((y + self.map_yrange[1]) / self.unit_grid_size)  # y is inverted
        return (map_x, map_y)

    def map_path_to_real_position(self, path):
        path[:, 0] = (path[:, 0] * self.unit_grid_size) + self.map_xrange[0]
        path[:, 1] = (path[:, 1] * self.unit_grid_size) - self.map_yrange[1]
        return path


    def planning(self):
        """Main path planning RRT function"""
        self.node_list = {0: self.start}

        for i in range(self.max_iter):

            rnd = self.get_random_point()
            nind = self.get_nearest_list_index(self.node_list, rnd)  # get nearest node index to random point

            newNode, theta = self.steer(rnd, nind)  # generate new node from that nearest node in direction of random point

            bbox_theta_bias = self.get_bbox_anchor_rotation_bias(theta)

            if self.is_not_collide(newNode, bbox_theta_bias):  # if it does not collide
                nearinds = self.find_near_nodes(newNode, self.near_search_rcoef)  # find nearest nodes to newNode in search range
                newNode = self.choose_parent(newNode,
                                             nearinds)  # from that nearest nodes find the best parent to newNode
                self.node_list[newNode.parent].leaf = False
                self.node_list[i + 100] = newNode  # add newNode to nodeList
                self.rewire(i + 100, newNode, nearinds)  # make newNode a parent of another node if necessary

                if i > self.max_iter:
                    leaves = [key for key, node in self.node_list.items() if node.leaf == True]
                    ind = leaves[random.randint(0, len(leaves) - 1)]

                    self.node_list[self.node_list[ind].parent].leaf = True
                    for value in self.node_list.values():
                        if value.parent == self.node_list[ind].parent and value != self.node_list[ind]:
                            self.node_list[self.node_list[ind].parent].leaf = False
                            break
                    self.node_list.pop(ind)
        # generate course
        lastIndex = self.get_best_last_index()
        if lastIndex is None:
            return None
        path = self.gen_final_course(lastIndex)
        return path

    def choose_parent(self, newNode, nearinds):
        if len(nearinds) == 0:
            return newNode

        dlist = []
        for i in nearinds:
            dx = newNode.x - self.node_list[i].x
            dy = newNode.y - self.node_list[i].y
            d = math.sqrt(dx ** 2 + dy ** 2)
            theta = math.atan2(dy, dx)
            if self.check_collision_extend(self.node_list[i], theta, d):
                dlist.append(self.node_list[i].cost + d)
            else:
                dlist.append(float("inf"))

        mincost = min(dlist)
        minind = nearinds[dlist.index(mincost)]

        if mincost == float("inf"):
            return newNode

        newNode.cost = mincost
        newNode.parent = minind

        return newNode

    def steer(self, rnd, nind):
        """Generate node for one step in direction of random point"""

        # expand tree
        nearestNode = self.node_list[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)
        newNode = copy.deepcopy(nearestNode)
        newNode.x += int(round(self.expand_dist * math.cos(theta)))
        newNode.y += int(round(self.expand_dist * math.sin(theta)))

        newNode.cost += self.expand_dist
        newNode.parent = nind
        newNode.leaf = True
        return newNode, theta

    def get_random_point(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [random.uniform(0, self.x_limit), random.uniform(0, self.y_limit)]
        else:  # goal point sampling
            rnd = [self.end.x, self.end.y]
        return rnd

    def get_best_last_index(self):

        disglist = [(key, self.calc_dist_to_goal(node.x, node.y)) for key, node in self.node_list.items()]
        goalinds = [key for key, distance in disglist if distance <= self.expand_dist]

        if len(goalinds) == 0:
            return None

        mincost = min([self.node_list[key].cost for key in goalinds])
        for i in goalinds:
            if self.node_list[i].cost == mincost:
                return i

        return None

    def gen_final_course(self, goalind):
        path = [[self.end.x, self.end.y]]
        while self.node_list[goalind].parent is not None:
            node = self.node_list[goalind]
            path.append([node.x, node.y])
            goalind = node.parent
        path.append([self.start.x, self.start.y])

        return np.array(path)[::-1]

    def calc_dist_to_goal(self, x, y):
        return np.linalg.norm([x - self.end.x, y - self.end.y])

    def find_near_nodes(self, newNode, value):
        r = self.expand_dist * value
        dlist = [(key, (node.x - newNode.x) ** 2 + (node.y - newNode.y) ** 2) for key, node in self.node_list.items()]
        nearinds = [key for key, distance in dlist if distance <= r ** 2]
        return nearinds

    def rewire(self, newNodeInd, newNode, nearinds):
        # nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.node_list[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
                theta = math.atan2(dy, dx)
                if self.check_collision_extend(nearNode, theta, d):
                    self.node_list[nearNode.parent].leaf = True
                    for value in self.node_list.values():
                        if value.parent == nearNode.parent and value != nearNode:
                            self.node_list[nearNode.parent].leaf = False
                            break

                    nearNode.parent = newNodeInd
                    nearNode.cost = scost
                    newNode.leaf = False

    def check_collision_extend(self, nearNode, theta, d):

        tmpNode = copy.deepcopy(nearNode)
        bbox_theta_bias = self.get_bbox_anchor_rotation_bias(theta)

        for i in range(int(d / self.expand_dist)):
            tmpNode.x += int(round(self.expand_dist * math.cos(theta)))
            tmpNode.y += int(round(self.expand_dist * math.sin(theta)))
            if not self.is_not_collide(tmpNode, bbox_theta_bias):
                return False
        return True

    def get_nearest_list_index(self, nodeList, rnd):
        dlist = [(key, (node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2) for key, node in nodeList.items()]
        minind = min(dlist, key=lambda d: d[1])
        return minind[0]

    def is_not_collide(self, node, bbox_bias):
        """
        Check collision if any of anchor point in obstacle
        """

        bbox_contour = self.get_bbox_anchor_points(node.x, node.y, bbox_bias)  # get bbox rectangle contour points
        for x, y in bbox_contour:
            if self.obstacle_map[y][x] == 1:
                return False  # collision case
        return True  # safe case

    def plot_state(self, bbox_contour=None):
        """For debugging use"""
        plt.figure(figsize=(20, 20))
        obstacle_map = self.obstacle_map
        obstacle_map = 1 - obstacle_map
        im = plt.imshow(obstacle_map, cmap='gray')
        plt.scatter(self.map_pos_x, self.map_pos_y, c='r', marker='.')
        plt.scatter(self.map_goal_x, self.map_goal_y, c='b', marker='.')

        if bbox_contour is not None:
            for pt in bbox_contour:
                plt.scatter(pt[0], pt[1], c='red', marker=',')

        for node in self.node_list.values():
            if node.parent is not None:
                plt.plot([self.node_list[node.parent].x, node.x], [self.node_list[node.parent].y, node.y], c='orange')

        # plt.xlim((450, 850))
        # plt.ylim((450, 850))
        msg = f"Real pos: {round(self.real_pos_x)}, {round(self.real_pos_y)}, theta {round(self.map_theta)}"
        plt.colorbar(im)
        plt.grid()
        plt.title(msg)
        plt.show()
