"""Adapted from https://github.com/richardos/icp"""

import numpy as np
import math
from sklearn.neighbors import NearestNeighbors


class ICP():
    """
    Iterative Closes Point algorithm
    Calculate displacement based on adjacent lidar points cloud
    """

    @staticmethod
    def point_cloud_to_pairs(cloud):
        # move to lidar
        dp = np.array(cloud)
        x = (dp[:, 1] * np.cos(np.radians(dp[:, 0]))).reshape(-1, 1)
        y = (dp[:, 1] * np.sin(np.radians(dp[:, 0]))).reshape(-1, 1)
        return np.hstack((x, y))


    def point_based_matching(self, point_pairs):

        x_mean = 0
        y_mean = 0
        xp_mean = 0
        yp_mean = 0
        n = len(point_pairs)

        if n == 0:
            return None, None, None

        for pair in point_pairs:
            (x, y), (xp, yp) = pair

            x_mean += x
            y_mean += y
            xp_mean += xp
            yp_mean += yp

        x_mean /= n
        y_mean /= n
        xp_mean /= n
        yp_mean /= n

        s_x_xp = 0
        s_y_yp = 0
        s_x_yp = 0
        s_y_xp = 0
        for pair in point_pairs:
            (x, y), (xp, yp) = pair

            s_x_xp += (x - x_mean) * (xp - xp_mean)
            s_y_yp += (y - y_mean) * (yp - yp_mean)
            s_x_yp += (x - x_mean) * (yp - yp_mean)
            s_y_xp += (y - y_mean) * (xp - xp_mean)

        rot_angle = math.atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp)
        translation_x = xp_mean - (x_mean * math.cos(rot_angle) - y_mean * math.sin(rot_angle))
        translation_y = yp_mean - (x_mean * math.sin(rot_angle) + y_mean * math.cos(rot_angle))

        return rot_angle, translation_x, translation_y


    def icp(self,
            reference_points,
            points,
            max_iterations=100,
            distance_threshold=250,
            convergence_translation_threshold=1e-3,
            convergence_rotation_threshold=1e-4,
            point_pairs_threshold=100,
            verbose=False):
        """
        An implementation of the Iterative Closest Point algorithm that matches a set of M 2D points to another set
        of N 2D (reference) points.

        Parameters
        ---------

        reference_points:
            the reference point set as a numpy array (N x 2)

        points:
            the point that should be aligned to the reference_points set as a numpy array (M x 2)

        max_iterations:
            the maximum number of iteration to be executed

        distance_threshold:
            the distance threshold between two points in order to be considered as a pair

        convergence_translation_threshold:
            the threshold for the translation parameters (x and y) for the transformation to be considered converged

        convergence_rotation_threshold:
            the threshold for the rotation angle (in rad) for the transformation to be considered converged

        point_pairs_threshold:
            the minimum number of point pairs that should exist

        verbose:
            whether to print informative messages about the process (default: False)

        Returns
        -------

             resulting_displacement:
                the transformation history as a list of numpy arrays containing the rotation (R) and translation (T)
                 transformation in each iteration in the format [R | T]

            points:
            the aligned points as a numpy array M x 2

        """

        transformation_history = []

        # for x, y, and rot
        resulting_displacement = np.zeros(3)

        nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)

        for iter_num in range(max_iterations):
            if verbose:
                print('------ iteration', iter_num, '------')

            closest_point_pairs = []  # list of point correspondences for closest point rule

            distances, indices = nbrs.kneighbors(points)
            for nn_index in range(len(distances)):
                if distances[nn_index][0] < distance_threshold:
                    closest_point_pairs.append((points[nn_index], reference_points[indices[nn_index][0]]))

            # if only few point pairs, stop process
            if verbose:
                print('number of pairs found:', len(closest_point_pairs))
            if len(closest_point_pairs) < point_pairs_threshold:
                if verbose:
                    print('No better solution can be found (very few point pairs)!')
                break

            # compute translation and rotation using point correspondences
            closest_rot_angle, closest_translation_x, closest_translation_y = self.point_based_matching(closest_point_pairs)
            if closest_rot_angle is not None:
                if verbose:
                    print('Rotation:', math.degrees(closest_rot_angle), 'degrees')
                    print('Translation:', closest_translation_x, closest_translation_y)
            if closest_rot_angle is None or closest_translation_x is None or closest_translation_y is None:
                if verbose:
                    print('No better solution can be found!')
                break

            # transform 'points' (using the calculated rotation and translation)
            c, s = math.cos(closest_rot_angle), math.sin(closest_rot_angle)
            rot = np.array([[c, -s],
                            [s, c]])
            aligned_points = np.dot(points, rot.T)
            aligned_points[:, 0] += closest_translation_x
            aligned_points[:, 1] += closest_translation_y

            resulting_displacement += [closest_translation_x, closest_translation_y, closest_rot_angle]

            # update 'points' for the next iteration
            points = aligned_points

            # update transformation history
            transformation_history.append(np.hstack((rot, np.array([[closest_translation_x], [closest_translation_y]]))))

            # check convergence
            if (abs(closest_rot_angle) < convergence_rotation_threshold) \
                    and (abs(closest_translation_x) < convergence_translation_threshold) \
                    and (abs(closest_translation_y) < convergence_translation_threshold):
                if verbose:
                    print('Converged!')
                break

        return resulting_displacement, points


if __name__ == "__main__":

    """Usage test run"""

    import pickle
    from matplotlib import pyplot as plt
    from glob import glob

    icp = ICP()

    files = glob("test_data/*.pkl")
    with open(files[0], 'rb') as f:
        data = pickle.load(f)

    data_length = len(data)


    print("data_length", data_length)
    print("data_length", data[0])

    plt.figure(figsize=(20, 16))
    if len(data) > 0:

        reference_points = icp.point_cloud_to_pairs(data[0])

        displacement = np.zeros(3)

        for i in range(1, data_length):
            # plt.figure(figsize=(20, 16))
            points_to_be_aligned = icp.point_cloud_to_pairs(data[i])

            transformation_history, aligned_points = icp.icp(reference_points,
                                                             points_to_be_aligned,
                                                             distance_threshold=280,
                                                             verbose=False)

            displacement += transformation_history
            msg = f"i:{i}, dx: {round(displacement[0],0)}, xy: {round(displacement[1])}, rotate: {round(np.degrees(displacement[2]),2)}"

            print(msg)
            points = points_to_be_aligned
            c, s = math.cos(displacement[2]), math.sin(displacement[2])
            rot = np.array([[c, -s],
                            [s, c]])
            points = np.dot(points, rot.T)
            points[:, 0] += displacement[0]
            points[:, 1] += displacement[1]

            plt.scatter(*zip(*points))

            reference_points = points_to_be_aligned
        plt.show()
