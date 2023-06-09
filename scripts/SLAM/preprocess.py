import numpy as np
import pickle
from glob import glob
from scripts.SLAM.icp import ICP
from scripts.lidar.n10_cmd import Lidar

class LidarDataProcessor():
    def __init__(self, min_angle=240, max_angle=120, angle_resolution=1, icp_distance_thr=280):
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle_range = self.get_angle_range()
        self.icp_distance_thr = icp_distance_thr

        self.bins = np.arange(0, self.angle_range, angle_resolution)

        self.icp = ICP()
        self.lidar = Lidar()

        self.init_scan()


    def init_scan(self):

        raw_data = self.lidar.get_limited_angle_batch()
        data_range = self.convert_data_to_range(raw_data)

        self.reference_points = self.icp.point_cloud_to_pairs(raw_data)

        self.displacement = np.zeros(3)

        self.data = {
        "range": list(data_range),
        "x": 0,
        "y": 0,
        "theta": 0.
        }

    def get_angle_range(self):

        if self.min_angle > self.max_angle:
            angle_range = 360 - self.min_angle + self.max_angle
        else:
            angle_range = self.max_angle - self.min_angle
        return angle_range

    def convert_data_to_range(self, data_points):
        """
        Distribute data points to bins of fixed angles

        # Sanity check for outliers in sensor measurement (rare 0 dist window)
        """

        data_points = np.array(data_points)

        angles = data_points[:,0]
        distances = data_points[:, 1]

        bins = np.arange(0, self.angle_range, 1)

        angles = angles%360
        normalized_angles = np.where(angles>=180, self.min_angle - (angles-self.min_angle), self.max_angle-angles)

        ids = np.digitize(normalized_angles, bins, right=False)-1  # shift bins to match zero-indexed array

        range_data = np.zeros(len(bins))

        for i, dist in enumerate(distances):
            if ids[i] >= 0:
                range_data[ids[i]] = dist

        return range_data.astype(int)


    def scan_frame(self):

        raw_data = self.lidar.get_limited_angle_batch()

        points_to_be_aligned = self.icp.point_cloud_to_pairs(raw_data)
        transformation_history, aligned_points = self.icp.icp(self.reference_points,
                                                              points_to_be_aligned,
                                                              distance_threshold=self.icp_distance_thr,
                                                              verbose=False)

        self.displacement += transformation_history

        data_range = self.convert_data_to_range(raw_data)

        self.data = {
        "range": list(data_range),
        "x": self.displacement[0],
        "y": self.displacement[1],
        "theta": self.displacement[2]
        }

        self.reference_points = points_to_be_aligned

    def close_lidar_port(self):
        self.lidar.close_port()


if __name__ == "__main__":
    """Debug test usage"""

    import time, os

    p = LidarDataProcessor()
    icp = ICP()
    # read raw_data
    files = glob("test_data/raw_data/*.pkl")
    with open(files[0], 'rb') as f:
        data = pickle.load(f)
    data_length = len(data)

    processed_data = {
        'map': {}
    }

    data_range = p.convert_data_to_range(data[0])
    reference_points = icp.point_cloud_to_pairs(data[0])


    processed_data['map'][str(time.time())] = {
        "range": data_range,
        "theta": 0.,
        "x": 0,
        "y": 0
    }

    total_displacement = np.zeros(3)  # [x,y,rotation]
    for i in range(1, data_length):
        points_to_be_aligned = icp.point_cloud_to_pairs(data[i])

        transformation_history, aligned_points = icp.icp(reference_points,
                                                         points_to_be_aligned,
                                                         distance_threshold=280,
                                                         verbose=False)

        total_displacement += transformation_history
        x, y, rot = total_displacement
        data_range = p.convert_data_to_range(data[i])
        processed_data['map'][str(time.time())] = {
            "range": list(data_range),
            "theta": rot,
            "x": x,
            "y": y
        }

        reference_points = points_to_be_aligned

    tree = "test_data/preprocessed_data"
    if not os.path.exists(tree):
        os.makedirs(tree)

    with open(f"{tree}/test.pkl", 'wb') as f:
        pickle.dump(processed_data, f)


