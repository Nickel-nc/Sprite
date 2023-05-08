import numpy as np
import matplotlib.pyplot as plt
from Sprite.scripts.lidar.n10_cmd import Lidar

"""Matplolib visualizer for real time point cloud estimation"""

# Define a function to update the plot with new models
def update_plot(angles, distances, intensities):
    # Convert polar coordinates to Cartesian coordinates
    x = distances * np.cos(np.radians(angles))
    y = distances * np.sin(np.radians(angles))

    # Clear the previous plot and add the new points
    ax.clear()
    ax.scatter(x, y, c=intensities, cmap='cool', s=10)

    # Set the axis limits based on the models range
    ax.set_xlim([np.min(x), np.max(x)])
    ax.set_ylim([np.min(y), np.max(y)])

    # Redraw the plot
    fig.canvas.draw()
    # plt.savefig("test.png")


if __name__ == "__main__":
    lidar = Lidar()

    # Initialize the plot
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Real-time lidar Point Cloud')
    try:

        while True:
            data = np.array(lidar.get_circle_batch())
            angles = data[:,0]
            distances = data[:,1]
            intensities = data[:, 2]
            update_plot(angles, distances, intensities)
            # Pause for a short interval to simulate real-time updates
            plt.pause(0.05)

    finally:
        lidar.close_port()