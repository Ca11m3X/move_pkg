#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import rospy
from move_pkg.msg import EE_Pose


class Ploting():
    def __init__(self):
        self.axes = ['X', 'Y', 'Z']
        self.sub = rospy.Subscriber(
            "/plot_pose", EE_Pose, self.callback_funct)

    def callback_funct(self, msg: EE_Pose):
        data_x = []
        data_y = []
        data_z = []
        for i in range(len(msg.pose_array.poses)):
            data_x.append(msg.pose_array.poses[i].position.x * 1000)
            data_y.append(msg.pose_array.poses[i].position.y * 1000)
            data_z.append(msg.pose_array.poses[i].position.z * 1000)
        self.x = np.array(data_x)
        self.y = np.array(data_y)
        self.z = np.array(data_z)
        self.time = np.array(msg.time)
        self.data = (self.x, self.y, self.z)
        self.create_plot()

    def create_plot(self):
        for i in range(3):
            plt.close()
            plt.title(self.axes[i] + " by Time")
            plt.ylabel("Position (mm)")
            plt.xlabel("Time (sec)")
            # plt.figure(figsize=(8, 6))
            plt.grid(True)
            plt.plot(self.time, self.data[i], marker=".")
            plt.tight_layout()
            plt.savefig(
                f'/home/drx/test_ws/src/move_pkg/scripts/figure/{self.axes[i]}.png')


if __name__ == "__main__":
    rospy.init_node("MatPlot_Pose")
    plot = Ploting()
    rospy.spin()
