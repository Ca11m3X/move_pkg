#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import rospy
from moveit_msgs.msg import RobotTrajectory


class Ploting():
    def __init__(self):
        self.joints = ['Waist', 'Shoulder', 'Elbow',
                                "Forearm", "Wrist Angle", "Wrist Rotate"]
        self.sub = rospy.Subscriber(
            "/plot_data", RobotTrajectory, self.callback_funct)

    def callback_funct(self, msg: RobotTrajectory):
        joint_values = []
        time = []
        for i in range(len(msg.joint_trajectory.points)):
            joint_values.append(
                msg.joint_trajectory.points[i].positions)
            time.append(
                msg.joint_trajectory.points[i].time_from_start.to_sec())

        self.joint_values = np.array(joint_values)
        self.time = np.array(time)
        self.create_plot()

    def create_plot(self):
        for i in range(len(self.joints)):
            plt.close()
            joint_positions = self.joint_values[:, i]
            plt.title(self.joints[i] + " by Time")
            plt.ylabel("Position (rad)")
            plt.xlabel("Time (sec)")
            # plt.figure(figsize=(8, 6))
            plt.grid(True)
            plt.plot(self.time, joint_positions, marker=".")
            plt.tight_layout()
            plt.savefig(
                f'/home/drx/test_ws/src/move_pkg/scripts/figure/{self.joints[i]}.png')


if __name__ == "__main__":
    rospy.init_node("MatPlot")
    plot = Ploting()
    rospy.spin()
