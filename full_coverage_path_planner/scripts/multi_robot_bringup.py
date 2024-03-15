#!/usr/bin/env python3

import rospy
import roslaunch
import roslaunch.rlutil
import roslaunch.parent
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from typing import cast, List, Tuple

class RobotTracker:
    def __init__(self, robot_id: int, start: Pose, odom_topic: str):
        self.robot_id = robot_id
        self.start = start
        self.positions = [start.position]  # Initialize with start position
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

    def odom_callback(self, msg: Odometry):
        # Append new position to the list
        self.positions.append(msg.pose.pose.position)

    def log_positions(self):
        # Log start, all intermediate positions, and end position
        rospy.loginfo(f"Robot {self.robot_id} start position: {self.start.position}")
        for idx, position in enumerate(self.positions[1:], start=1):  # Skip the first position as it's the start
            rospy.loginfo(f"Robot {self.robot_id} position {idx}: {position}")
        rospy.loginfo(f"Robot {self.robot_id} end position: {self.positions[-1]}")

def start_robots(n_agents: int, robot_namespace: str, launch_files: List[Tuple[str, List[str]]]):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    parent.start()

    trackers = []
    for robot_id in range(n_agents):
        path_topic = f"{robot_namespace}_{robot_id}/waypoints"
        start_pose = cast(Path, rospy.wait_for_message(path_topic, Path)).poses[0].pose
        odom_topic = f"{robot_namespace}_{robot_id}/odom"
        tracker = RobotTracker(robot_id, start_pose, odom_topic)
        trackers.append(tracker)

    return trackers

def main():
    rospy.init_node("multi_robot_bringup", anonymous=True)
    n_agents = cast(UInt8, rospy.wait_for_message("number_of_agents", UInt8)).data
    robot_namespace = rospy.get_param("~robot_namespace", "robot")

    launch_files = []
    pkg = "bringup"
    src = "coverage.launch"

    for robot_id in range(n_agents):
        path_topic = f"{robot_namespace}_{robot_id}/waypoints"
        start_pose = cast(Path, rospy.wait_for_message(path_topic, Path)).poses[0].pose
        cli_args = [
            pkg,
            src,
            f"id:={robot_id}",
            f"x_pos:={start_pose.position.x}",
            f"y_pos:={start_pose.position.y}",
            f"yaw:={0.0}",
        ]
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        roslaunch_args = cli_args[2:]
        launch_files.append((roslaunch_file, roslaunch_args))

    trackers = start_robots(n_agents, robot_namespace, launch_files)

    def shutdown_hook():
        # Log positions for each robot on shutdown
        for tracker in trackers:
            tracker.log_positions()

    rospy.on_shutdown(shutdown_hook)
    rospy.spin()

if __name__ == "__main__":
    main()
