#!/usr/bin/env python3

import roslaunch.rlutil
import roslaunch.parent
import roslaunch
import rospy
from std_msgs.msg import UInt8
from typing import cast
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path


uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

rospy.init_node("multi_robot_bringup", anonymous=True)

n_agents = cast(UInt8, rospy.wait_for_message("number_of_agents", UInt8)).data

launch_files = []

pkg = "bringup"
src = "coverage.launch"
robot_namespace = rospy.get_param("~robot_namespace", "robot")
print(n_agents)
print(type(n_agents))
for robot_id in range(n_agents):
    print(robot_id)
    path_topic = f"{robot_namespace}_{robot_id}/waypoints"
    start = (
        cast(Path, rospy.wait_for_message(path_topic, Path)).poses[0].pose
    )  # type: Pose
    print(robot_id)
    cli_args = [
        pkg,
        src,
        f"id:={robot_id}",
        f"x_pos:={start.position.x}",
        f"y_pos:={start.position.y}",
        f"yaw:={0.0}",
    ]
    print(robot_id)
    roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
    print(robot_id)
    roslaunch_args = cli_args[2:]
    print(robot_id)
    launch_files.append((roslaunch_file, roslaunch_args))
    print(robot_id)

print("launch filessss")
print(launch_files)

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
parent.start()
parent.spin()
