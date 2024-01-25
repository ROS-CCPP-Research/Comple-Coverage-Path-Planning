#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
        
def og_callback(event):

    occupancy_grid_msg = OccupancyGrid()
    og_array = [0] * 100

    occupancy_grid_msg.header = Header()
    occupancy_grid_msg.header.stamp = rospy.Time.now()
    occupancy_grid_msg.header.frame_id = 'map_frame'
    occupancy_grid_msg.info = MapMetaData()
    occupancy_grid_msg.info.resolution = 1

    occupancy_grid_msg.info.width = 10
    occupancy_grid_msg.info.height = 10

    occupancy_grid_msg.info.origin.position.x = -5.0
    occupancy_grid_msg.info.origin.position.y = -5.0
    occupancy_grid_msg.info.origin.position.z = 0.0
    occupancy_grid_msg.info.origin.orientation.x = 0.0
    occupancy_grid_msg.info.origin.orientation.y = 0.0
    occupancy_grid_msg.info.origin.orientation.z = 0.0
    occupancy_grid_msg.info.origin.orientation.w = 1.0
    occupancy_grid_msg.data = og_array

    og_pub.publish(occupancy_grid_msg)


if __name__ == '__main__':

    try:
        rospy.init_node('empty_map')
        rospy.loginfo("Map Node has been started")
        og_pub = rospy.Publisher("/empty_map",OccupancyGrid,queue_size=10)
        og_timer = rospy.Timer(rospy.Duration(0.5), og_callback)

        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS node was interrupted. Exiting...")
        rospy.logerr(e)