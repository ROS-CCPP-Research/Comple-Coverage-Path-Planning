#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

def og_callback(event):
    
    occupancy_grid_msg2 = OccupancyGrid()
    og_array = [0] * 100
        
    indices_to_set_to_100 = [10,11,12,20,30,40,50,51,52,53,60,70,80,19,29,39,49,59,69,79,89]
    indices_to_set2_to_100 =[0,1,2,3,4,5,6,7,8,9,90,91,92,93,94,95,96,97,98,99]
        
    for i in indices_to_set_to_100:
        og_array[i] = 100
            
    for i in indices_to_set2_to_100:
        og_array[i] = 100
    
    occupancy_grid_msg2.header = Header()
    occupancy_grid_msg2.header.stamp = rospy.Time.now()
    occupancy_grid_msg2.header.frame_id = 'map'
    occupancy_grid_msg2.info = MapMetaData()
    occupancy_grid_msg2.info.resolution = 1
        
    occupancy_grid_msg2.info.width = 10
    occupancy_grid_msg2.info.height = 10

    occupancy_grid_msg2.info.origin.position.x = -5.0
    occupancy_grid_msg2.info.origin.position.y = -5.0
    occupancy_grid_msg2.info.origin.position.z = 0.0
    occupancy_grid_msg2.info.origin.orientation.x = 0.0
    occupancy_grid_msg2.info.origin.orientation.y = 0.0
    occupancy_grid_msg2.info.origin.orientation.z = 0.0
    occupancy_grid_msg2.info.origin.orientation.w = 0.0
    occupancy_grid_msg2.data = og_array
    
    og_pub.publish(occupancy_grid_msg2)

if __name__ == '__main__':

    try:
        rospy.init_node('square_map')
        rospy.loginfo("Map Node has been started")
        og_pub = rospy.Publisher("/square_map",OccupancyGrid,queue_size=10)
        og_timer = rospy.Timer(rospy.Duration(0.5), og_callback)

        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS node was interrupted. Exiting...")
        rospy.logerr(e)