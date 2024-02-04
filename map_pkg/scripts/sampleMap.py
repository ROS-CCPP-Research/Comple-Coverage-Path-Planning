#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
        
def save_occupancy_grid_as_png(occupancy_grid_data, file_path):
    
    binary_image = np.where(occupancy_grid_data == 100, 0, 255).astype(np.uint8)

    plt.imsave(file_path, binary_image, cmap='gray', vmin=0, vmax=255)
    
    
def og_callback(event):

    occupancy_grid_msg = OccupancyGrid()
    og_array = [0] * 100
    
    indices_to_set_to_100 = [53, 63, 73, 83, 93]
    indices_to_set2_to_100 =[6,16,26,7,17,27]
    
    for i in indices_to_set_to_100:
        og_array[i] = 100
        
    for i in indices_to_set2_to_100:
        og_array[i] = 100

    for i in range(65, 70):
        og_array[i] = 100

    occupancy_grid_msg.header = Header()
    occupancy_grid_msg.header.stamp = rospy.Time.now()
    occupancy_grid_msg.header.frame_id = 'map'
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
    
    # occupancy_grid_data = np.array(og_array).reshape(occupancy_grid_msg.info.height, occupancy_grid_msg.info.width)
    
    # save_path = "/home/sahan/catkin_ws/src/Path-Planning/node_pkg/src/map/sampleMap.png"
    
    # save_occupancy_grid_as_png(occupancy_grid_data,save_path)
    

if __name__ == '__main__':

    try:
        rospy.init_node('sample_map')
        rospy.loginfo("Map Node has been started")
        og_pub = rospy.Publisher("/custom_occupancy_grid",OccupancyGrid,queue_size=10)
        og_timer = rospy.Timer(rospy.Duration(0.5), og_callback)
        # og_callback(og_pub)

        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS node was interrupted. Exiting...")
        rospy.logerr(e)
