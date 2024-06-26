#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import cv2
import numpy as np
import os
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.srv import SetMap, SetMapRequest


class MapDecomposer:
    
    def __init__(self):
        self.og_sub = rospy.Subscriber('/map', OccupancyGrid, self.og_callback)
        # self.og_sub = rospy.Subscriber('/sampleMap', OccupancyGrid, self.og_callback)
        # self.og_sub = rospy.Subscriber("/square_map", OccupancyGrid, self.og_callback)
        self.partition_pubs = [rospy.Publisher(f"/partition_map_{i}", OccupancyGrid, queue_size=10) for i in range(1, 5)]

        self.map_services = [
            rospy.Service(f"/update_map_{i}", Empty, self.update_map_callback(i))
            for i in range(1, 5)
        ]
        
        self.static_map_service = rospy.ServiceProxy('/static_map', SetMap)
        
    def update_map_callback(self, partition_number):
        def callback(request):
            self.partition_pubs[partition_number - 1].publish()  # Publish the partitioned map
            rospy.loginfo(f"Partitioned map {partition_number} published")
            
            # If partition_number is 1, set it as the static_map
            if partition_number == 1:
                self.set_static_map(self.partition_pubs[0])

            return EmptyResponse()  # Return an empty response as acknowledgment
        return callback
          
    def update_map_callback(self, partition_number):
        def callback(request):
            self.partition_pubs[partition_number - 1].publish()  # Publish the partitioned map
            rospy.loginfo(f"Partitioned map {partition_number} published")
            return EmptyResponse()  # Return an empty response as acknowledgment
        return callback
    
    def set_static_map(self, map_data_publisher):
        # Assuming map_data_publisher is the publisher for partition_map_1
        map_data = rospy.wait_for_message(map_data_publisher.name, OccupancyGrid)
        
        # Prepare the request for SetMap service
        set_map_request = SetMapRequest()
        set_map_request.map = map_data
        
        # Call the SetMap service
        response = self.static_map_service(set_map_request)
        
        if response.success:
            rospy.loginfo("Partitioned map set as the static_map.")
        else:
            rospy.logerr("Failed to set the partitioned map as the static_map.")
    
    def og_callback(self, msg):
        input_map_data = msg

        if isinstance(input_map_data, OccupancyGrid):
            input_info = input_map_data.info
            # rospy.loginfo(input_map_data.data)

            sub_maps = [OccupancyGrid() for _ in range(4)]

            sub_width = input_info.width // 2
            sub_height = input_info.height // 2

            for i in range(4):
                sub_maps[i].header = Header()
                sub_maps[i].header.stamp = rospy.Time.now()
                sub_maps[i].header.frame_id = input_map_data.header.frame_id  # Set frame_id consistent with the input map

                sub_maps[i].info = MapMetaData()
                sub_maps[i].info.resolution = input_info.resolution
                sub_maps[i].info.width = sub_width
                sub_maps[i].info.height = sub_height

                sub_maps[i].info.origin.position.x = input_info.origin.position.x
                sub_maps[i].info.origin.position.y = input_info.origin.position.y
                sub_maps[i].info.origin.position.x = input_info.origin.position.x + i % 2 * sub_width * input_info.resolution
                sub_maps[i].info.origin.position.y = input_info.origin.position.y + i // 2 * sub_height * input_info.resolution

                sub_maps[i].info.origin.position.z = input_info.origin.position.z 
                sub_maps[i].info.origin.orientation.x = 0
                sub_maps[i].info.origin.orientation.y = 0
                sub_maps[i].info.origin.orientation.z = 0
                sub_maps[i].info.origin.orientation.w = 1

                # Copy the values from the original map to the sub-map
                start_row = i // 2 * sub_height
                end_row = start_row + sub_height
                start_col = i % 2 * sub_width
                end_col = start_col + sub_width

                sub_map_values = []
                for row in range(start_row, end_row):
                    sub_map_values.extend(input_map_data.data[row * input_info.width + start_col : row * input_info.width + end_col])

                sub_maps[i].data = sub_map_values
                
                # og_array = [0] * (sub_width * sub_height)
                # sub_maps[i].data = og_array

                # self.partition_pubs[i - 1].publish(sub_maps[i])
                # rospy.loginfo(f"Partitioned map {i} published")
            rate = rospy.Rate(10) # 10hz
            while not rospy.is_shutdown():
                hello_str = "hello world %s" % rospy.get_time()
                # rospy.loginfo(hello_str)
                for i in range(4):
                    self.partition_pubs[i].publish(sub_maps[i] )
                rate.sleep()
                
if __name__ == '__main__':
    try:
        rospy.init_node('map_decomposer')
        rospy.loginfo("Map Decomposer Node has been started")
        map_decomposer = MapDecomposer()
        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr("ROS node was interrupted. Exiting...")
        rospy.logerr(e)
