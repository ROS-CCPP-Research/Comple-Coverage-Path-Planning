#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
import yaml
import numpy as np

def occupancy_grid_callback(msg):
    width = msg.info.width
    height = msg.info.height
    data = msg.data

    # Convert data to a 2D numpy array
    occupancy_data = np.array(data).reshape(height, width)

    # Create a dictionary to store map information
    map_info = {
        'image': 'map.pgm',
        'resolution': msg.info.resolution,
        'origin': {
            'position': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'z': msg.info.origin.position.z,
            },
            'orientation': {
                'x': msg.info.origin.orientation.x,
                'y': msg.info.origin.orientation.y,
                'z': msg.info.origin.orientation.z,
                'w': msg.info.origin.orientation.w,
            }
        }
    }

    # Save the map data to PGM file
    with open('map.pgm', 'wb') as pgm_file:
        pgm_file.write(bytearray("P5\n{} {}\n255\n".format(width, height), 'ascii'))
        np.save(pgm_file, occupancy_data)

    # Save map info to YAML file
    with open('map.yaml', 'w') as yaml_file:
        yaml.dump(map_info, yaml_file, default_flow_style=False)

def save_map():
    rospy.init_node('map_saver', anonymous=True)
    rospy.Subscriber('/custom_occupancy_grid', OccupancyGrid, occupancy_grid_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        save_map()
    except rospy.ROSInterruptException:
        pass
