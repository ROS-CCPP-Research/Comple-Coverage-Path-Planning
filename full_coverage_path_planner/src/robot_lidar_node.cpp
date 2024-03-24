#include "full_coverage_path_planner/spiral_stc.h"
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <unordered_map>
#include <utility> // for std::pair
#include <functional> // For std::hash
#include "full_coverage_path_planner/lidar_sensor.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_scan_node");
    ros::NodeHandle nh;
    

    LaserScannerSimulator laser_sim(&nh);
    ros::AsyncSpinner spinner(1);
    
    ROS_INFO("--- Starting LaserScanner simulator");
    
    ros::Duration(0.5).sleep();
    
    laser_sim.start();
    
    spinner.start();
    while (nh.ok()) {
        //ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    spinner.stop();
    
    laser_sim.stop();
    
    return 0;
}