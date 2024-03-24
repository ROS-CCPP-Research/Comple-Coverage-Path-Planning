#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"

#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

#include <math.h>
#include <random>


#ifndef LASER_SIMULATOR
#define LASER_SIMULATOR

class LaserScannerSimulator {

public:

    LaserScannerSimulator(ros::NodeHandle *nh);
    ~LaserScannerSimulator();

    void start(); // 
    void stop(); // 

private:

    void track_lidar_data(const ros::TimerEvent& event);
    
    /*! gets the current map */
    void get_map();

    void get_params();


    ros::NodeHandle * nh_ptr;

    ros::Timer loop_timer; // timer for the update loop
    bool is_running;
    
    // map 
    std::string map_service;
    nav_msgs::OccupancyGrid map; //map data
    bool have_map;

    std::string robot_naamespace;

    double l_frequency; // frequency of laser scans
    size_t robotCount;

};

#endif