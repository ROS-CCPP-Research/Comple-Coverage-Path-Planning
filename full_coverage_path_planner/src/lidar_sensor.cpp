#include "ros/ros.h"
#include <std_msgs/UInt8.h>
#include "full_coverage_path_planner/lidar_sensor.h"


LaserScannerSimulator::LaserScannerSimulator(ros::NodeHandle *nh)
{
    nh_ptr = nh;
    // get parameters
    get_params();

    // robotCount = ros::topic::waitForMessage<std_msgs::UInt8>("number_of_agents")->data;

    // get map
    get_map();
    ROS_INFO("Initialized laser scanner simulator");
}

LaserScannerSimulator::~LaserScannerSimulator()
{
    if (is_running) stop();
}


void LaserScannerSimulator::get_params()
{
    nh_ptr->param<std::string>("robot_naamespace", robot_naamespace, "robot");
    nh_ptr->param<double>("laser_frequency", l_frequency, 10.0);
}


void LaserScannerSimulator::start()
{
    loop_timer = nh_ptr->createTimer(ros::Duration(1.0/l_frequency),&LaserScannerSimulator::track_lidar_data, this);
    loop_timer.start(); // should not be necessary
    is_running = true;
    ROS_INFO("Started laser scanner simulator update loop");
}

void LaserScannerSimulator::stop()
{
    loop_timer.stop();
    is_running = false;
    ROS_INFO("Stopped laser scanner simulator");
}

void LaserScannerSimulator::track_lidar_data(const ros::TimerEvent& event)
{
    // If we don't have a map, try to get one
    if (!have_map) get_map();
    
    std::cout<<"scanning sensor....."<<std::endl;

    
}

void LaserScannerSimulator::get_map()
{
    nav_msgs::GetMapRequest req;
    nav_msgs::GetMapResponse resp;
    if (ros::service::call(map_service, req, resp))
    {
        map = resp.map;
        ROS_INFO_STREAM("Got a " << map.info.width << "x" << map.info.height << " map with resolution " << map.info.resolution);
        have_map = true;
    }
    else 
    {
        ROS_WARN_THROTTLE(10,"No map received - service '/static_map' not available (will publish only max_range)");
        have_map = false;
    }
}