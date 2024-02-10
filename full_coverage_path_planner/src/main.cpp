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

using PoseStamped = geometry_msgs::PoseStamped;

inline bool operator<(const Point_t& lhs, const Point_t& rhs) {
    // Compare based on x and y values
    return (lhs.x < rhs.x) || (lhs.x == rhs.x && lhs.y < rhs.y);
}

static double euDist2D(const PoseStamped& p1, const PoseStamped& p2)
{
    return sqrt(pow(p1.pose.position.x - p2.pose.position.x, 2) + pow(p1.pose.position.y - p2.pose.position.y, 2));
}

static PoseStamped parametricInterp1(const PoseStamped& p1, const PoseStamped& p2, double alpha)
{
    PoseStamped p;
    p.header.frame_id = p1.header.frame_id;
    p.pose.orientation = p1.pose.orientation;
    p.pose.position.x = p1.pose.position.x + alpha * (p2.pose.orientation.x - p1.pose.orientation.x);
    p.pose.position.y = p1.pose.position.y + alpha * (p2.pose.orientation.y - p1.pose.orientation.y);
    return p;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    std::string start_pose, robotNamespace;
    float robotRadius,tool_radius;
    int sub_width;
    int sub_height;

    // Load Parameters
    ros::param::get("~start_pose", start_pose);
    ros::param::get("~robot_radius", robotRadius);
    ros::param::get("~tool_radius", tool_radius);
    ros::param::get("~robot_namespace", robotNamespace);
    ros::param::get("~sub_width", sub_width);
    ros::param::get("~sub_height", sub_height);

    auto occ = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map");


    const nav_msgs::OccupancyGrid& occ_grid = *occ;

    std::vector<std::vector<bool>> grid;  // Binary matrix for path planning
    full_coverage_path_planner::SpiralSTC planner;
    Point_t scaled;    // This will hold the index of the start location in binary matrix
    PoseStamped pose;  // This is the point from which path planning starts
    {
        std::stringstream ss(start_pose);
        ss >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z;
        double roll, pitch, yaw;
        ss >> roll >> pitch >> yaw;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.header.frame_id = "map";
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
    }

    // Convert occupancy grid to binary matrix
    planner.parseGrid(*occ, grid, robotRadius, tool_radius, pose, scaled);


    // std::list<std::vector<Point_t>> sub_regions = full_coverage_path_planner::SpiralSTC::boustrophedon_subregions(grid,sub_width,sub_height);

    // std::cout<<"subb count"<<sub_regions.size()<<std::endl;


    // std::vector<std::vector<bool>> sub_grid;

    // for (const auto& sub_region : sub_regions) {

    //     Point_t start_position = sub_region.front();
    //     // Extract sub-grid boundaries from sub_region points
    //     int min_x = std::min_element(sub_region.begin(), sub_region.end())->x;
    //     int max_x = std::max_element(sub_region.begin(), sub_region.end())->x;
    //     int min_y = std::min_element(sub_region.begin(), sub_region.end())->y;
    //     int max_y = std::max_element(sub_region.begin(), sub_region.end())->y;

    // // std::cout << "Start position of sub-region: (" << start_position.x << "," << start_position.y <<")" << std::endl;

    // // Copy relevant portion from the main grid
    // sub_grid.clear();
    // for (int y = min_y; y <= max_y; ++y) {
    //     sub_grid.push_back(std::vector<bool>(grid[y].begin() + min_x, grid[y].begin() + max_x + 1));
    // }


    // PoseStamped sub_Pose;  // This is the point from which path planning starts

    // pose.pose.position.x = start_position.x;
    // pose.pose.position.y = start_position.y;
    // pose.pose.position.z = 0.0; // Assuming z-coordinate is 0
    // pose.pose.orientation.x = 0.0; // Assuming orientation x,y,z,w are all 0
    // pose.pose.orientation.y = 0.0;
    // pose.pose.orientation.z = 0.0;
    // pose.pose.orientation.w = 1.0; // Assuming orientation w is 1 for no rotation
    // pose.header.frame_id = "map"; // Assuming the frame is "map"


    // Point_t sub_Scaled;

    // sub_Scaled.x = static_cast<unsigned int>(clamp((pose.pose.position.x - occ_grid.info.origin.position.x) / dmax(floor(tool_radius / occ_grid.info.resolution), 1) * occ_grid.info.resolution, 0.0,
    //                          floor(occ_grid.info.width / dmax(floor(tool_radius / occ_grid.info.resolution), 1) * occ_grid.info.resolution)));
    // sub_Scaled.y = static_cast<unsigned int>(clamp((pose.pose.position.y - occ_grid.info.origin.position.y) / dmax(floor(tool_radius / occ_grid.info.resolution), 1) * occ_grid.info.resolution, 0.0,
    //                          floor(occ_grid.info.height / dmax(floor(tool_radius / occ_grid.info.resolution), 1) * occ_grid.info.resolution)));

    

    // int multiple_pass_counter, visited_counter;
    
    // std::list<Point_t> sub_path = full_coverage_path_planner::SpiralSTC::boustrophedon_stc(sub_grid, sub_Scaled, multiple_pass_counter, visited_counter); // Call with appropriate arguments

    // std::cout<< "sub grid start path"<<"("<<sub_path.front().x<<","<<sub_path.front().y<<")"<<std::endl;
    // std::cout<< "sub grid end path"<<"("<<sub_path.back().x<<","<<sub_path.back().y<<")"<<std::endl;
    // std::cout<<"Lenght"<<sub_path.size()<<std::endl;

    // std::cout<<"-------------"<<std::endl;

    // Convert sub-path points back to original grid coordinates
    // ... adjust path points based on sub-region origin

    // Store the sub-path in a data structure (e.g., list or vector)
    // ... (replace placeholder)
    



    // Boustrophedon path planning
    int multiple_pass_counter, visited_counter;
    // std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid, scaled, multiple_pass_counter, visited_counter);

    std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::boustrophedon_stc(grid, scaled, multiple_pass_counter, visited_counter);

    // path has the indices of corner points in the path, we now convert that to real world coordinates
    std::vector<PoseStamped> plan;
    plan.reserve(path.size());
    planner.parsePointlist2Plan(pose, path, plan);

    auto planPub = nh.advertise<nav_msgs::Path>("boustrophedon/path", 10, true);
    auto pathMsg = nav_msgs::Path();
    pathMsg.header.frame_id = "map";
    pathMsg.header.stamp = ros::Time::now();
    pathMsg.poses = plan;
    planPub.publish(pathMsg);

    // // Inorder to divide the path among agents, we need more points in between the corner points of the path.
    // // We just perform a linear parametric up-sampling
    std::vector<PoseStamped> upSampled;

    ROS_INFO("Path computed. Up-sampling...");
    for (size_t i = 0; i < plan.size() - 1; ++i)
    {
        double mul = 0.5 * euDist2D(plan[i], plan[i + 1]) / floor(euDist2D(plan[i], plan[i + 1]) / robotRadius);
        double a = 0;
        while (a < 1)
        {
            upSampled.push_back(parametricInterp1(plan[i], plan[i + 1], a));
            a += mul;
        }
        upSampled.push_back(plan[i + 1]);
    }
    ROS_INFO("Up-sampling complete");
    ROS_INFO("Waiting for number of agents");

    // Prepare publishers
    size_t nAgents = ros::topic::waitForMessage<std_msgs::UInt8>("number_of_agents")->data;
    std::vector<ros::Publisher> waypointPublishers;
    waypointPublishers.reserve(nAgents);
    for (auto i = 0; i < nAgents; ++i)
    {
        std::stringstream ss;
        ss << robotNamespace << "_" << i << "/waypoints";
        waypointPublishers.push_back(nh.advertise<nav_msgs::Path>(ss.str(), 100, true));
    }

    // Now divide the path approximately equally for each agent
    std::vector<nav_msgs::Path> agentPaths(nAgents);

    size_t length = upSampled.size() / nAgents;
    size_t leftover = upSampled.size() % nAgents;
    size_t begin = 0, end = 0;

    ROS_INFO("Publishing paths");
    for (size_t i = 0; i < std::min(nAgents, upSampled.size()); ++i)
    {
        end += leftover > 0 ? (length + !!(leftover--)) : length;

        agentPaths[i].header.frame_id = "map";
        agentPaths[i].header.stamp = ros::Time::now();
        agentPaths[i].poses.reserve(end - begin);

        for (size_t j = begin; j < end; ++j)
        {
            agentPaths[i].poses.push_back(upSampled[j]);
        }
        waypointPublishers[i].publish(agentPaths[i]);
        begin = end;
    }
    ROS_ERROR_STREAM(upSampled.size());
    size_t s = 0;
    for (size_t i = 0; i < nAgents; ++i)
    {
        s += agentPaths[i].poses.size();
    }
    ROS_ERROR_STREAM(s);
    ros::spin();
    return EXIT_SUCCESS;
}
