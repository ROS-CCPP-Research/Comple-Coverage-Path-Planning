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
    size_t robotCount = ros::topic::waitForMessage<std_msgs::UInt8>("number_of_agents")->data;


    const nav_msgs::OccupancyGrid& occ_grid = *occ;

    std::vector<std::vector<bool>> grid;  // Binary matrix for path planning
    full_coverage_path_planner::SpiralSTC planner;
    Point_t scaled;    // This will hold the index of the start location in binary matrix
    PoseStamped pose;  // This is the point from which path planning starts
    // int explored_height, explored_width;
    
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

    // std::cout<<"scalled x : "<<scaled.x<<","<<"scalled y  : "<<scaled.y<<std::endl;

    // printGridBinary(grid);

    int nRows = grid.size();
    int nCols = grid[0].size();

    std::cout<<"robotCount"<<robotCount<<std::endl;

    std::vector<std::vector<bool>> free_visited(grid.size(), std::vector<bool>(grid[0].size(), false));

    std::vector<Point_t> boundary;

    explore_free_area(grid, scaled.x, scaled.y, free_visited, boundary);  

    std::vector<std::vector<bool>> explored_free_area_grid (nRows, std::vector<bool>(nCols, false));  

    create_explored_grid(grid,boundary,explored_free_area_grid);

    std::cout<<"start point : "<<boundary.front()<<std::endl;

    printGridBinary(explored_free_area_grid);

    std::vector<std::vector<bool>> explored_area_visited(nRows, std::vector<bool>(nCols, false));
    std::vector<std::vector<Node*>> explored_area_graph(nRows, std::vector<Node*>(nCols, nullptr));
    Node* node = nullptr;

    for (int i = 0; i < nRows; ++i) {
        for (int j = 0; j < nCols; ++j) {
            if (explored_free_area_grid[i][j] == 0 && !explored_area_visited[i][j]) {
                bfs(i, j,nRows,nCols,explored_free_area_grid, explored_area_visited,explored_area_graph,node);
            }
        }
    }

    int single_partitin_point_count = boundary.size()/robotCount;

    std::cout<<"toral size : "<<boundary.size()<<std::endl;
    std::cout<<"one size : "<<single_partitin_point_count<<std::endl;

    std::vector<std::vector<Node*>> partitions;
    std::vector<Node*> partition_point;

    preOrderPartition(node,single_partitin_point_count,partitions,partition_point);

    std::cout<<"par len : "<<partitions.size()<<std::endl;

    // printGraph(explored_area_graph,explored_area_visited.size(),explored_area_visited[0].size(),explored_area_visited);


    // std::list<std::vector<Point_t>> sub_regions = partition_free_area(boundary, robotCount);

    // std::cout << "\nPartitioned Areas:\n";
    // int partition_index = 1;
    // for (std::vector<Point_t>& partition : sub_regions) {
    //     std::cout << "Partition " << partition_index++ << ":\n";
    //     print_matrix(grid,partition);
    // }


    int explored_width = grid.size();
    int explored_height = grid[0].size();
    std::list<std::vector<Point_t>> sub_regions = full_coverage_path_planner::SpiralSTC::explore_subregions(grid,explored_width,explored_height,robotCount);

    std::cout<<"sub count"<<sub_regions.size()<<std::endl;

    std::cout<<"------------------------------################################----------------------------------"<<std::endl;

    std::vector<std::vector<bool>> sub_grid;

    std::list<Point_t> merged_path;

    std::vector<ros::Publisher> sub_waypointPublishers;
    sub_waypointPublishers.reserve(sub_regions.size());

    std::vector<nav_msgs::Path> sub_agentPaths(sub_regions.size());

    size_t index = 0;

    for (const auto& sub_region : sub_regions) {

        Point_t start_position = sub_region.front();
        Point_t end_position = sub_region.back();

        // Extract sub-grid boundaries from sub_region points
        int min_x = std::min_element(sub_region.begin(), sub_region.end())->x;
        int max_x = std::max_element(sub_region.begin(), sub_region.end())->x;
        int min_y = std::min_element(sub_region.begin(), sub_region.end())->y;
        int max_y = std::max_element(sub_region.begin(), sub_region.end())->y;


    std::cout << "Start position of sub-region according to main grid: (" << start_position.x << "," << start_position.y <<")" << std::endl;
    std::cout << "End position of sub-region according to main grid: (" << end_position.x << "," << end_position.y <<")" << std::endl;
    

    // Copy relevant portion from the main grid
    sub_grid.clear();
    for (int y = min_y; y <= max_y; ++y) {
        sub_grid.push_back(std::vector<bool>(grid[y].begin() + min_x, grid[y].begin() + max_x + 1));
    }

    PoseStamped sub_Pose;  // This is the point from which path planning starts

    sub_Pose.pose.position.x = start_position.x - min_x;
    sub_Pose.pose.position.y = start_position.y - min_y;
    sub_Pose.pose.position.z = 0.0; // Assuming z-coordinate is 0
    sub_Pose.pose.orientation.x = 0.0; // Assuming orientation x,y,z,w are all 0
    sub_Pose.pose.orientation.y = 0.0;
    sub_Pose.pose.orientation.z = 0.0;
    sub_Pose.pose.orientation.w = 1.0; // Assuming orientation w is 1 for no rotation
    sub_Pose.header.frame_id = "map"; // Assuming the frame is "map"


    std::cout<<"start position of sub grid : "<<sub_Pose.pose.position<<std::endl;
    std::cout<<"sub grid value start position : "<<sub_grid[sub_Pose.pose.position.x][sub_Pose.pose.position.y]<<std::endl;


    Point_t sub_Scaled;
    fPoint_t sub_grid_origin_;
    
    sub_grid_origin_.x = 0.0;  // x-sub-grid-origin in meters (occ_grid.info.origin.position.x)
    sub_grid_origin_.y = 0.0;  // y-sub-grid-origin in meters (occ_grid.info.origin.position.y)

    sub_Scaled.x = static_cast<unsigned int>(clamp(sub_Pose.pose.position.x / planner.tile_size_, 0.0,
                             floor(sub_grid.size() / planner.tile_size_)));
    sub_Scaled.y = static_cast<unsigned int>(clamp(sub_Pose.pose.position.y / planner.tile_size_, 0.0,
                             floor(sub_grid[0].size() / planner.tile_size_)));


    
    std::cout<< "sub_Scaled point"<<sub_Scaled<<std::endl;

    int sub_nRows = sub_grid.size();
    int sub_nCols = sub_grid[0].size();

    std::vector<std::vector<bool>> sub_visited(sub_nRows, std::vector<bool>(sub_nCols, false));
    std::vector<std::vector<Node*>> graph(sub_nRows, std::vector<Node*>(sub_nCols, nullptr));

    // for (int i = 0; i < sub_nRows; ++i) {
    //     for (int j = 0; j < sub_nCols; ++j) {
    //         if (grid[i][j] == 0 && !sub_visited[i][j]) {
    //             bfs(i, j,sub_nRows,sub_nCols,sub_grid, sub_visited,graph);
    //         }
    //     }
    // }
    // printGraph(graph,sub_visited.size(),sub_visited[0].size(),sub_visited);
    // visualizeGraph(graph);


    int multiple_pass_counter, visited_counter;

    std::list<Point_t> sub_path = full_coverage_path_planner::SpiralSTC::new_spiral_stc(sub_grid, sub_Scaled, multiple_pass_counter, visited_counter);
    
    // std::list<Point_t> sub_path = full_coverage_path_planner::SpiralSTC::boustrophedon_stc(sub_grid, sub_Scaled, multiple_pass_counter, visited_counter); // Call with appropriate arguments
    
    std::cout<<"Lenght : "<<sub_path.size()<<std::endl;

    std::cout<< "sub grid start path before offset : "<<sub_path.front()<<std::endl;
    std::cout<< "sub grid end path before offset : "<<sub_path.back()<<std::endl;

    std::cout << "Sub-path coordinates before offset:" << std::endl;
    for (const auto& point : sub_path) {
        std::cout << "(" << point.x << ", " << point.y << ")";
    }
    std::cout<<std::endl;

    std::cout<<"##############################################################################################"<<std::endl;

    for (auto& point : sub_path) {
        point.x += min_x; // Adjust x-coordinate by the minimum x-coordinate of the sub-grid
        point.y += min_y; // Adjust y-coordinate by the minimum y-coordinate of the sub-grid
    }
    
    std::cout<<"Sub path start after offset : "<<sub_path.front()<<std::endl;
    std::cout<<"Sub path end after offset : "<<sub_path.back()<<std::endl;

    std::cout << "Sub-path coordinates after offset:" << std::endl;
    for (const auto& point : sub_path) {
        std::cout << "(" << point.x << ", " << point.y << ")";
    }
    std::cout<<std::endl;

    printGridBinary(sub_grid);

    std::cout<<"-------------"<<std::endl;

    planner.sub_paths_array.push_back(sub_path);

    std::vector<PoseStamped> sub_plan;
    sub_plan.reserve(sub_path.size());
    planner.parsePointlist2Plan(pose, sub_path, sub_plan);

    std::stringstream ss;
    ss << robotNamespace << "_" << index << "/waypoints";
    sub_waypointPublishers.push_back(nh.advertise<nav_msgs::Path>(ss.str(), 100, true));

    sub_agentPaths[index].header.frame_id = "map";
    sub_agentPaths[index].header.stamp = ros::Time::now();
    sub_agentPaths[index].poses.reserve(sub_plan.end() - sub_plan.begin());

    for (const auto& pose : sub_plan)
    {
        sub_agentPaths[index].poses.push_back(pose);
    }

    sub_waypointPublishers[index].publish(sub_agentPaths[index]);

    merged_path.splice(merged_path.end(), sub_path);

    // Convert sub-path points back to original grid coordinates
    // ... adjust path points based on sub-region origin

    // Store the sub-path in a data structure (e.g., list or vector)
    // ... (replace placeholder)
    ++index;
    break;
    }
    
    
    // Boustrophedon path planning

    // int multiple_pass_counter, visited_counter;

    // std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::spiral_stc(grid, scaled, multiple_pass_counter, visited_counter);

    // std::list<Point_t> path = full_coverage_path_planner::SpiralSTC::boustrophedon_stc(grid, scaled, multiple_pass_counter, visited_counter);

    // path has the indices of corner points in the path, we now convert that to real world coordinates

    std::vector<PoseStamped> plan;
    plan.reserve(merged_path.size());
    planner.parsePointlist2Plan(pose, merged_path, plan);

    auto planPub = nh.advertise<nav_msgs::Path>("boustrophedon/path", 10, true);
    auto pathMsg = nav_msgs::Path();
    pathMsg.header.frame_id = "map";
    pathMsg.header.stamp = ros::Time::now();
    pathMsg.poses = plan;
    planPub.publish(pathMsg);

    // // // Inorder to divide the path among agents, we need more points in between the corner points of the path.
    // // // We just perform a linear parametric up-sampling
    // std::vector<PoseStamped> upSampled;

    // ROS_INFO("Path computed. Up-sampling...");
    // for (size_t i = 0; i < plan.size() - 1; ++i)
    // {
    //     double mul = 0.5 * euDist2D(plan[i], plan[i + 1]) / floor(euDist2D(plan[i], plan[i + 1]) / robotRadius);
    //     double a = 0;
    //     while (a < 1)
    //     {
    //         upSampled.push_back(parametricInterp1(plan[i], plan[i + 1], a));
    //         a += mul;
    //     }
    //     upSampled.push_back(plan[i + 1]);
    // }
    // ROS_INFO("Up-sampling complete");
    // ROS_INFO("Waiting for number of agents");

    // // // Prepare publishers
    // size_t nAgents = ros::topic::waitForMessage<std_msgs::UInt8>("number_of_agents")->data;
    // std::vector<ros::Publisher> waypointPublishers;
    // waypointPublishers.reserve(nAgents);
    // for (auto i = 0; i < nAgents; ++i)
    // {
    //     std::stringstream ss;
    //     ss << robotNamespace << "_" << i << "/waypoints";
    //     waypointPublishers.push_back(nh.advertise<nav_msgs::Path>(ss.str(), 100, true));
    // }

    // // Now divide the path approximately equally for each agent
    // std::vector<nav_msgs::Path> agentPaths(nAgents);

    // size_t length = upSampled.size() / nAgents;
    // size_t leftover = upSampled.size() % nAgents;
    // size_t begin = 0, end = 0;

    // ROS_INFO("Publishing paths");
    // for (size_t i = 0; i < std::min(nAgents, upSampled.size()); ++i)
    // {
    //     end += leftover > 0 ? (length + !!(leftover--)) : length;

    //     agentPaths[i].header.frame_id = "map";
    //     agentPaths[i].header.stamp = ros::Time::now();
    //     agentPaths[i].poses.reserve(end - begin);

    //     for (size_t j = begin; j < end; ++j)
    //     {
    //         agentPaths[i].poses.push_back(upSampled[j]);
    //     }
    //     waypointPublishers[i].publish(agentPaths[i]);
    //     begin = end;
    // }

    // ROS_ERROR_STREAM(upSampled.size());
    // size_t s = 0;
    // for (size_t i = 0; i < nAgents; ++i)
    // {
    //     s += agentPaths[i].poses.size();
    // }
    // ROS_ERROR_STREAM(s);

    ros::spin();
    return EXIT_SUCCESS;
}
