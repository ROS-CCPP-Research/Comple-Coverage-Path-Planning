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

// using Node = std::pair<int, int>; // Node representation as a pair of coordinates
// using Graph = std::unordered_map<Node, std::vector<Node>>; // Graph as an adjacency list

// // Define the Node type
// using Node = std::pair<int, int>;

// // Specialize std::hash for Node
// namespace std {
// template <>
// struct hash<Node> {
//     size_t operator()(const Node& node) const {
//         // Compute individual hash values for first and second,
//         // and then combine them
//         size_t h1 = std::hash<int>()(node.first);
//         size_t h2 = std::hash<int>()(node.second);

//         // Combine the hash values (example of a simple way to combine)
//         return h1 ^ (h2 << 1); 
//     }
// };
// }

// Graph convertGridToGraph(const std::vector<std::vector<int>>& grid) {
//     Graph graph;
//     int numRows = grid.size();
//     int numCols = grid[0].size();

//     for (int i = 0; i < numRows; ++i) {
//         for (int j = 0; j < numCols; ++j) {
//             if (grid[i][j] == 0) { // Assuming 0 represents a walkable cell
//                 Node node = {i, j};

//                 // Check for possible movements (up, down, left, right)
//                 std::vector<Node> neighbors;
//                 if (i > 0 && grid[i-1][j] == 0) neighbors.push_back({i-1, j}); // Up
//                 if (i < numRows-1 && grid[i+1][j] == 0) neighbors.push_back({i+1, j}); // Down
//                 if (j > 0 && grid[i][j-1] == 0) neighbors.push_back({i, j-1}); // Left
//                 if (j < numCols-1 && grid[i][j+1] == 0) neighbors.push_back({i, j+1}); // Right

//                 // Add the node and its neighbors to the graph
//                 graph[node] = neighbors;
//             }
//         }
//     }

//     return graph;
// }

// void printGraph(const std::unordered_map<Node, std::vector<Node>>& graph) {
//     for (const auto& pair : graph) {
//         const Node& node = pair.first;
//         const std::vector<Node>& neighbors = pair.second;
        
//         std::cout << "Node (" << node.first << ", " << node.second << "): ";
        
//         for (const auto& neighbor : neighbors) {
//             std::cout << "(" << neighbor.first << ", " << neighbor.second << ") ";
//         }
        
//         std::cout << std::endl;
//     }
// }

using PoseStamped = geometry_msgs::PoseStamped;

int countFreeCells(const std::vector<std::vector<bool>>& grid) {
    int freeCells = 0;
    for (const auto& row : grid) {
        for (bool cell : row) {
            if (!cell) freeCells++;
        }
    }
    return freeCells;
}

int floodFillUtil(const std::vector<std::vector<bool>>& grid, std::vector<std::vector<bool>>& visited, std::vector<std::vector<bool>>& subGrid, int i, int j, int& remaining) {
    if (i < 0 || i >= grid.size() || j < 0 || j >= grid[0].size() || visited[i][j] || grid[i][j] || remaining <= 0) {
        return 0;
    }

    visited[i][j] = true;
    subGrid[i][j] = false; // Mark this cell as free in the sub-grid
    remaining--;

    int filled = 1;
    filled += floodFillUtil(grid, visited, subGrid, i + 1, j, remaining);
    filled += floodFillUtil(grid, visited, subGrid, i - 1, j, remaining);
    filled += floodFillUtil(grid, visited, subGrid, i, j + 1, remaining);
    filled += floodFillUtil(grid, visited, subGrid, i, j - 1, remaining);

    return filled;
}

std::vector<std::vector<std::vector<bool>>> partitionGrid(const std::vector<std::vector<bool>>& grid, int noOfRobots) {
    int totalFreeCells = countFreeCells(grid); // Use countFreeCells to get the actual free cells
    std::cout << "Total Free Cells: " << totalFreeCells << std::endl;
    int freeCellsForRobot = totalFreeCells / noOfRobots;
    
    std::vector<std::vector<std::vector<bool>>> partitions(noOfRobots, std::vector<std::vector<bool>>(grid.size(), std::vector<bool>(grid[0].size(), true)));
    std::vector<std::vector<bool>> visited(grid.size(), std::vector<bool>(grid[0].size(), false));
    
    int robot = 0, remaining = freeCellsForRobot;
    for (int i = 0; i < grid.size() && robot < noOfRobots; ++i) {
        for (int j = 0; j < grid[0].size() && robot < noOfRobots; ++j) {
            if (!grid[i][j] && !visited[i][j]) { // If the cell is free and not visited
                // Reset remaining for each robot
                remaining = freeCellsForRobot;
                floodFillUtil(grid, visited, partitions[robot], i, j, remaining);
                robot++; // Move on to the next robot after allocation
            }
        }
    }
    return partitions;
}

// int floodFillUtil(const std::vector<std::vector<bool>>& grid, std::vector<std::vector<bool>>& visited, std::vector<std::vector<bool>>& sub_grid, int i, int j, int& remaining) {
//     if (i < 0 || i >= grid.size() || j < 0 || j >= grid[0].size() || visited[i][j] || grid[i][j] || remaining <= 0) {
//         return 0;
//     }
//     visited[i][j] = true;
//     sub_grid[i][j] = false; // Mark this cell as free in the sub-grid
//     remaining--;
//     int filled = 1;
//     filled += floodFillUtil(grid, visited, sub_grid, i + 1, j, remaining);
//     filled += floodFillUtil(grid, visited, sub_grid, i - 1, j, remaining);
//     filled += floodFillUtil(grid, visited, sub_grid, i, j + 1, remaining);
//     filled += floodFillUtil(grid, visited, sub_grid, i, j - 1, remaining);
//     return filled;
// }

std::vector<std::vector<std::vector<bool>>> floodFill(const std::vector<std::vector<bool>>& grid, int noofRobots) {
    int totalFreeCells = 0; // Calculate the total number of free cells
    for (const auto& row : grid) {
        for (bool cell : row) {
            if (!cell) totalFreeCells++;
        }
    }

    int subFreeCellCount = totalFreeCells / noofRobots; // Number of free cells each robot should cover
    std::vector<std::vector<std::vector<bool>>> sub_grids(noofRobots, std::vector<std::vector<bool>>(grid.size(), std::vector<bool>(grid[0].size(), true))); // Initialize all sub-grids as occupied

    int freeCellsAllocated = 0; // Track the number of free cells allocated
    int startRow = 0, startCol = 0; // Start position for the next sub-grid
    std::vector<std::vector<bool>> visited = grid; // Initialize visited array with the same size as the grid

    for (int robot = 0; robot < noofRobots; robot++) {
        int subGridFreeCellCount = subFreeCellCount; // Reset for each robot

        // Attempt to allocate subFreeCellCount cells to the current robot's sub-grid
        bool found = false;
        for (int i = startRow; i < grid.size() && subGridFreeCellCount > 0; ++i) {
            for (int j = (i == startRow ? startCol : 0); j < grid[0].size() && subGridFreeCellCount > 0; ++j) {
                if (!grid[i][j] && !visited[i][j]) { // If the cell is free and not visited
                    int filled = floodFillUtil(grid, visited, sub_grids[robot], i, j, subGridFreeCellCount);
                    freeCellsAllocated += filled;
                    subGridFreeCellCount -= filled;
                    if (!found) {
                        found = true;
                        // Update the start position for the next sub-grid
                        startRow = i;
                        startCol = j + 1;
                        if (startCol >= grid[0].size()) {
                            startRow++;
                            startCol = 0;
                        }
                    }
                }
            }
        }

        // Check if there are remaining free cells to be allocated to subsequent robots
        if (freeCellsAllocated < totalFreeCells && robot == noofRobots - 1) {
            subFreeCellCount = (totalFreeCells - freeCellsAllocated) / (noofRobots - robot); // Adjust remaining cells for the last robot(s)
        }
    }

    return sub_grids; // Return all sub-grids
}

void printSubGrids(const std::vector<std::vector<std::vector<bool>>>& sub_grids) {
    for (size_t robot = 0; robot < sub_grids.size(); ++robot) {
        std::cout << "Sub-grid for Robot " << (robot + 1) << ":" << std::endl;
        int freeCells = 0; // Counter for free cells in the current sub-grid
        for (const auto& row : sub_grids[robot]) {
            for (bool cell : row) {
                if (!cell) freeCells++; // Increment if the cell is free (false)
                std::cout << (cell ? "1 " : "0 "); // Print 1 for occupied (true), 0 for free (false)
            }
            std::cout << std::endl; // New line at the end of each row
        }
        // Print the count of free cells for this robot's sub-grid
        std::cout << "Free Cells for Robot " << (robot + 1) << ": " << freeCells << std::endl << std::endl;
    }
}


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


    // printGridBinary(explored_free_area_grid);
    // convertGridToGraph(grid)

     std::vector<std::vector<bool>> visited(grid.size(), std::vector<bool>(grid[0].size(), false));

    int numOfRobots = 2;

    // auto sub_grids = floodFill(explored_free_area_grid, numOfRobots);
    auto partitions = partitionGrid(explored_free_area_grid, numOfRobots);
    printSubGrids(partitions);

    int multiple_pass_counter, visited_counter;

    Point_t sub_Scaled;

    sub_Scaled.x = boundary.front().y;
    sub_Scaled.y = boundary.front().x;

    std::list<Point_t> sub_path = full_coverage_path_planner::SpiralSTC::boustrophedon_stc(grid, sub_Scaled, multiple_pass_counter, visited_counter); // Call with appropriate arguments


    int subpart_size = sub_path.size()/robotCount;

    // if(sub_path.size() % 2 == 1){
    //     sub_path.
    // }

    std::cout<<"point len : "<<sub_path.size()<<std::endl;
    std::cout<<"sub len : "<<subpart_size<<std::endl;

    std::list<std::vector<Point_t>> all_partitions;
    std::vector<Point_t> part;

    // printGraph(explored_area_graph,explored_area_visited.size(),explored_area_visited[0].size(),explored_area_visited);

    for(const auto& parts : sub_path){
        part.push_back(parts);

        if(part.size()>=subpart_size){
            all_partitions.push_back(part);
            part.clear();
        }
    }

    std::cout<<"partition count : "<<all_partitions.size()<<std::endl;

    std::list<Point_t> merged_path;

    std::vector<ros::Publisher> sub_waypointPublishers;
    sub_waypointPublishers.reserve(all_partitions.size());

    std::vector<nav_msgs::Path> sub_agentPaths(all_partitions.size());

    size_t index = 0;

    for (std::vector<Point_t> single : all_partitions){

        std::vector<std::vector<bool>> sub_region (nRows, std::vector<bool>(nCols, true)); 

        for (int i = 0; i < nRows; ++i) {
            for (int j = 0; j < nCols; ++j) {
                bool inside_boundary = false;
                for (const auto& point : single) {
                    if (point.y == i && point.x== j) {
                        inside_boundary = true;
                        break;
                    }
                }
                if(inside_boundary){
                    sub_region[i][j] = false;
                }
                
            }
        }
        // std::cout<<"sub start point : "<<"("<<single.front().x<<","<<single.front().y<<")"<<std::endl;
        // printGridBinary(sub_region);

        Point_t sub_region_Scaled;
        int multiple_pass_counter, visited_counter;

        sub_region_Scaled.x = single.front().x;
        sub_region_Scaled.y = single.front().y;
        std::list<Point_t> sub_region_path;

        sub_region_path = full_coverage_path_planner::SpiralSTC::spiral_stc(sub_region, sub_region_Scaled, multiple_pass_counter, visited_counter);

        // sub_region_path = full_coverage_path_planner::SpiralSTC::boustrophedon_stc(sub_region, sub_region_Scaled, multiple_pass_counter, visited_counter);

        planner.sub_paths_array.push_back(sub_region_path);

        std::vector<PoseStamped> sub_plan;
        sub_plan.reserve(sub_region_path.size());
        planner.parsePointlist2Plan(pose, sub_region_path, sub_plan);

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

        // merged_path.insert(merged_path.end(), sub_region_path.begin(),sub_region_path.end());
        if (!merged_path.empty()) {
            auto it = merged_path.end();
            --it; // Move iterator to the last element
            merged_path.insert(it, sub_region_path.begin(), sub_region_path.end());
        } else {
            // If merged_path is empty, just insert at the beginning
            merged_path.insert(merged_path.begin(), sub_region_path.begin(), sub_region_path.end());
        }


        for (const auto& pose : sub_plan)
        {
            sub_agentPaths[index].poses.push_back(pose);
        }

        sub_waypointPublishers[index].publish(sub_agentPaths[index]);

        ++index;
        // break;

    }

    // int explored_width = grid.size();
    // int explored_height = grid[0].size();
    // std::list<std::vector<Point_t>> sub_regions = full_coverage_path_planner::SpiralSTC::explore_subregions(grid,explored_width,explored_height,robotCount);

    // std::cout<<"sub count"<<sub_regions.size()<<std::endl;

    // std::cout<<"------------------------------################################----------------------------------"<<std::endl;

    // std::vector<std::vector<bool>> sub_grid;

    // std::list<Point_t> merged_path;

    // std::vector<ros::Publisher> sub_waypointPublishers;
    // sub_waypointPublishers.reserve(sub_regions.size());

    // std::vector<nav_msgs::Path> sub_agentPaths(sub_regions.size());

    // size_t index = 0;

    // for (const auto& sub_region : sub_regions) {

    //     Point_t start_position = sub_region.front();
    //     Point_t end_position = sub_region.back();

    //     // Extract sub-grid boundaries from sub_region points
    //     int min_x = std::min_element(sub_region.begin(), sub_region.end())->x;
    //     int max_x = std::max_element(sub_region.begin(), sub_region.end())->x;
    //     int min_y = std::min_element(sub_region.begin(), sub_region.end())->y;
    //     int max_y = std::max_element(sub_region.begin(), sub_region.end())->y;


    // std::cout << "Start position of sub-region according to main grid: (" << start_position.x << "," << start_position.y <<")" << std::endl;
    // std::cout << "End position of sub-region according to main grid: (" << end_position.x << "," << end_position.y <<")" << std::endl;
    

    // // Copy relevant portion from the main grid
    // sub_grid.clear();
    // for (int y = min_y; y <= max_y; ++y) {
    //     sub_grid.push_back(std::vector<bool>(grid[y].begin() + min_x, grid[y].begin() + max_x + 1));
    // }

    // PoseStamped sub_Pose;  // This is the point from which path planning starts

    // sub_Pose.pose.position.x = start_position.x - min_x;
    // sub_Pose.pose.position.y = start_position.y - min_y;
    // sub_Pose.pose.position.z = 0.0; // Assuming z-coordinate is 0
    // sub_Pose.pose.orientation.x = 0.0; // Assuming orientation x,y,z,w are all 0
    // sub_Pose.pose.orientation.y = 0.0;
    // sub_Pose.pose.orientation.z = 0.0;
    // sub_Pose.pose.orientation.w = 1.0; // Assuming orientation w is 1 for no rotation
    // sub_Pose.header.frame_id = "map"; // Assuming the frame is "map"


    // std::cout<<"start position of sub grid : "<<sub_Pose.pose.position<<std::endl;
    // std::cout<<"sub grid value start position : "<<sub_grid[sub_Pose.pose.position.x][sub_Pose.pose.position.y]<<std::endl;


    // Point_t sub_Scaled;
    // fPoint_t sub_grid_origin_;
    
    // sub_grid_origin_.x = 0.0;  // x-sub-grid-origin in meters (occ_grid.info.origin.position.x)
    // sub_grid_origin_.y = 0.0;  // y-sub-grid-origin in meters (occ_grid.info.origin.position.y)

    // sub_Scaled.x = static_cast<unsigned int>(clamp(sub_Pose.pose.position.x / planner.tile_size_, 0.0,
    //                          floor(sub_grid.size() / planner.tile_size_)));
    // sub_Scaled.y = static_cast<unsigned int>(clamp(sub_Pose.pose.position.y / planner.tile_size_, 0.0,
    //                          floor(sub_grid[0].size() / planner.tile_size_)));


    
    // std::cout<< "sub_Scaled point"<<sub_Scaled<<std::endl;

    // int sub_nRows = sub_grid.size();
    // int sub_nCols = sub_grid[0].size();

    // std::vector<std::vector<bool>> sub_visited(sub_nRows, std::vector<bool>(sub_nCols, false));
    // std::vector<std::vector<Node*>> graph(sub_nRows, std::vector<Node*>(sub_nCols, nullptr));

    // // for (int i = 0; i < sub_nRows; ++i) {
    // //     for (int j = 0; j < sub_nCols; ++j) {
    // //         if (grid[i][j] == 0 && !sub_visited[i][j]) {
    // //             bfs(i, j,sub_nRows,sub_nCols,sub_grid, sub_visited,graph);
    // //         }
    // //     }
    // // }
    // // printGraph(graph,sub_visited.size(),sub_visited[0].size(),sub_visited);
    // // visualizeGraph(graph);


    // int multiple_pass_counter, visited_counter;

    // std::list<Point_t> sub_path = full_coverage_path_planner::SpiralSTC::new_spiral_stc(sub_grid, sub_Scaled, multiple_pass_counter, visited_counter);
    
    // // std::list<Point_t> sub_path = full_coverage_path_planner::SpiralSTC::boustrophedon_stc(sub_grid, sub_Scaled, multiple_pass_counter, visited_counter); // Call with appropriate arguments
    
    // std::cout<<"Lenght : "<<sub_path.size()<<std::endl;

    // std::cout<< "sub grid start path before offset : "<<sub_path.front()<<std::endl;
    // std::cout<< "sub grid end path before offset : "<<sub_path.back()<<std::endl;

    // std::cout << "Sub-path coordinates before offset:" << std::endl;
    // for (const auto& point : sub_path) {
    //     std::cout << "(" << point.x << ", " << point.y << ")";
    // }
    // std::cout<<std::endl;

    // std::cout<<"##############################################################################################"<<std::endl;

    // for (auto& point : sub_path) {
    //     point.x += min_x; // Adjust x-coordinate by the minimum x-coordinate of the sub-grid
    //     point.y += min_y; // Adjust y-coordinate by the minimum y-coordinate of the sub-grid
    // }
    
    // std::cout<<"Sub path start after offset : "<<sub_path.front()<<std::endl;
    // std::cout<<"Sub path end after offset : "<<sub_path.back()<<std::endl;

    // std::cout << "Sub-path coordinates after offset:" << std::endl;
    // for (const auto& point : sub_path) {
    //     std::cout << "(" << point.x << ", " << point.y << ")";
    // }
    // std::cout<<std::endl;

    // printGridBinary(sub_grid);

    // std::cout<<"-------------"<<std::endl;

    // planner.sub_paths_array.push_back(sub_path);

    // std::vector<PoseStamped> sub_plan;
    // sub_plan.reserve(sub_path.size());
    // planner.parsePointlist2Plan(pose, sub_path, sub_plan);

    // std::stringstream ss;
    // ss << robotNamespace << "_" << index << "/waypoints";
    // sub_waypointPublishers.push_back(nh.advertise<nav_msgs::Path>(ss.str(), 100, true));

    // sub_agentPaths[index].header.frame_id = "map";
    // sub_agentPaths[index].header.stamp = ros::Time::now();
    // sub_agentPaths[index].poses.reserve(sub_plan.end() - sub_plan.begin());

    // for (const auto& pose : sub_plan)
    // {
    //     sub_agentPaths[index].poses.push_back(pose);
    // }

    // sub_waypointPublishers[index].publish(sub_agentPaths[index]);

    // merged_path.splice(merged_path.end(), sub_path);

    // // Convert sub-path points back to original grid coordinates
    // // ... adjust path points based on sub-region origin

    // // Store the sub-path in a data structure (e.g., list or vector)
    // // ... (replace placeholder)
    // ++index;
    // break;
    // }
    
    
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
