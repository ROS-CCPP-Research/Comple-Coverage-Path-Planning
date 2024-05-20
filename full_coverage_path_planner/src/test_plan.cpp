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
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <queue>


void convertToOccupancyGrid(const std::vector<std::vector<bool>>& matrix, nav_msgs::OccupancyGrid& grid) {
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = 1.0; // Set the resolution of the grid (meters/cell)
    grid.info.width = matrix[0].size();
    grid.info.height = matrix.size();
    grid.info.origin.position.x = -(static_cast<double>(grid.info.width) / 2.0) * grid.info.resolution;
    grid.info.origin.position.y = -(static_cast<double>(grid.info.height) / 2.0) * grid.info.resolution;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(grid.info.width * grid.info.height);

    for (size_t i = 0; i < matrix.size(); ++i) {
        for (size_t j = 0; j < matrix[i].size(); ++j) {
            int value = matrix[i][j] ? 100 : 0; // Occupied if true, free if false
            grid.data[i * grid.info.width + j] = value;
        }
    }
}

nav_msgs::Path convertToPath(const std::list<Point_t>& points, const nav_msgs::OccupancyGrid& grid) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    double origin_x = grid.info.origin.position.x;
    double origin_y = grid.info.origin.position.y;
    double resolution = grid.info.resolution;

    for (const auto& point : points) {
        geometry_msgs::PoseStamped pose;
        pose.header = path.header;
        // Adjust the point coordinates according to the grid origin and resolution
        pose.pose.position.x = (point.x * resolution) + origin_x + (resolution / 2.0);
        pose.pose.position.y = (point.y * resolution) + origin_y + (resolution / 2.0);
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    return path;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_plan");
    ros::NodeHandle nh;


std::vector<std::vector<bool>> test_matrix_01 = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

std::vector<std::vector<bool>> test_matrix_02 = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

std::vector<std::vector<bool>> test_matrix_03 = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};



std::vector<std::vector<bool>> test_matrix_04 = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

std::vector<std::vector<bool>> test_matrix_05 = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

std::vector<std::vector<bool>> test_matrix_06 = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};

    test_matrix_list.clear();
    test_matrix_list.push_back(test_matrix_01);
    test_matrix_list.push_back(test_matrix_02);
    test_matrix_list.push_back(test_matrix_03);
    test_matrix_list.push_back(test_matrix_04);
    test_matrix_list.push_back(test_matrix_05);
    test_matrix_list.push_back(test_matrix_06);

    int index = 0;

    std::vector<ros::Publisher> occupancyGridPub;
    occupancyGridPub.reserve(test_matrix_list.size());

    std::vector<ros::Publisher> pathPub;
    pathPub.reserve(test_matrix_list.size());

    std::vector<nav_msgs::Path> sub_paths_msg(test_matrix_list.size());
    std::vector<nav_msgs::OccupancyGrid> occupancyGridMsg(test_matrix_list.size());

for (std::vector<std::vector<bool>> test_matrix : test_matrix_list){

    std::string occupancyGridTopic = "/custom_occupancy_grid_" + std::to_string(index);
    std::string pathTopic = "/sub_region_path_" + std::to_string(index);

    occupancyGridPub.push_back(nh.advertise<nav_msgs::OccupancyGrid>(occupancyGridTopic, 10,true));
    pathPub.push_back(nh.advertise<nav_msgs::Path>(pathTopic, 10,true));
    

    nRows = test_matrix.size();
    nCols = test_matrix[0].size();

    narrow_area_grid_points.clear();
    narrow_area_points_vertical.clear();
    narrow_area_points_horizontal.clear();
    narrow_area_grid_points = std::vector<std::vector<bool>>(nRows, std::vector<bool>(nCols, true));

    int max_row = 0;
    int max_col = 0;

    current_col = nullptr;
    current_root.clear();
    temp_odd_root.clear();
    col_count = 0;


    test_area_visited_h.clear();
    test_area_visited_v.clear();
    test_area_graph_h.clear();
    test_area_graph_v.clear();

    test_area_visited_h = std::vector<std::vector<bool>>(nRows, std::vector<bool>(nCols, false));
    test_area_visited_v = std::vector<std::vector<bool>>(nRows, std::vector<bool>(nCols, false));
    test_area_graph_h = std::vector<std::vector<Node*>>(nRows, std::vector<Node*>(nCols, nullptr));
    test_area_graph_v = std::vector<std::vector<Node*>>(nRows, std::vector<Node*>(nCols, nullptr));

    root_h = nullptr;
    root_v = nullptr;

    for (int i = 0; i < nRows; ++i) {
        for (int j = 0; j < nCols; ++j) {
            if (test_matrix[i][j] == 0 && !test_area_visited_h[i][j]) {

                std::cout<<"Found pont h : "<<"("<<i<<", "<<j<<")"<<std::endl;
                bfs(i, j,nRows,nCols,test_matrix, test_area_visited_h,test_area_graph_h,root_h);
            }
        }
    }

    for (int i = 0; i < nRows; ++i) {
        for (int j = 0; j < nCols; ++j) {
            if (test_matrix[i][j] == 0 && !test_area_visited_v[i][j]) {
                std::cout<<"Found pont v : "<<"("<<i<<", "<<j<<")"<<std::endl;
                bfs_vertical(i, j,nRows,nCols,test_matrix,test_area_visited_v,test_area_graph_v,root_v);
            }
        }
    }



    if (root_h != nullptr) {
        int max_row_count = max_row - root_h->x;
        preOrderTraversalHorizontal(root_h,current_col,current_root,max_row_count,temp_odd_root,col_count,narrow_area_grid_points,narrow_area_points_horizontal);
    }
    
    else {
        std::cout << "No column root node found." << std::endl;
    }

    if (root_v != nullptr) {
        int max_row_count = max_row - root_v->x;

        current_col = nullptr;
        current_root.clear();
        temp_odd_root.clear();
        col_count = 0;
        preOrderTraversalVertical(root_v,current_col,current_root,max_row_count,temp_odd_root,col_count,narrow_area_grid_points,narrow_area_points_vertical);
    }
    
    else {
        std::cout << "No column root node found." << std::endl;
    }
    
    // std::cout<<std::endl;

    // std::cout<<"narrow_area_points_vertical size : "<<narrow_area_points_vertical.size()<<std::endl;
    // std::cout<<"narrow_area_points_horizontal size : "<<narrow_area_points_horizontal.size()<<std::endl;
    // std::cout<<std::endl;

    // std::cout<<"# Main matrix  #############################"<<std::endl;
    // std::cout<<std::endl;

    // printGridBinary(test_matrix);

    // std::cout<<std::endl;

    // std::cout<<"# Main matrix identified narrow points  #############################"<<std::endl;
    // std::cout<<std::endl;

    // printGridBinary(narrow_area_grid_points);


    std::list<Point_t> sub_region_path;

    Point_t sub_region_Scaled;
    int multiple_pass_counter, visited_counter;

    sub_region_Scaled.x = root_v->x;
    sub_region_Scaled.y = root_v->y;


    sub_region_path = full_coverage_path_planner::SpiralSTC::new_spiral_stc(test_matrix, sub_region_Scaled, multiple_pass_counter, visited_counter,narrow_area_grid_points,narrow_area_points_horizontal,narrow_area_points_vertical);

    convertToOccupancyGrid(test_matrix, occupancyGridMsg[index]);

    sub_paths_msg[index] = convertToPath(sub_region_path,occupancyGridMsg[index]);

    occupancyGridPub[index].publish(occupancyGridMsg[index]);
    pathPub[index].publish(sub_paths_msg[index]);

    // ros::Rate loop_rate(10); // 10 Hz
    // while (ros::ok()) {
    //     occupancyGridPub[index].publish(occupancyGridMsg[index]);
    //     pathPub[index].publish(sub_paths_msg[index]);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    index++;
}


    ros::spin();
    return EXIT_SUCCESS;
}