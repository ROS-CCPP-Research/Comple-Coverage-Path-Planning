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

void preOrderTraversalHorizontal(Node* node,
                        Node*& current_col,
                        std::vector<Node*>& current_root,
                        int max_row_count,
                        std::vector<Node*>& temp_odd_root,
                        int& col_count
                        ) {
    if (!node) return;
    
    preOrderTraversalHorizontal(node->left,current_col,current_root,max_row_count,temp_odd_root,col_count);
    
    
    preOrderTraversalHorizontal(node->right,current_col,current_root,max_row_count,temp_odd_root,col_count);

    if(current_col == nullptr){
        current_col = node;
    }

    if(current_col->x !=  node->x){
        // std::cout<<"new col"<<std::endl;
        // std::cout<<"col size : "<<current_root.size()<<std::endl;
        if(current_root.size() %2 != 0 && current_root.front()->y == node->y){

            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            col_count++;
            current_root.clear();

            if(narrow_area_grid_points[node->x][node->y] == 1){
                current_root.push_back(node);
            }
            

        }
        else{
            // std::cout<<"Push back "<<std::endl;
            if(!temp_odd_root.empty() && current_root.back()->y == temp_odd_root.back()->y){
                temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
                col_count++;
            }
            // std::cout<<"narrow point count : "<<temp_odd_root.size()<<std::endl;;

            if(col_count>=3){

                narrow_area_points_horizontal.push_back(temp_odd_root);

                for (const auto& node_ptr : temp_odd_root) {
                    narrow_area_grid_points[node_ptr->x][node_ptr->y] = false;
                }

                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();

                if(narrow_area_grid_points[node->x][node->y] == 1){
                    current_root.push_back(node);
                }
            }
            else{

                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();

                if(narrow_area_grid_points[node->x][node->y] == 1){
                    current_root.push_back(node);
                }
            }
        }
    }
    else{
        if(narrow_area_grid_points[node->x][node->y] == 1){
                current_root.push_back(node);
            }
    }
    // std::cout << "(" << node->x << ", " << node->y << ") ";
    current_col = node;
}


void preOrderTraversalVertical(Node* node,
                        Node*& current_col,
                        std::vector<Node*>& current_root,
                        int max_row_count,
                        std::vector<Node*>& temp_odd_root,
                        int& col_count
                        ) {
    if (!node) return;
    
    preOrderTraversalVertical(node->right,current_col,current_root,max_row_count,temp_odd_root,col_count);
    
    
    preOrderTraversalVertical(node->left,current_col,current_root,max_row_count,temp_odd_root,col_count);

    if(current_col == nullptr){
        current_col = node;
    }

    if(current_col->y !=  node->y){
        // std::cout<<"new col"<<std::endl;
        // std::cout<<"col size : "<<current_root.size()<<std::endl;
        if(current_root.size() %2 != 0 && current_root.front()->x == node->x){

            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            col_count++;
            current_root.clear();
            if(narrow_area_grid_points[node->x][node->y] == 1){
                current_root.push_back(node);
            }
            

        }
        else{
            // std::cout<<"Push back "<<std::endl;
            if(!temp_odd_root.empty() && current_root.back()->x == temp_odd_root.back()->x){
                temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
                col_count++;
            }

            // std::cout<<"narrow point count : "<<temp_odd_root.size()<<std::endl;;

            if(col_count>=3){
                narrow_area_points_vertical.push_back(temp_odd_root);
                for (const auto& node_ptr : temp_odd_root) {
                    narrow_area_grid_points[node_ptr->x][node_ptr->y] = false;
                }
                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();

                if(narrow_area_grid_points[node->x][node->y] == 1){
                    current_root.push_back(node);
                }
            }
            else{
                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();
                if(narrow_area_grid_points[node->x][node->y] == 1){
                    current_root.push_back(node);
                }
            }
        }
    }
    else{
        if(narrow_area_grid_points[node->x][node->y] == 1){
                    current_root.push_back(node);
           }
    }
    // std::cout << "(" << node->x << ", " << node->y << ") ";
    current_col = node;
}

void preOrderTraversal_h(Node* root){

    if(!root) return;

    preOrderTraversal_h(root->left);
    preOrderTraversal_h(root->right);
    std::cout << "(" << root->x << ", " << root->y << ") ";
    
}

void preOrderTraversal_v(Node* root){

    if(!root) return;

    preOrderTraversal_v(root->right);
    preOrderTraversal_v(root->left);
    std::cout << "(" << root->x << ", " << root->y << ") ";
    
}


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

    ros::Publisher occupancyGridPub = nh.advertise<nav_msgs::OccupancyGrid>("/custom_occupancy_grid", 10);
    ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("/sub_region_path", 10);

    nav_msgs::OccupancyGrid occupancyGridMsg;



// std::vector<std::vector<bool>> test_matrix = {
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 0, 0, 0, 0, 1, 1, 1, 1, 1},
//     {1, 0, 0, 0, 0, 1, 1, 1, 1, 1},
//     {1, 0, 0, 0, 0, 1, 1, 1, 1, 1},
//     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
// };

// std::vector<std::vector<bool>> test_matrix = {
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
// };

// std::vector<std::vector<bool>> test_matrix = {
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
// };



// std::vector<std::vector<bool>> test_matrix = {
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//         {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//         {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//         {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//         {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
//         {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
//         {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
//         {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
//         {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
//         {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
//     };

// std::vector<std::vector<bool>> test_matrix = {
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
//     {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1},
//     {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
// };

std::vector<std::vector<bool>> test_matrix = {
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



    nRows = test_matrix.size();
    nCols = test_matrix[0].size();

    narrow_area_grid_points = std::vector<std::vector<bool>>(nRows, std::vector<bool>(nCols, true));

    int max_row = 0;
    int max_col = 0;

    Node* current_col = nullptr;
    // Node* currentStart = nullptr;
    // int root_lenghts = 0;
    std::vector<Node*> current_root;
    std::vector<Node*> temp_odd_root;
    int col_count = 0;


    std::vector<std::vector<bool>> test_area_visited_h(nRows, std::vector<bool>(nCols, false));
    std::vector<std::vector<bool>> test_area_visited_v(nRows, std::vector<bool>(nCols, false));
    std::vector<std::vector<Node*>> test_area_graph_h(nRows, std::vector<Node*>(nCols, nullptr));
    std::vector<std::vector<Node*>> test_area_graph_v(nRows, std::vector<Node*>(nCols, nullptr));

    Node* root_h = nullptr;
    Node* root_v = nullptr;

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
        preOrderTraversalHorizontal(root_h,current_col,current_root,max_row_count,temp_odd_root,col_count);
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
        preOrderTraversalVertical(root_v,current_col,current_root,max_row_count,temp_odd_root,col_count);
    }
    
    else {
        std::cout << "No column root node found." << std::endl;
    }
    
    std::cout<<std::endl;

    std::cout<<"narrow_area_points_vertical size : "<<narrow_area_points_vertical.size()<<std::endl;
    std::cout<<"narrow_area_points_horizontal size : "<<narrow_area_points_horizontal.size()<<std::endl;
    std::cout<<std::endl;

    std::cout<<"# Main matrix  #############################"<<std::endl;
    std::cout<<std::endl;

    printGridBinary(test_matrix);

    std::cout<<std::endl;

    std::cout<<"# Main matrix identified narrow points  #############################"<<std::endl;
    std::cout<<std::endl;

    printGridBinary(narrow_area_grid_points);


    std::list<Point_t> sub_region_path;

    Point_t sub_region_Scaled;
    int multiple_pass_counter, visited_counter;

    sub_region_Scaled.x = root_v->x;
    sub_region_Scaled.y = root_v->y;


    sub_region_path = full_coverage_path_planner::SpiralSTC::new_spiral_stc(test_matrix, sub_region_Scaled, multiple_pass_counter, visited_counter,narrow_area_grid_points,narrow_area_points_horizontal,narrow_area_points_vertical);
    // sub_region_path = full_coverage_path_planner::SpiralSTC::spiral_stc(test_matrix, sub_region_Scaled, multiple_pass_counter, visited_counter);
    
    // std::cout<<std::endl;
    // std::cout<<std::endl;

    // for (const auto& point : sub_region_path) {
    //     std::cout<<"("<<point.x<<", "<<point.y<<")";
    // }

    convertToOccupancyGrid(test_matrix, occupancyGridMsg);
    // occupancyGridPub.publish(occupancyGridMsg);

    nav_msgs::Path pathMsg = convertToPath(sub_region_path,occupancyGridMsg);
    // pathPub.publish(pathMsg);

    ros::Rate loop_rate(10); // 10 Hz
    while (ros::ok()) {
        occupancyGridPub.publish(occupancyGridMsg);
        pathPub.publish(pathMsg);
        ros::spinOnce();
        loop_rate.sleep();
    }


    ros::spin();
    return EXIT_SUCCESS;
}