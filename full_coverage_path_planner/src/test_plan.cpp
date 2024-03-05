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


void create_test_occupancy_grid(nav_msgs::OccupancyGrid& occupancyGridMsg,ros::Publisher occupancyGridPub){

    // Set the header
    occupancyGridMsg.header = std_msgs::Header();
    occupancyGridMsg.header.stamp = ros::Time::now();
    occupancyGridMsg.header.frame_id = "map_frame"; // Change frame_id if needed

    // Set the metadata
    occupancyGridMsg.info.resolution = 1.0;
    occupancyGridMsg.info.width = 10; // Change width if needed
    occupancyGridMsg.info.height = 10; // Change height if needed
    occupancyGridMsg.info.origin.position.x = -5.0;
    occupancyGridMsg.info.origin.position.y = -5.0;
    occupancyGridMsg.info.origin.position.z = 0.0;
    occupancyGridMsg.info.origin.orientation.x = 0.0;
    occupancyGridMsg.info.origin.orientation.y = 0.0;
    occupancyGridMsg.info.origin.orientation.z = 0.0;
    occupancyGridMsg.info.origin.orientation.w = 1.0;

    // Set occupancy grid data
    std::vector<int8_t> occupancyGridData(100, 0); // Initialize with all cells free

    // Set cells to occupied
    std::vector<int> indicesToSetTo100 = {53, 63, 73, 83, 93};
    for (int i : indicesToSetTo100) {
        occupancyGridData[i] = 100;
    }

    // Set more cells to occupied
    std::vector<int> indicesToSet2To100 = {6, 16, 26, 7, 17, 27};
    for (int i : indicesToSet2To100) {
        occupancyGridData[i] = 100;
    }

    for (int i = 65; i < 70; ++i) {
        occupancyGridData[i] = 100;
    }

    // Assign occupancy grid data to message
    occupancyGridMsg.data = occupancyGridData;

    // Publish the occupancy grid message
    while (ros::ok()) {
        occupancyGridPub.publish(occupancyGridMsg);
        ros::Duration(0.5).sleep(); // Adjust publishing rate as needed
    }

}

void preOrder(Node* root, int depth, Node*& currentStart, std::vector<Node*>& current_root, 
                std::vector<std::vector<Node*>>& narrow_area_points, std::vector<std::vector<bool>>& narrow_area_grid_points,
                std::vector<Node*>& temp_root) {
    if (root == nullptr) {
        return;
    }

    for (int i = 0; i < depth; ++i) {
        std::cout << "  ";
    }

    preOrder(root->left, depth + 1, currentStart, current_root, narrow_area_points, narrow_area_grid_points, temp_root);
    preOrder(root->right, depth + 1, currentStart, current_root, narrow_area_points, narrow_area_grid_points, temp_root);

    if (currentStart == nullptr) {
        currentStart = root;
    }

    if (root->left != nullptr) {
        current_root.push_back(root);

        if (current_root.size() % 2 == 1) {
            narrow_area_points.push_back(current_root);

            for (const auto& node_ptr : current_root) {
                narrow_area_grid_points[node_ptr->x][node_ptr->y] = false;
            }
        }
    } else if (root != nullptr) {
        current_root.push_back(root);
    }

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_plan");
    ros::NodeHandle nh;

    ros::Publisher occupancyGridPub = nh.advertise<nav_msgs::OccupancyGrid>("/custom_occupancy_grid", 10);

    nav_msgs::OccupancyGrid occupancyGridMsg;

    // create_test_occupancy_grid(occupancyGridMsg,occupancyGridPub);



    std::vector<std::vector<bool>> test_matrix = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };


    int nRows = test_matrix.size();
    int nCols = test_matrix[0].size();

    std::vector<std::vector<bool>> test_area_visited(nRows, std::vector<bool>(nCols, false));
    std::vector<std::vector<Node*>> test_area_graph(nRows, std::vector<Node*>(nCols, nullptr));
    Node* root = nullptr;

    for (int i = 0; i < nRows; ++i) {
        for (int j = 0; j < nCols; ++j) {
            if (test_matrix[i][j] == 0 && !test_area_visited[i][j]) {
                bfs(i, j,nRows,nCols,test_matrix, test_area_visited,test_area_graph,root);
            }
        }
    }

    printGraph(test_area_graph,test_area_visited.size(),test_area_visited[0].size(),test_area_visited);

    // ----------------------------------------------------------------------------------------------------------------------
    

    // Node* currentStart = nullptr;
    // int root_lenghts = 0;
    // std::vector<Node*> current_root;
    // std::vector<Node*> temp_root;
    // std::vector<std::vector<Node*>> narrow_area_points;
    // std::vector<std::vector<bool>> narrow_area_grid_points(nRows, std::vector<bool>(nCols, true));

    // if (root != nullptr) {
    //     std::cout << "Tree Structure:" << std::endl;
    //     std::cout<<"root right : "<<root->right<<std::endl;
    //     std::cout<<"root left : "<<root->left<<std::endl;
    //     preOrder(root,0,currentStart,current_root,narrow_area_points,narrow_area_grid_points,temp_root);
    // } else {
    //     std::cout << "No root node found." << std::endl;
    // }

    // std::cout<<"Size of narrow_area_points  : "<<narrow_area_points.size()<<std::endl;

    // for (size_t i = 0; i < narrow_area_points.size(); ++i) {
    //     std::cout << "List " << i << ":" << std::endl;
    //     for (const auto node_ptr : narrow_area_points[i]) {
    //         std::cout << "(" << node_ptr->x << ", " << node_ptr->y << ") ";
    //     }
    //     std::cout << std::endl;
    // }

    // printGridBinary(narrow_area_grid_points);


    ros::spin();
    return EXIT_SUCCESS;
}