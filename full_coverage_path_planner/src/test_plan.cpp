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

void preOrderTraversalHorizontal(Node* node, std::vector<std::vector<Node*>>& narrow_area_points, 
                        std::vector<std::vector<bool>>& narrow_area_grid_points,
                        Node*& current_col,
                        std::vector<Node*>& current_root,
                        int max_row_count,
                        std::vector<Node*>& temp_odd_root,
                        int& col_count
                        ) {
    if (!node) return;
    
    preOrderTraversalHorizontal(node->left,narrow_area_points,narrow_area_grid_points,current_col,current_root,max_row_count,temp_odd_root,col_count);
    
    
    preOrderTraversalHorizontal(node->right,narrow_area_points,narrow_area_grid_points,current_col,current_root,max_row_count,temp_odd_root,col_count);

    if(current_col == nullptr){
        current_col = node;
    }

    if(current_col->x !=  node->x){
        // std::cout<<"new col"<<std::endl;
        // std::cout<<"col size : "<<current_root.size()<<std::endl;
        if(current_root.size() %2 != 0 && current_root.front()->y == node->y && current_root.size() <= 3 ){

            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            col_count++;
            current_root.clear();
            current_root.push_back(node);

        }
        else{
            // std::cout<<"Push back "<<std::endl;
            if(current_root.size()<=3){
                temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            }
            // std::cout<<"narrow point count : "<<temp_odd_root.size()<<std::endl;;

            if(col_count>=3){
                narrow_area_points.push_back(temp_odd_root);
                for (const auto& node_ptr : temp_odd_root) {
                    narrow_area_grid_points[node_ptr->x][node_ptr->y] = false;
                }
                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();
                current_root.push_back(node);
            }
            else{
                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();
                current_root.push_back(node);
            }
        }
    }
    else{
        current_root.push_back(node);
    }
    std::cout << "(" << node->x << ", " << node->y << ") ";
    current_col = node;
}


void preOrderTraversalVertical(Node* node, std::vector<std::vector<Node*>>& narrow_area_points, 
                        std::vector<std::vector<bool>>& narrow_area_grid_points,
                        Node*& current_col,
                        std::vector<Node*>& current_root,
                        int max_row_count,
                        std::vector<Node*>& temp_odd_root,
                        int& col_count
                        ) {
    if (!node) return;
    
    preOrderTraversalVertical(node->right,narrow_area_points,narrow_area_grid_points,current_col,current_root,max_row_count,temp_odd_root,col_count);
    
    
    preOrderTraversalVertical(node->left,narrow_area_points,narrow_area_grid_points,current_col,current_root,max_row_count,temp_odd_root,col_count);

    if(current_col == nullptr){
        current_col = node;
    }

    if(current_col->y !=  node->y){
        // std::cout<<"new col"<<std::endl;
        // std::cout<<"col size : "<<current_root.size()<<std::endl;
        if(current_root.size() %2 != 0 && current_root.front()->x == node->x && current_root.size() <= 3 ){

            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            col_count++;
            current_root.clear();
            current_root.push_back(node);

        }
        else{
            // std::cout<<"Push back "<<std::endl;
            if(current_root.size()<=3){
                temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            }

            // std::cout<<"narrow point count : "<<temp_odd_root.size()<<std::endl;;

            if(col_count>=3){
                narrow_area_points.push_back(temp_odd_root);
                for (const auto& node_ptr : temp_odd_root) {
                    narrow_area_grid_points[node_ptr->x][node_ptr->y] = false;
                }
                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();
                current_root.push_back(node);
            }
            else{
                temp_odd_root.clear();
                col_count = 0;
                current_root.clear();
                current_root.push_back(node);
            }
        }
    }
    else{
        current_root.push_back(node);
    }
    std::cout << "(" << node->x << ", " << node->y << ") ";
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


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_plan");
    ros::NodeHandle nh;

    ros::Publisher occupancyGridPub = nh.advertise<nav_msgs::OccupancyGrid>("/custom_occupancy_grid", 10);

    nav_msgs::OccupancyGrid occupancyGridMsg;

    full_coverage_path_planner::SpiralSTC planner;

    // create_test_occupancy_grid(occupancyGridMsg,occupancyGridPub);



    std::vector<std::vector<bool>> test_matrix_h = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };



    std::vector<std::vector<bool>> test_matrix_v = {
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 1, 1, 1, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };



    int nRows = test_matrix_h.size();
    int nCols = test_matrix_h[0].size();

    int max_row = 0;
    int max_col = 0;

    std::vector<std::vector<Node*>> narrow_area_points_vertical;
    std::vector<std::vector<Node*>> narrow_area_points_horizontal;
    std::vector<std::vector<bool>> narrow_area_grid_points_h(nRows, std::vector<bool>(nCols, true));
    std::vector<std::vector<bool>> narrow_area_grid_points_v(nRows, std::vector<bool>(nCols, true));

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
            if (test_matrix_h[i][j] == 0 && !test_area_visited_h[i][j]) {
                bfs(i, j,nRows,nCols,test_matrix_h, test_area_visited_h,test_area_graph_h,root_h);
            }
        }
    }

    for (int i = 0; i < nRows; ++i) {
        for (int j = 0; j < nCols; ++j) {
            if (test_matrix_v[i][j] == 0 && !test_area_visited_v[i][j]) {
                bfs_vertical(i, j,nRows,nCols,test_matrix_v, test_area_visited_v,test_area_graph_v,root_v);
            }
        }
    }


    std::cout<<"test graph size H : "<<test_area_graph_h.size()<<std::endl;
    preOrderTraversal_h(root_h);

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout<<"test graph size V : "<<test_area_graph_v.size()<<std::endl;
    preOrderTraversal_v(root_v);
    std::cout<<std::endl;
    std::cout<<std::endl;



    if (root_h != nullptr) {
        int max_row_count = max_row - root_h->x;
        preOrderTraversalHorizontal(root_h,narrow_area_points_horizontal,narrow_area_grid_points_h,current_col,current_root,max_row_count,temp_odd_root,col_count);
    }
    else {
        std::cout << "No column root node found." << std::endl;
    }
    std::cout<<std::endl;

    std::cout<<"narrow_area_points_horizontal size : "<<narrow_area_points_horizontal.size()<<std::endl;

    printGridBinary(test_matrix_h);

    std::cout<<std::endl;

    printGridBinary(narrow_area_grid_points_h);




    if (root_v != nullptr) {
        int max_row_count = max_row - root_v->x;

        current_col = nullptr;
        current_root.clear();
        temp_odd_root.clear();
        col_count = 0;
        preOrderTraversalVertical(root_v,narrow_area_points_vertical,narrow_area_grid_points_v,current_col,current_root,max_row_count,temp_odd_root,col_count);
    }
    else {
        std::cout << "No column root node found." << std::endl;
    }
    
    std::cout<<std::endl;

    std::cout<<"narrow_area_points_vertical size : "<<narrow_area_points_vertical.size()<<std::endl;

    printGridBinary(test_matrix_v);

    std::cout<<std::endl;

    printGridBinary(narrow_area_grid_points_v);


    // std::list<Point_t> sub_region_path;

    // Point_t sub_region_Scaled;
    // int multiple_pass_counter, visited_counter;

    // sub_region_Scaled.x = root_h->x;
    // sub_region_Scaled.y = root_h->y;

    // sub_region_path = full_coverage_path_planner::SpiralSTC::new_spiral_stc(test_matrix_h, sub_region_Scaled, multiple_pass_counter, visited_counter,narrow_area_grid_points);

    // std::cout<<std::endl;
    // std::cout<<std::endl;

    // for (const auto& point : sub_region_path) {
    //     std::cout<<"("<<point.x<<", "<<point.y<<")";
    // }


    // ros::spin();
    // return EXIT_SUCCESS;
}