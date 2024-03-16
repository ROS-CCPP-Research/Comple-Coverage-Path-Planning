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



void addRowNodes(const std::vector<std::vector<bool>>& matrix, int row, Node*& rowStart, Node*& prevRowStart, int& max_row) {
    Node* current = nullptr;
    for (int col = 0; col < matrix[row].size(); ++col) {
        if (matrix[row][col] == 0) {

            if(max_row<col) max_row = col;

            if (!current) {

                current = new Node(row, col);

                if (!rowStart) rowStart = current; // Mark start of the row
                
                if (prevRowStart) {
                    prevRowStart->right = current; // Link from previous row start to current start
                    prevRowStart = nullptr; // Disconnect prevRowStart as it's now linked
                }
            } else {
                current->left = new Node(row, col); // Link to the next node in the row
                current = current->left;
            }
        }
    }
}

Node* createBinaryTreeFromMatrix(const std::vector<std::vector<bool>>& matrix, int& max_row) {
    Node* root = nullptr;
    Node* prevRowStart = nullptr;
    for (int row = 0; row < matrix.size(); ++row) {
        Node* rowStart = nullptr;
        addRowNodes(matrix, row, rowStart, prevRowStart,max_row);
        if (!root) root = rowStart; // First non-null rowStart is the root
        prevRowStart = rowStart;
    }
    return root;
}

void addColumnNodes(const std::vector<std::vector<bool>>& matrix, int col, Node*& colStart, Node*& prevColStart, int& max_col) {
    Node* current = nullptr;
    for (int row = 0; row < matrix.size(); ++row) {
        if (matrix[row][col] == 0) { // Free area found
            if(max_col<row) max_col=row;
            if (!current) {
                current = new Node(row, col);
                if (!colStart) colStart = current; // Mark start of the column
                
                if (prevColStart) {
                    prevColStart->right = current; // Link from previous column start to current start
                    while (prevColStart->left) { // Find the next linkable node in the previous column
                        prevColStart = prevColStart->left;
                    }
                    prevColStart = nullptr; // Reset for next column
                }
            } else {
                current->left = new Node(row, col); // Link to the next node in the column
                current = current->left;
            }
        }
    }
}

Node* createColumnWiseBinaryTreeFromMatrix(const std::vector<std::vector<bool>>& matrix, int& max_col) {
    Node* root = nullptr;
    Node* prevColStart = nullptr;
    for (int col = 0; col < matrix[0].size(); ++col) {
        Node* colStart = nullptr;
        addColumnNodes(matrix, col, colStart, prevColStart,max_col);
        if (!root) root = colStart; // First non-null colStart is the root
        if (!prevColStart) prevColStart = colStart; // Keep track of the start of the previous column
    }
    return root;
}

void preOrderTraversal(Node* node, std::vector<std::vector<Node*>>& narrow_area_points, 
                        std::vector<std::vector<bool>>& narrow_area_grid_points,
                        Node*& current_col,
                        std::vector<Node*>& current_root,
                        int max_col_count,
                        std::vector<Node*>& temp_odd_root,
                        int& col_count
                        ) {
    if (!node) return;

    if(current_col->y !=  node->y && current_col->x+1 != node->x){
        std::cout<<"new col"<<std::endl;
        std::cout<<"col size : "<<current_root.size()+1<<std::endl;
        if(current_root.size()+1 %2 != 0 && current_root.size()<max_col_count/2){
            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            col_count++;
            current_root.clear();
        }
        else{
            std::cout<<temp_odd_root.size();
            if(col_count>=3){
                narrow_area_points.push_back(temp_odd_root);
                for (const auto& node_ptr : temp_odd_root) {
                    narrow_area_grid_points[node_ptr->x][node_ptr->y] = false;
                }
                temp_odd_root.clear();
                col_count = 0;
            }
            else{
                temp_odd_root.clear();
                col_count = 0;
            }
        }
    }
    else{
        current_root.push_back(node);
    }

    std::cout << "(" << node->x << ", " << node->y << ") ";
    current_col = node;
    
    preOrderTraversal(node->left,narrow_area_points,narrow_area_grid_points,current_col,current_root,max_col_count,temp_odd_root,col_count);
    
    
    preOrderTraversal(node->right,narrow_area_points,narrow_area_grid_points,current_col,current_root,max_col_count,temp_odd_root,col_count);
}

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
        std::cout<<"new col"<<std::endl;
        std::cout<<"col size : "<<current_root.size()<<std::endl;
        if(current_root.size() %2 != 0 && current_root.front()->y == node->y && current_root.size() <= 3 ){

            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            col_count++;
            current_root.clear();
            current_root.push_back(node);

        }
        else{
            std::cout<<"Push back "<<std::endl;
            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            std::cout<<"narrow point count : "<<temp_odd_root.size()<<std::endl;;

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
    std::vector<std::vector<bool>> narrow_area_grid_points(nRows, std::vector<bool>(nCols, true));

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
        preOrderTraversalHorizontal(root_h,narrow_area_points_horizontal,narrow_area_grid_points,current_col,current_root,max_row_count,temp_odd_root,col_count);
    }
    else {
        std::cout << "No column root node found." << std::endl;
    }
    std::cout<<std::endl;

    std::cout<<"narrow_area_points_horizontal size : "<<narrow_area_points_horizontal.size()<<std::endl;

    printGridBinary(narrow_area_grid_points);

    std::list<Point_t> sub_region_path;

    Point_t sub_region_Scaled;
    int multiple_pass_counter, visited_counter;

    sub_region_Scaled.x = root_h->x;
    sub_region_Scaled.y = root_h->y;

    sub_region_path = full_coverage_path_planner::SpiralSTC::new_spiral_stc(test_matrix_h, sub_region_Scaled, multiple_pass_counter, visited_counter,narrow_area_grid_points);

    std::cout<<std::endl;
    std::cout<<std::endl;

    for (const auto& point : sub_region_path) {
        std::cout<<"("<<point.x<<", "<<point.y<<")";
    }


    // ros::spin();
    // return EXIT_SUCCESS;
}