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
        if(current_root.size() %2 != 0 && current_root.size()<=max_row_count/2){

            temp_odd_root.insert(temp_odd_root.end(),current_root.begin(),current_root.end());
            col_count++;
            current_root.clear();
            current_root.push_back(node);

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


void preOrderTrave(Node* root){

    if(!root) return;

    std::cout << "(" << root->x << ", " << root->y << ") ";
    preOrderTrave(root->left);
    preOrderTrave(root->right);

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
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 0, 0, 0, 1, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

    int nRows = test_matrix.size();
    int nCols = test_matrix[0].size();

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

    Node* root_row = createBinaryTreeFromMatrix(test_matrix,max_row);
    // std::cout << "Pre-Order Traversal of Binary Tree Created from Matrix:" << std::endl;
    // std::cout<<"Max length of row : "<<max_row - root_row->y<<std::endl;

    // if (root_row != nullptr) {
    //     preOrderTraversal(root_row,narrow_area_points_vertical,narrow_area_grid_points);
    // }
    // else {
    //     std::cout << "No row root node found." << std::endl;
    // }
    

    // std::cout<<std::endl;
    // std::cout<<std::endl;

    // Node* root_col = createColumnWiseBinaryTreeFromMatrix(test_matrix,max_col);
    // std::cout << "Pre-Order Traversal of Column-wise Binary Tree Created from Matrix:" << std::endl;
    // std::cout<<"Max length of col : "<<max_col - root_col->x<<std::endl;

    // if (root_col != nullptr) {
    //     current_col = root_col;
    //     int max_col_count = max_col - root_col->x;
    //     preOrderTraversal(root_col,narrow_area_points_vertical,narrow_area_grid_points,current_col,current_root,max_col_count,temp_odd_root,col_count);
    // }
    // else {
    //     std::cout << "No column root node found." << std::endl;
    // }


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


    std::cout<<"test graph size : "<<test_area_graph.size()<<std::endl;

    if (root != nullptr) {
        int max_row_count = max_row - root->x;
        preOrderTraversalHorizontal(root,narrow_area_points_vertical,narrow_area_grid_points,current_col,current_root,max_row_count,temp_odd_root,col_count);
    }
    else {
        std::cout << "No column root node found." << std::endl;
    }

    preOrderTrave(root);

    // preOrderTraversal(root,narrow_area_points_horizontal,narrow_area_grid_points);

    
    // printGraph(test_area_graph_row,test_area_visited_row.size(),test_area_visited_row[0].size(),test_area_visited_row);

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

    // std::cout<<"Size of narrow_area_points  : "<<narrow_area_points_vertical.size()<<std::endl;

    // for (size_t i = 0; i < narrow_area_points_vertical.size(); ++i) {
    //     std::cout << "List " << i << ":" << std::endl;
    //     for (const auto node_ptr : narrow_area_points_vertical[i]) {
    //         std::cout << "(" << node_ptr->x << ", " << node_ptr->y << ") ";
    //     }
    //     std::cout << std::endl;
    // }

    // printGridBinary(narrow_area_grid_points);


    // ros::spin();
    // return EXIT_SUCCESS;
}