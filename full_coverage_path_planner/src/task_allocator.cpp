#include "task_allocator.h"
#include <sstream>
#include <algorithm>
#include <ros/ros.h>
#include <random>
#include <map> 
#include <cmath>

namespace full_coverage_path_planner {

TaskAllocator::TaskAllocator(ros::NodeHandle& nh, const std::vector<std::string>& robotNamespaces) : nh_(nh) {
    assignments_pub_ = nh_.advertise<std_msgs::String>("assignments", 10);
    ROS_INFO("Task Allocator Node Initialized");

    // Initialize robot start positions
    initializeRobotStartPositions(robotNamespaces);

    for (int index = 0; index < robotNamespaces.size(); ++index) {
        std::string topicName = robotNamespaces[index] + "/waypoints";
        ROS_INFO("Subscribing to: %s", topicName.c_str());
        dynamicSubscribers_.push_back(nh_.subscribe<nav_msgs::Path>(
            topicName, 10, [this, topicName, index](const nav_msgs::Path::ConstPtr& msg) {
                this->dynamicCallback(msg, topicName, index);
        }));
    }
}

TaskAllocator::~TaskAllocator() {
    ROS_INFO("Task Allocator Node Shutting Down");
}

void TaskAllocator::dynamicCallback(const nav_msgs::Path::ConstPtr& msg, const std::string& topic, int robotId) {
    static int waypointId = 0;
    ROS_INFO("[DynamicCallback] Received waypoint on topic: %s from Robot %d", topic.c_str(), robotId);
    Task waypoint;
    waypoint.id = waypointId++;
    waypoint.path = *msg;
    tasks_.push_back(waypoint);
    calculateBidsBasedOnDistance(waypoint.id);
}

void TaskAllocator::initializeRobotStartPositions(const std::vector<std::string>& robotNamespaces) {
    robotStartLocations.resize(robotNamespaces.size());
    for (int index = 0; index < robotNamespaces.size(); ++index) {
        std::string paramName = robotNamespaces[index] + "/start_position";
        double x, y;
        if (nh_.getParam(paramName + "/x", x) && nh_.getParam(paramName + "/y", y)) {
            robotStartLocations[index].x = x;
            robotStartLocations[index].y = y;
            ROS_INFO("Robot %d start position: (%f, %f)", index, x, y);
        } else {
            ROS_WARN("Failed to get start position for robot %d", index);
        }
    }
}

void TaskAllocator::calculateBidsBasedOnDistance(int waypointId) {
    if (waypointId < 0 || waypointId >= tasks_.size()) {
        ROS_WARN("Invalid waypoint ID for bid calculation.");
        return;
    }
    auto& waypointStartPos = tasks_[waypointId].path.poses.front().pose.position;
    ROS_INFO("Waypoint %d start position: (%f, %f)", waypointId, waypointStartPos.x, waypointStartPos.y);
    
    for (size_t i = 0; i < robotStartLocations.size(); ++i) {
        auto dx = robotStartLocations[i].x - waypointStartPos.x;
        auto dy = robotStartLocations[i].y - waypointStartPos.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double bidValue = 1 / (distance + 0.01); // Avoid division by zero
        task_bids_[waypointId].emplace_back(waypointId, i, bidValue);
        ROS_INFO("[calculateBidsBasedOnDistance] Robot %zu bids %f for Waypoint %d (distance: %f)", i, bidValue, waypointId, distance);
    }

    bool allBidsReceived = true;

    // Check if all waypoints have bids from all robots
    for (const auto& waypoint : tasks_) {
        if (task_bids_[waypoint.id].size() < dynamicSubscribers_.size()) {
            allBidsReceived = false;
            break;
        }
    }

    if (allBidsReceived) {
        ROS_INFO("[calculateBidsBasedOnDistance] All bids received, allocating waypoints.");
        allocateTasks();
    }
}

void TaskAllocator::allocateTasks() {
    ROS_INFO("[allocateTasks] Allocating waypoints...");
    std::set<int> remainingTasks;
    for (const auto& waypoint : tasks_) {
        remainingTasks.insert(waypoint.id);
    }

    std::vector<int> remainingRobots(dynamicSubscribers_.size());
    std::iota(remainingRobots.begin(), remainingRobots.end(), 0); 

    std::map<int, int> taskAssignments; // Maps waypoint IDs to robot IDs

    while (!remainingTasks.empty() && !remainingRobots.empty()) {
        for (auto waypointId : remainingTasks) {
            double highestBidValue = -1;
            int highestBidRobotId = -1;
            for (const auto& bid : task_bids_[waypointId]) {
                if (std::find(remainingRobots.begin(), remainingRobots.end(), bid.robot_id) != remainingRobots.end() && 
                    (highestBidRobotId == -1 || bid.bid_value > highestBidValue)) {
                    highestBidValue = bid.bid_value;
                    highestBidRobotId = bid.robot_id;
                }
            }

            if (highestBidRobotId != -1) {
                // Assign this waypoint to the robot with the highest bid
                taskAssignments[waypointId] = highestBidRobotId;
                std_msgs::String msg;
                std::stringstream ss;
                ss << "Assign Waypoint " << waypointId << " to Robot " << highestBidRobotId;
                msg.data = ss.str();
                assignments_pub_.publish(msg);
                ROS_INFO("[allocateTasks] %s", ss.str().c_str());

                // Remove the assigned robot from the pool of available robots
                remainingRobots.erase(std::remove(remainingRobots.begin(), remainingRobots.end(), highestBidRobotId), remainingRobots.end());
            }
        }

        // Reevaluate remaining waypoints for next iteration
        std::set<int> newRemainingTasks;
        for (const auto& waypoint : tasks_) {
            if (taskAssignments.find(waypoint.id) == taskAssignments.end()) { // If this waypoint hasn't been assigned
                newRemainingTasks.insert(waypoint.id);
            }
        }
        remainingTasks.swap(newRemainingTasks);

        // If no waypoints were assigned in this iteration, break to avoid loop
        if (remainingTasks.empty()) {
            break;
        }
    }

    // Print out the waypoint assignments after all have been allocated
    ROS_INFO("[allocateTasks] Waypoint allocations completed. Summary:");
    for (const auto& assignment : taskAssignments) {
        ROS_INFO("Waypoint %d assigned to Robot %d", assignment.first, assignment.second);
    }
}

// Determine the winner based on the bids
int TaskAllocator::determineWinner(const std::vector<RobotBid>& bids, const std::set<int>& excludedRobots) {
    if (bids.empty()) {
        return -1; // No winner if there are no bids
    }
    for (const auto& bid : bids) {
        if (excludedRobots.find(bid.robot_id) == excludedRobots.end()) {
            ROS_INFO("[determineWinner] Winner for waypoint is Robot %d with bid %f", bid.robot_id, bid.bid_value);
            return bid.robot_id;
        }
    }
    return -1; // No valid winner found
}

} // namespace full_coverage_path_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_allocator");
    ros::NodeHandle nh;

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);
    std::vector<std::string> robotNamespaces;

    for (const auto& topic : master_topics) {
        const std::string& topic_name = topic.name;
        if (topic_name.find("/waypoints") != std::string::npos) {
            size_t pos = topic_name.find_last_of('/');
            if (pos != std::string::npos) {
                std::string robot_namespace = topic_name.substr(1, pos - 1);
                robotNamespaces.push_back(robot_namespace);
            }
        }
    }

    if (robotNamespaces.empty()) {
        ROS_WARN("No robot namespaces found. Exiting...");
        return EXIT_FAILURE;
    }

    full_coverage_path_planner::TaskAllocator taskAllocator(nh, robotNamespaces);
    ros::spin();
    return EXIT_SUCCESS;
}
