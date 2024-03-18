#include "task_allocator.h"
#include <sstream>
#include <algorithm>
#include <ros/ros.h>
#include <random>
#include <map> 

namespace full_coverage_path_planner {

TaskAllocator::TaskAllocator(ros::NodeHandle& nh, const std::vector<std::string>& robotNamespaces) : nh_(nh) {
    assignments_pub_ = nh_.advertise<std_msgs::String>("assignments", 10);
    ROS_INFO("Task Allocator Node Initialized");

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
    static int taskId = 0;
    ROS_INFO("[DynamicCallback] Received task on topic: %s from Robots", topic.c_str(), robotId);
    Task task;
    task.id = taskId++;
    task.path = *msg;
    tasks_.push_back(task);
    simulateBids(task.id);
}

void TaskAllocator::initializeRobotStartPositions(const std::vector<std::string>& robotNamespaces) {
    robotStartLocations.resize(robotNamespaces.size());
    for (int index = 0; index < robotNamespaces.size(); ++index) {
        std::string paramName = robotNamespaces[index] + "/start_position";
        double x, y;
        nh_.getParam(paramName + "/x", x);
        nh_.getParam(paramName + "/y", y);
        robotStartLocations[index].x = x;
        robotStartLocations[index].y = y;
    }
}

void TaskAllocator::calculateBidsBasedOnDistance(int taskId) {
    if (taskId < 0 || taskId >= tasks_.size()) {
        ROS_WARN("Invalid task ID for bid calculation.");
        return;
    }
    auto& taskStartPos = tasks_[taskId].path.poses.front().pose.position;
    for (size_t i = 0; i < robotStartLocations.size(); ++i) {
        auto dx = robotStartLocations[i].x - taskStartPos.x;
        auto dy = robotStartLocations[i].y - taskStartPos.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        double bidValue = 1 / (distance + 0.01); // Avoid division by zero
        task_bids_[taskId].emplace_back(taskId, i, bidValue);
    }
}

void TaskAllocator::simulateBids(int taskId) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count() + taskId;
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> distribution(1.0, 10.0);

    bool allBidsReceived = true;

    for (int robotId = 0; robotId < dynamicSubscribers_.size(); ++robotId) {
        double bidValue = distribution(generator);
        RobotBid bid(taskId, robotId, bidValue);
        task_bids_[taskId].push_back(bid);
        ROS_INFO("[simulateBids] Robot %d bids %f for Task %d", robotId, bidValue, taskId);
    }

    // Check if all tasks have bids from all robots
    for (const auto& task : tasks_) {
        if (task_bids_[task.id].size() < dynamicSubscribers_.size()) {
            allBidsReceived = false;
            break;
        }
    }

    if (allBidsReceived) {
        ROS_INFO("[simulateBids] All bids received, allocating tasks.");
        allocateTasks();
    }
}

void TaskAllocator::allocateTasks() {
    ROS_INFO("[allocateTasks] Allocating tasks...");
    std::set<int> remainingTasks;
    for (const auto& task : tasks_) {
        remainingTasks.insert(task.id);
    }

    std::vector<int> remainingRobots(dynamicSubscribers_.size());
    std::iota(remainingRobots.begin(), remainingRobots.end(), 0); 

    std::map<int, int> taskAssignments; // Maps task IDs to robot IDs

    while (!remainingTasks.empty() && !remainingRobots.empty()) {
        for (auto taskId : remainingTasks) {
            double highestBidValue = -1;
            int highestBidRobotId = -1;
            for (const auto& bid : task_bids_[taskId]) {
                if (std::find(remainingRobots.begin(), remainingRobots.end(), bid.robot_id) != remainingRobots.end() && 
                    (highestBidRobotId == -1 || bid.bid_value > highestBidValue)) {
                    highestBidValue = bid.bid_value;
                    highestBidRobotId = bid.robot_id;
                }
            }

            if (highestBidRobotId != -1) {
                // Assign this task to the robot with the highest bid
                taskAssignments[taskId] = highestBidRobotId;
                std_msgs::String msg;
                std::stringstream ss;
                ss << "Assign Task " << taskId << " to Robot " << highestBidRobotId;
                msg.data = ss.str();
                assignments_pub_.publish(msg);
                ROS_INFO("[allocateTasks] %s", ss.str().c_str());

                // Remove the assigned robot from the pool of available robots
                remainingRobots.erase(std::remove(remainingRobots.begin(), remainingRobots.end(), highestBidRobotId), remainingRobots.end());
            }
        }

        // Reevaluate remaining tasks for next iteration
        std::set<int> newRemainingTasks;
        for (const auto& task : tasks_) {
            if (taskAssignments.find(task.id) == taskAssignments.end()) { // If this task hasn't been assigned
                newRemainingTasks.insert(task.id);
            }
        }
        remainingTasks.swap(newRemainingTasks);

        // If no tasks were assigned in this iteration, break to avoid loop
        if (remainingTasks.empty()) {
            break;
        }
    }

    // Print out the task assignments after all have been allocated
    ROS_INFO("[allocateTasks] Task allocations completed. Summary:");
    for (const auto& assignment : taskAssignments) {
        ROS_INFO("Task %d assigned to Robot %d", assignment.first, assignment.second);
    }
}



// Determine the winner based on the bids
int TaskAllocator::determineWinner(const std::vector<RobotBid>& bids, const std::set<int>& excludedRobots) {
    if (bids.empty()) {
        return -1; // No winner if there are no bids
    }
    for (const auto& bid : bids) {
        if (excludedRobots.find(bid.robot_id) == excludedRobots.end()) {
            ROS_INFO("[determineWinner] Winner for task is Robot %d with bid %f", bid.robot_id, bid.bid_value);
            return bid.robot_id;
        }
    }
    return -1; // No valid winner found
}

} // namespace full_coverage_path_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "task_allocator");
    ros::NodeHandle nh;
    std::string robot_name;
    // ros::param::get("~robot_namespace", robot_name)

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
