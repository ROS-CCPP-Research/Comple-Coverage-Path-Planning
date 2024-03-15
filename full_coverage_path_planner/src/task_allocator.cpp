#include "task_allocator.h"
#include <sstream>
#include <algorithm>
#include <ros/ros.h>
#include <random>

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

// Callback for dynamic topic subscription
void TaskAllocator::dynamicCallback(const nav_msgs::Path::ConstPtr& msg, const std::string& topic, int robotId) {
    static int taskId = 0;
    ROS_INFO("[DynamicCallback] Received task on topic: %s from Robot %d", topic.c_str(), robotId);
    Task task;
    task.id = taskId++;
    task.path = *msg;
    tasks_.push_back(task);

    // Adjusted: Simulate bids immediately for the new task from all robots, including the one that received the task
    simulateBids(task.id);
}

// Adjusted: Simulate bids for a specific task from all robots, not just the one receiving the task
void TaskAllocator::simulateBids(int taskId) {
    std::default_random_engine generator(taskId); // Use task ID as seed for reproducibility
    std::uniform_real_distribution<double> distribution(1.0, 10.0);

    // Generate bids from all robots for the new task
    for (int robotId = 0; robotId < dynamicSubscribers_.size(); ++robotId) {
        double bidValue = distribution(generator); // Generate a bid value
        RobotBid bid(taskId, robotId, bidValue);
        task_bids_[taskId].push_back(bid);
        ROS_INFO("[simulateBids] Robot %d bids %f for Task %d", robotId, bidValue, taskId);
    }

    // Allocate tasks only after all robots have bid for all tasks
    if (tasks_.size() * dynamicSubscribers_.size() == task_bids_.size()) {
        allocateTasks();
    }
}

// Allocate tasks ensuring one task per robot
void TaskAllocator::allocateTasks() {
    ROS_INFO("[allocateTasks] Allocating tasks...");
    std::set<int> remainingTasks;
    for (const auto& task : tasks_) {
        remainingTasks.insert(task.id);
    }

    std::set<int> remainingRobots;
    for (int robotId = 0; robotId < dynamicSubscribers_.size(); ++robotId) {
        remainingRobots.insert(robotId);
    }

    // Assign each task to a unique robot
    for (auto taskId : remainingTasks) {
        if (remainingRobots.empty()) {
            ROS_WARN("[allocateTasks] No available robots to assign tasks.");
            break;
        }
        auto& bids = task_bids_[taskId];
        int winner = determineWinner(bids, assignedRobots); // Pass the set of assigned robots as the second argument
        if (winner >= 0 && remainingRobots.find(winner) != remainingRobots.end()) {
            assignedRobots.insert(winner);
            std_msgs::String msg;
            std::stringstream ss;
            ss << "Assign Task " << taskId << " to Robot " << winner;
            msg.data = ss.str();
            assignments_pub_.publish(msg);
            ROS_INFO("[allocateTasks] %s", ss.str().c_str());
            remainingRobots.erase(winner);
        } else {
            ROS_WARN("[allocateTasks] No valid bid or robot already assigned for Task %d", taskId);
        }
    }

    // Reset for the next round of task allocations
    tasks_.clear();
    task_bids_.clear();
    assignedRobots.clear();
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
