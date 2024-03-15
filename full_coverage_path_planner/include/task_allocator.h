#ifndef TASK_ALLOCATOR_H
#define TASK_ALLOCATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <map>
#include <set>

namespace full_coverage_path_planner {

struct Task {
    int id;
    nav_msgs::Path path;
};

struct TaskAssignment {
    int task_id;
    int robot_id;
    TaskAssignment(int task_id, int robot_id) : task_id(task_id), robot_id(robot_id) {}
};

struct RobotBid {
    int task_id;
    int robot_id;
    double bid_value;
    RobotBid(int task_id, int robot_id, double bid_value) : task_id(task_id), robot_id(robot_id), bid_value(bid_value) {}
};

class TaskAllocator {
public:
    explicit TaskAllocator(ros::NodeHandle& nh, const std::vector<std::string>& robotNamespaces);
    ~TaskAllocator();

private:
    ros::NodeHandle nh_;
    ros::Publisher assignments_pub_;
    std::vector<ros::Subscriber> dynamicSubscribers_;
    std::vector<Task> tasks_;
    std::map<int, std::vector<RobotBid>> task_bids_; // Maps task IDs to bids
    std::set<int> assignedRobots; // Keeps track of robots that have been assigned tasks

    void dynamicCallback(const nav_msgs::Path::ConstPtr& msg, const std::string& topic, int robotId);
    void simulateBids(int taskId); // Adjusted to not directly use robotId
    void allocateTasks();
    int determineWinner(const std::vector<RobotBid>& bids, const std::set<int>& excludedRobots);
};

} // namespace full_coverage_path_planner

#endif // TASK_ALLOCATOR_H
