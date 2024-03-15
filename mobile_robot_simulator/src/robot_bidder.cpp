#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"

class RobotBidder {
public:
    RobotBidder(ros::NodeHandle& nh, const std::string& robot_id)
    : nh_(nh), robot_id_(robot_id) {
        std::string odom_topic = "/" + robot_id + "/odom";
        odom_sub_ = nh_.subscribe(odom_topic, 10, &RobotBidder::odomCallback, this);
        
        std::string bid_topic = "/" + robot_id + "/bid_value";
        bid_pub_ = nh_.advertise<std_msgs::Float64>(bid_topic, 10);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Extract robot position and compute bid value (e.g., distance from a goal)
        double bid_value = computeBid(msg->pose.pose.position);
        
        // Publish the bid value
        std_msgs::Float64 bid;
        bid.data = bid_value;
        bid_pub_.publish(bid);

        // Log the bid value
        ROS_INFO("Robot %s bid value: %f", robot_id_.c_str(), bid_value);
    }

private:
    double computeBid(const geometry_msgs::Point& position) {
        double task_x = 10.0; // Hypothetical task position x
        double task_y = 10.0; // Hypothetical task position y
        return sqrt(pow(position.x - task_x, 2) + pow(position.y - task_y, 2));
    }

    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher bid_pub_;
    std::string robot_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_bidder");
    
    if (argc != 2) {
        return -1;
    }
    std::string robot_id(argv[1]);
    
    ros::NodeHandle nh;
    RobotBidder bidder(nh, robot_id);

    ros::spin();
    return 0;
}
