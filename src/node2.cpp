#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <vector>

// Global variables for positions
turtlesim::Pose::ConstPtr turtle1_pose = nullptr;
turtlesim::Pose::ConstPtr turtle2_pose = nullptr;

// Thresholds
const float DISTANCE_THRESHOLD = 1.0;
const float BOUNDARY_LIMIT = 1.0;
const float MAX_BOUNDARY = 10.0;

// Callback for turtle1 pose updates
void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = msg;
}

// Callback for turtle2 pose updates
void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = msg;
}

// Calculate the distance between turtle1 and turtle2
float calculateDistance() {
    if (turtle1_pose && turtle2_pose) {
        float x1 = turtle1_pose->x, y1 = turtle1_pose->y;
        float x2 = turtle2_pose->x, y2 = turtle2_pose->y;
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
    }
    return -1.0; // Invalid distance
}

// Stop the turtle
void stopTurtle(const std::string& turtle_name) {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/" + turtle_name + "/cmd_vel", 10);
    geometry_msgs::Twist stop_msg;
    pub.publish(stop_msg);
    ROS_INFO_STREAM(turtle_name << " has been stopped.");
}

// Wait for turtle2 to be available
void waitForTurtle2() {
    ROS_INFO("Waiting for turtle2 to be spawned...");
    ros::Rate rate(1); // 1 Hz
    ros::master::V_TopicInfo topics;
    while (ros::ok()) {
        ros::master::getTopics(topics);
        for (const auto& topic : topics) {
            if (topic.name == "/turtle2/pose") {
                ROS_INFO("turtle2 is available.");
                return;
            }
        }
        rate.sleep();
    }
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "node2");
    ros::NodeHandle nh;
    ROS_INFO("Node 2 (Distance Checker) Started");

    waitForTurtle2();

    ros::Subscriber turtle1_sub = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber turtle2_sub = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);

    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        if (turtle1_pose && turtle2_pose) {
            float distance = calculateDistance();
            if (distance >= 0) {
                std_msgs::Float32 distance_msg;
                distance_msg.data = distance;
                distance_pub.publish(distance_msg);
                ROS_INFO_STREAM("Distance: " << distance);

                // Stop turtles if too close
                if (distance < DISTANCE_THRESHOLD) {
                    ROS_WARN("Turtles too close! Stopping.");
                    stopTurtle("turtle1");
                    stopTurtle("turtle2");
                }

                // Check boundaries for turtle1
                if (turtle1_pose->x < BOUNDARY_LIMIT || turtle1_pose->x > MAX_BOUNDARY ||
                    turtle1_pose->y < BOUNDARY_LIMIT || turtle1_pose->y > MAX_BOUNDARY) {
                    ROS_WARN("Turtle1 near boundary! Stopping.");
                    stopTurtle("turtle1");
                }

                // Check boundaries for turtle2
                if (turtle2_pose->x < BOUNDARY_LIMIT || turtle2_pose->x > MAX_BOUNDARY ||
                    turtle2_pose->y < BOUNDARY_LIMIT || turtle2_pose->y > MAX_BOUNDARY) {
                    ROS_WARN("Turtle2 near boundary! Stopping.");
                    stopTurtle("turtle2");
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

