#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

// Function to spawn turtle2 at a specific position
void spawnTurtle2() {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawnSrv;
    spawnSrv.request.x = 5.0;
    spawnSrv.request.y = 5.0;
    spawnSrv.request.theta = 0.0;
    spawnSrv.request.name = "turtle2";

    if (client.call(spawnSrv)) {
        ROS_INFO("Turtle2 spawned successfully.");
    } else {
        ROS_ERROR("Failed to spawn turtle2.");
    }
}

// Function to send velocity commands to the turtle
void sendVelocity(const std::string& turtleName, double linearVel, double angularVel) {
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/" + turtleName + "/cmd_vel", 10);

    geometry_msgs::Twist velMsg;
    velMsg.linear.x = linearVel;
    velMsg.angular.z = angularVel;

    ros::Rate rate(10); // 10 Hz publishing rate
    auto startTime = std::chrono::steady_clock::now();

    // Publish velocity commands for 1 second
    while (ros::ok() && std::chrono::steady_clock::now() - startTime < std::chrono::seconds(1)) {
        pub.publish(velMsg);
        rate.sleep();
    }

    // Stop the turtle
    velMsg.linear.x = 0;
    velMsg.angular.z = 0;
    pub.publish(velMsg);
}

// Main function to run the interface
int main(int argc, char** argv) {
    ros::init(argc, argv, "node1");
    ros::NodeHandle nh;

    while (ros::ok()) {
        std::cout << "\n--- Choose an option ---" << std::endl;
        std::cout << "1. Spawn turtle2" << std::endl;
        std::cout << "2. Send velocity command" << std::endl;
        std::cout << "3. Exit" << std::endl;

        std::cout << "Enter your choice: ";
        int choice;
        std::cin >> choice;

        if (choice == 1) {
            spawnTurtle2(); // Spawn turtle2
        } else if (choice == 2) {
            std::cout << "Enter turtle name (turtle1/turtle2): ";
            std::string turtleName;
            std::cin >> turtleName;

            double linearVel, angularVel;
            std::cout << "Enter linear velocity: ";
            std::cin >> linearVel;
            std::cout << "Enter angular velocity: ";
            std::cin >> angularVel;

            sendVelocity(turtleName, linearVel, angularVel); // Send velocity command
        } else if (choice == 3) {
            ROS_INFO("Exiting the program."); // Exit the program
            break;
        } else {
            std::cout << "Invalid choice! Try again." << std::endl;
        }
    }

    return 0;
}


