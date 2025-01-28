#!/usr/bin/env python3
# Import necessary libraries
import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time

# Global variable for obstacle data
obstacle_data = []

# Function to handle obstacle callback
def obstacle_callback(msg):
    global obstacle_data
    obstacle_data = msg.data  # Array of obstacle distances (16 elements)

# Function to spawn turtle2 at a specific position
def spawn_turtle2():
    rospy.wait_for_service('/spawn')  # Wait for the spawn service
    try:
        spawn = rospy.ServiceProxy('/spawn', Spawn)
        spawn(5.0, 5.0, 0.0, 'turtle2')  # Spawn at x= 5, y= 5 , theta= 0
        rospy.loginfo("Turtle2 spawned successfully.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to spawn turtle2: {e}")  # If service call fails, log an error message

# Function to send velocity commands to the turtle
def send_velocity(turtle_name, linear_vel, angular_vel):
    pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = linear_vel
    vel_msg.angular.z = angular_vel

    start_time = time.time()
    rate = rospy.Rate(10)  # Set the publishing rate to 10 Hz
    while time.time() - start_time < 1:  # Send velocity command for 1 second
        pub.publish(vel_msg)
        rate.sleep()

    # Stop the turtle after 1 second
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)

# Main function to run the interface
def main():
    rospy.init_node('node1', anonymous=True)
    rospy.Subscriber('/obstacles', Float64MultiArray, obstacle_callback)

    while True:
        print("\n--- Choose an option ---")
        print("1. Spawn turtle2")
        print("2. Send velocity command")
        print("3. Check obstacle distances")
        print("4. Exit")

        choice = input("Enter your choice: ")

        if choice == '1':
            spawn_turtle2()  # Spawn turtle2
        elif choice == '2':
            turtle_name = input("Enter turtle name (turtle1/turtle2): ")
            try:
                linear_vel = float(input("Enter linear velocity: "))
                angular_vel = float(input("Enter angular velocity: "))
                send_velocity(turtle_name, linear_vel, angular_vel)  # Send velocity command
            except ValueError:
                print("Invalid input! Please enter numerical values.")
        elif choice == '3':
            if obstacle_data:
                print("Obstacle distances:", obstacle_data)
            else:
                print("No obstacle data available yet.")
        elif choice == '4':
            rospy.loginfo("Exiting the program.")  # Option to exit the program
            break
        else:
            print("Invalid choice! Try again.")

if __name__ == "__main__":
    main()


