#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import random

def obstacle_publisher():
    rospy.init_node('obstacle_publisher', anonymous=True)
    pub = rospy.Publisher('/obstacles', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Simulate distances for 16 angles (covering 360°)
        obstacle_data = Float64MultiArray()
        obstacle_data.data = [random.uniform(0.5, 5.0) for _ in range(16)]  # Random distances
        pub.publish(obstacle_data)
        rospy.loginfo(f"Published obstacle data: {obstacle_data.data}")
        rate.sleep()

if __name__ == "__main__":
    try:
        obstacle_publisher()
    except rospy.ROSInterruptException:
        pass

