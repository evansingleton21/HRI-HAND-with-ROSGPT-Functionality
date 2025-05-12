#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def wait_for_rosgpt_ready():
    rospy.loginfo("Waiting for ROSGPT to initialize...")
    try:
        rospy.wait_for_message("/gpt_output", String, timeout=5)
    except rospy.ROSException:
        rospy.logwarn("Timed out waiting for /gpt_output, continuing anyway.")

def main():
    rospy.init_node("rosgpt_input_scan_node")
    wait_for_rosgpt_ready()  # <- This ensures GPT node prints first
    pub = rospy.Publisher("/gpt_input", String, queue_size=10)

    rospy.loginfo("ROSGPT Input Node: Type commands for GPT interaction.")
    while not rospy.is_shutdown():
        user_input = input("Enter a pose or gesture: ")
        pub.publish(user_input)

if __name__ == "__main__":
    main()
