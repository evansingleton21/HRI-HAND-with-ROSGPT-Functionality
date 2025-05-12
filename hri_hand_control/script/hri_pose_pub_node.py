#!/usr/bin/env python3
import rospy
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def load_poses(pose_definitions):
    with open(pose_definitions, 'r') as f:
        return yaml.safe_load(f)

def publish_pose(pub, joint_names, positions):
    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()
    js.name = joint_names
    js.position = positions
    pub.publish(js)


if __name__ == "__main__":
    #Node Initialization
    rospy.init_node('yaml_hand_pose_publisher')

    pose_definitions = "/home/evansingleton21/catkin_ws/src/rosgpt/config/poses.yaml"  # or hardcode path
    poses = load_poses(pose_definitions)
    pose_names = list(poses.keys())

    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    rospy.sleep(2.0)

    #Initial Pose for Initialization Test    
    pose_name = "open"
    if pose_name in poses:
        pose = poses[pose_name]
        joint_names = list(pose.keys())
        joint_positions = [pose[j] for j in joint_names]
        rospy.loginfo(f"Publishing initial pose: {pose_name}")
        publish_pose(pub, joint_names, joint_positions)
    else:
        rospy.logwarn(f"Pose '{pose_name}' not found in pose definitions.")


    while not rospy.is_shutdown():
        pose_name = input("Enter pose name to publish (or 'q' to quit): ")
        if pose_name == 'q':
            break
        if pose_name in poses:
            pose = poses[pose_name]
            joint_names = list(pose.keys())
            joint_positions = [pose[j] for j in joint_names]
            rospy.loginfo(f"Publishing pose: {pose_name}")
            publish_pose(pub, joint_names, joint_positions)
        else:
            rospy.logwarn("Invalid pose name.")
