#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
from hri_hand_control.srv import HandCommand, HandCommandResponse

class HandService:
    def __init__(self):
        rospy.init_node('hand_command_service')

        # Publishers for each finger joint
        self.pubs = {
            "pris_thumb_jo_joint": rospy.Publisher('/pris_thumb_jo_position_controller/command', Float64MultiArray, queue_size=10),

            "pris_thumb_DIP_joint": rospy.Publisher('/pris_thumb_DIP_position_controller/command', Float64MultiArray, queue_size=10),

            "pris_index_joint": rospy.Publisher('/pris_index_position_controller/command', Float64MultiArray, queue_size=10),

            "pris_middle_joint": rospy.Publisher('/pris_middle_position_controller/command', Float64MultiArray, queue_size=10),

            "pris_ring_joint": rospy.Publisher('/pris_ring_position_controller/command', Float64MultiArray, queue_size=10),

            "pris_little_joint": rospy.Publisher('/pris_little_position_controller/command', Float64MultiArray, queue_size=10),

        }

        rospy.Service('/hand_command', HandCommand, self.handle_command)
        rospy.loginfo("Hand command service ready.")
        rospy.spin()

    def handle_command(self, req):
        cmd = req.command.lower()

        try:
            if cmd == "full_close":
                self.publish_all(0.01)
            elif cmd == "full_open":
                self.publish_all(0.0)
            elif cmd == "tri_close":
                self.publish_fingers(["pris_thumb_jo_joint", "pris_thumb_DIP_joint", "pris_index_joint", "pris_middle_joint"], 0.01)
            elif cmd == "tri_open":
                self.publish_fingers(["pris_thumb_jo_joint", "pris_thumb_DIP_joint", "pris_index_joint", "pris_middle_joint"], 0.0)
            elif cmd == "pinch":
                self.publish_fingers(["pris_thumb_jo_joint", "pris_thumb_DIP_joint", "pris_index_joint"], 0.01)
            elif cmd == "pinch_open":
                self.publish_fingers(["pris_thumb_jo_joint", "pris_thumb_DIP_joint", "pris_index_joint"], 0.0)
            elif cmd == "point_gesture":
                self.publish_fingers(["pris_thumb_jo_joint", "pris_thumb_DIP_joint", "pris_middle_joint", "pris_ring_joint", "pris_little_joint"], 0.01)
            elif cmd == "peace_gesture":
                self.publish_fingers(["pris_thumb_jo_joint", "pris_thumb_DIP_joint", "pris_ring_joint", "pris_little_joint"], 0.01)
            elif cmd == "thumbs_up":
                self.publish_fingers(["pris_index_joint", "pris_middle_joint", "pris_ring_joint", "pris_little_joint"], 0.01)

            else:
                return HandCommandResponse(False, "Unknown command")

            return HandCommandResponse(True, "Command executed")
        except Exception as e:
            return HandCommandResponse(False, str(e))

    def publish_scaled(self, controller, base_value, num_joints):
        msg = Float64MultiArray()
        msg.data = [base_value * (100 ** i) for i in range(num_joints)]
        self.pubs[controller].publish(msg)

    def publish_all(self, base_value):
        self.publish_scaled("pris_thumb_jo_joint", base_value, 2)  # 2 joints
        self.publish_scaled("pris_thumb_DIP_joint", base_value, 3)     # 3 joints
        self.publish_scaled("pris_index_joint", base_value, 4)
        self.publish_scaled("pris_middle_joint", base_value, 4)
        self.publish_scaled("pris_ring_joint", base_value, 4)
        self.publish_scaled("pris_little_joint", base_value, 4)

    def publish_fingers(self, fingers, base_value):
        config = {
            "pris_thumb_jo_joint": 2,
            "pris_thumb_DIP_joint": 3,
            "pris_index_joint": 4,
            "pris_middle_joint": 4,
            "pris_ring_joint": 4,
            "pris_little_joint": 4
        }
        for finger in fingers:
            if finger == "pris_thumb_DIP_joint":
                self.publish_scaled("pris_thumb_jo_joint", base_value, config["pris_thumb_jo_joint"])
            self.publish_scaled(finger, base_value, config[finger])

if __name__ == '__main__':
    HandService()

