#!/usr/bin/env python

import rospy
import actionlib
from jenga_msgs.msg import GetBlocksAction, GetBlocksResult
from sensor_msgs.msg import Image

class BlockDetector(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.action_server = actionlib.SimpleActionServer("get_blocks", GetBlocksAction, self.execute_action, False)
        self.action_server.start()

    def image_callback(self, image_msg):
        # Process image to detect blocks
        # ...
        # Publish progress feedback to client
        feedback = GetBlocksAction.Feedback()
        feedback.feedback_progress = 0.5
        self.action_server.publish_feedback(feedback)

    def execute_action(self, goal):
        result = GetBlocksResult()
        # Do something with goal.input_text
        # ...
        # Set action result and return
        result.success = True
        self.action_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("block_detector")
    block_detector = BlockDetector()
    rospy.spin()