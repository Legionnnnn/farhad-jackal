#!/usr/bin/env python3
import rospy
import struct
from can_msgs.msg import Frame
from geometry_msgs.msg import PoseStamped

class CanSubscriber:
    def __init__(self):
        rospy.init_node("can_subscriber", anonymous=True)
        self.pose_pub = rospy.Publisher('/drone/pose', PoseStamped, queue_size=10)
        rospy.Subscriber("/received_messages", Frame, self.callback)

    def publish_drone_pose(self, x, y, z):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.w = 1.0  # Neutral orientation
        self.pose_pub.publish(pose_msg)

    def callback(self, msg):
        x = struct.unpack("<h", bytes(msg.data[0:2]))[0] / 100.0
        y = struct.unpack("<h", bytes(msg.data[2:4]))[0] / 100.0
        z = struct.unpack("<h", bytes(msg.data[4:6]))[0] / 100.0
        self.publish_drone_pose(x, y, z)


if __name__ == "__main__":
    node = CanSubscriber()
    rospy.spin()
