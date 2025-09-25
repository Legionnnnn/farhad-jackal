#!/usr/bin/env python3
import rospy
import struct
from can_msgs.msg import Frame
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import sin, cos

class CANPublisher:
    def __init__(self):
        rospy.init_node("can_publisher", anonymous=True)
        self.pub = rospy.Publisher("/sent_messages", Frame, queue_size=10)
        self.rate = rospy.Rate(50)  # 50 Hz

        self.frame = Frame()
        self.frame.id = 0x200
        self.frame.is_rtr = False
        self.frame.is_extended = False
        self.frame.dlc = 8
        self.frame.data = [0] * 8

        self.last_pose = None
        self.last_spd = None

        self.sub_pose = rospy.Subscriber("/robot_pose", PoseStamped, self.callback_pose)
        self.sub_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odom)

    def publish_value(self, values):
        for i, value in enumerate(values):
            data_bytes = struct.pack("<h", int(value * 1000))  # CENTIMETERS Values as signed 16-bit integers
            data = list(data_bytes)
            self.frame.data[i * 2] = data[0]
            self.frame.data[i * 2 + 1] = data[1]

        rospy.loginfo(f"Publishing CAN frame ID=0x{self.frame.id:X}, data={list(self.frame.data)}")
        rospy.loginfo(f"Value: {values}")

        self.pub.publish(self.frame)

    def callback_pose(self, msg):
        self.last_pose = msg.pose
    
    def callback_odom(self, msg):
        self.last_spd = msg.twist.twist

    def run(self):
        while not rospy.is_shutdown():
            if self.last_pose is None or self.last_spd is None :
                self.rate.sleep()
                continue
            x = self.last_pose.position.x
            y = self.last_pose.position.y
            _, _, theta = euler_from_quaternion([self.last_pose.orientation.x,
                                               self.last_pose.orientation.y,
                                               self.last_pose.orientation.z,
                                               self.last_pose.orientation.w])
            vx = self.last_spd.linear.x * cos(theta)
            vy = self.last_spd.linear.x * sin(theta)

            self.publish_value((x, y, vx, vy))
            self.rate.sleep()


if __name__ == "__main__":
    try:
        can_pub = CANPublisher()
        can_pub.run()
    except rospy.ROSInterruptException:
        pass
