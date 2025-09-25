#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

import serial
import time
import struct
import sys
import threading

latest_covariance = None

def open_serial(port='/dev/ttyACM0', baudrate=9600, timeout=1):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Attendre l'initialisation de l'Arduino
        print("Serial port opened successfully.")
        return ser
    except serial.SerialException as e:
        print(f"Connection error on Serial port: {e}")
        return None

def send_command_binary(ser, command):
    "double command"
    if ser and ser.is_open:
        command_bytes = struct.pack('<d', command)
        ser.write(command_bytes)
        print(f"Sent binary command: {command}")

def read_response_binary(ser):
    if ser and ser.is_open:
        response = ser.read(8)  # Lire 8 octets pour un double
        if len(response) == 8:
            value = struct.unpack('<d', response)[0]
            print(f"Received binary response: {value}")
            return value
        else:
            print("Received incomplete binary response.")
    return None

def pose_callback(data):
    global latest_covariance
    latest_covariance = data.pose.covariance

def get_pose_from_tf(tf_buffer):
    try:
        trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        position = trans.transform.translation
        orientation = trans.transform.rotation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, theta = euler_from_quaternion(quaternion)
        return (round(position.x, 4), round(position.y, 4), round(theta, 4))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        return None

def is_covariance_good():
    if latest_covariance is None:
        rospy.logwarn("Covariance data not available yet.")
        return False
    covariances = np.array([
            latest_covariance[0],  # x variance
            latest_covariance[7],  # y variance
            latest_covariance[35]  # theta variance
        ])
    return np.mean(covariances[0:2]) < 2 or covariances[2] < 2

def reader_thread(ser):
    while True:
        response = read_response_binary(ser)
        if response is not None:
            pass
            # print(f"Thread received: {response}")
        time.sleep(0.1)  # Adjust sleep time as needed

if __name__ == '__main__':
    rospy.init_node('data_sender', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    last_pose = None
    
    rate = rospy.Rate(10)  # 10 Hz

    ser = open_serial(port = f'/dev/ttyUSB0', baudrate=57600, timeout=1)
    if not ser:
        rospy.logerr("Failed to open serial port. Exiting.")
        sys.exit(1)
    
    thread = threading.Thread(target=reader_thread, args=(ser,))
    thread.daemon = True  # Daemonize thread
    thread.start()

    try :
        while not rospy.is_shutdown() :
            if is_covariance_good():
                current_pose = get_pose_from_tf(tf_buffer)
                if current_pose:
                    send_command_binary(ser, current_pose[0])  # x position
                    send_command_binary(ser, current_pose[1])  # y position
                    send_command_binary(ser, 0              )  # z position

            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received. Exiting...")
    finally:
        if ser:
            ser.close()
            rospy.loginfo("Serial port closed.")
        rospy.loginfo("Node terminated.")

