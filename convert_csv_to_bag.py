#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf.msg
import rosbag
import math
from sensor_msgs.msg import LaserScan

import time, sys

# update_progress() : Displays or updates a console progress bar
## Accepts a float between 0 and 1. Any int will be converted to a float.
## A value under 0 represents a 'halt'.
## A value at 1 or bigger represents 100%
def update_progress(progress, text):
    barLength = 10 # Modify this to change the length of the progress bar
    status = ""
    if isinstance(progress, int):
        progress = float(progress)
    if not isinstance(progress, float):
        progress = 0
        status = "error: progress var must be float\r\n"
    if progress < 0:
        progress = 0
        status = "Halt...\r\n"
    if progress >= 1:
        progress = 1
        status = "Done...\r\n"
    block = int(round(barLength*progress))
    text = "\r" + text + ": [{0}] {1:.2f}% {2}".format( "#"*block + "-"*(barLength-block), progress*100, status)
    sys.stdout.write(text)
    sys.stdout.flush()

# def convert_odometry(outbag, csv_file_name = 'Bicocca_2009-02-25b-ODOMETRY_XYT.csv'):
def convert_odometry(outbag, csv_file_name = 'odom.csv'):
    data_frame = pd.read_csv(csv_file_name) 
    num_rows = len(data_frame.index)

    # Get the first row
    first_row = data_frame.iloc[0]
    t1, x1, y1, th1 = rospy.Time.from_sec(float(first_row[0])), first_row[4], first_row[5], first_row[6]

    for i, row in data_frame.iterrows():
        update_progress(float(i + 1) / num_rows, 'odometry')

        t = rospy.Time.from_sec(float(row[0]))
        x = float(row[4])
        y = float(row[5])
        th = float(row[6])
        odom = Odometry()
        odom.header.stamp = t
        odom.header.seq = i
        odom.header.frame_id = "odom"
        
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
        
        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    
        # set the velocity
        odom.child_frame_id = "base_link"
        if math.fabs((t - t1).to_sec()) < 1e-4:
            vx, vy, vth = 0.0, 0.0, 0.0
        else:
            dt = (t - t1).to_sec()
            vx = (x - x1) / dt
            vy = (y - y1) / dt
            vth = (th - th1) / dt
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        outbag.write("odom", odom, t)
        
        tf_msg = tf.msg.tfMessage()
        geo_msg = geometry_msgs.msg.TransformStamped()
        geo_msg.header.stamp = t 
        geo_msg.header.seq = i
        geo_msg.header.frame_id = "odom"
        geo_msg.child_frame_id = "base_link"
        geo_msg.transform.translation.x = x
        geo_msg.transform.translation.y = y
        geo_msg.transform.translation.z = 0
        
        geo_msg.transform.rotation.x = odom_quat[0]
        geo_msg.transform.rotation.y = odom_quat[1]
        geo_msg.transform.rotation.z = odom_quat[2]
        geo_msg.transform.rotation.w = odom_quat[3]
        tf_msg.transforms.append(geo_msg)

        outbag.write('/tf', tf_msg, t)

        t1, x1, y1, th1 = t, x, y, th


def construct_TransformStamped(x, y, z, yaw_displacement, parent_frame_id, child_frame_id):
    geo_msg = geometry_msgs.msg.TransformStamped()
    geo_msg.header.frame_id = 'base_link'
    geo_msg.child_frame_id = child_frame_id
    geo_msg.transform.translation.x = x
    geo_msg.transform.translation.y = y
    geo_msg.transform.translation.z = z
    q = tf.transformations.quaternion_from_euler(0,0,yaw_displacement) 
    geo_msg.transform.rotation.x = q[0]
    geo_msg.transform.rotation.y = q[1]
    geo_msg.transform.rotation.z = q[2]
    geo_msg.transform.rotation.w = q[3]
    return geo_msg

def write_static_tf(outbag, tf_msg, t_range):
    i = 0
    print("Generating static tf...")
    for t_float in np.arange(t_range[0], t_range[1], 0.1):
        t = rospy.Time.from_sec(t_float)
        tf_msg.transforms[0].header.stamp = t
        tf_msg.transforms[0].header.seq = i
        tf_msg.transforms[1].header.stamp = t
        tf_msg.transforms[1].header.seq = i
        outbag.write('/tf_static', tf_msg, t)
        i += 1

def convert_laser(outbag, csv_file_name, topic_name, tf_msg):
    data_frame = pd.read_csv(csv_file_name) 
    num_rows = len(data_frame.index)

    t_range = []
    
    for i, row in data_frame.iterrows():
        update_progress(float(i + 1) / num_rows, topic_name)

        t = rospy.Time.from_sec(float(row[0]))
        if not t_range:
            t_range = [float(row[0]), 0.0]
        t_range[1] = float(row[0])
    
        scan = LaserScan()

        scan.header.seq = i
        scan.header.stamp = t
        scan.header.frame_id = tf_msg.child_frame_id
    
        scan.angle_min = - math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / 180
        scan.time_increment = (1.0 / 75.0)/181
        scan.range_min = 0.0
        scan.range_max = 30.0
        scan.intensities = []
        scan.ranges = row[3:184]  
        outbag.write(topic_name, scan, t)
    return t_range



if __name__ == '__main__':
    with rosbag.Bag('output.bag', 'w') as outbag:
        convert_odometry(outbag)

        tf_front = construct_TransformStamped(x=0.08, y=0.0, z=0.45, yaw_displacement=0.0, \
            parent_frame_id='base_link', child_frame_id='front_laser')
        tf_rear = construct_TransformStamped(x=-0.463, y=0.001, z=0.454, yaw_displacement=math.pi, \
            parent_frame_id='base_link', child_frame_id='rear_laser')

        t_range_front = convert_laser(outbag, \
            csv_file_name = 'front.csv', \
            topic_name='front_scan', tf_msg=tf_front)
        t_range_rear = convert_laser(outbag, \
            csv_file_name = 'rear.csv', \
            topic_name='rear_scan', tf_msg=tf_rear)

        tf_msg = tf.msg.tfMessage()
        tf_msg.transforms = [tf_front, tf_rear]
        write_static_tf(outbag, tf_msg, 
            t_range=[min(t_range_front[0], t_range_rear[0]), max(t_range_front[1], t_range_rear[1])])

        print("Done!")
