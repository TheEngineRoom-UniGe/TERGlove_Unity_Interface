#!/usr/bin/env python3
import rospy
import struct
import socket
import argparse
import rospkg
import os
import json, requests
import numpy as np
from pyquaternion import Quaternion 
from sensor_msgs.msg import Imu
from baxter_unity.msg import Imu9
import threading 
from sksurgerycore.algorithms.averagequaternions import average_quaternions

class IMU_calibration:

    def __init__(self):
        self.global_first_back = 0
        self.global_first_finger = 0
        self.initial_offset = 0
        self.current_back = 0

        self.sensor_names = ['MC3', 'MC1', 'PD1', 'PP2', 'PM2', 'PP3', 'PM3', 'PP4', 'PM4', 'PP5', 'PM5']
        self.initial_rotations = [0,0,0,0,0,0,0,0,0,0,0]
        self.initial_references = [0,0,0,0,0,0,0,0,0,0,0]

        self.calibration_readings = [[],[],[],[],[],[],[],[],[],[],[]]

        self.pub_parent = rospy.Publisher("/MC3", Imu9, queue_size=0)
        self.pub_child = rospy.Publisher("/Child", Imu9, queue_size=0)
        self.pub_diff = rospy.Publisher("/Diff", Imu9, queue_size=0)

        self.timer = threading.Timer(5,lambda: self.stopCalibration())
        self.calibrating = True

    def stopCalibration(self):
        self.calibrating = False
        print('Timer', self.calibrating)

    def averageQuaternions(self,Q):
        # Number of quaternions to average
        new_list = []

        for i in range(len(Q)):
            q = Q[i]
            q = np.asarray([float(elem) for elem in q])
            new_list.append(q)
        Q = np.asarray(new_list)

        return Quaternion(average_quaternions(Q)[0],
                            average_quaternions(Q)[1],
                            average_quaternions(Q)[2],
                            average_quaternions(Q)[3],
                            )


    def callback_parent(self, data):

        tmp_back = Quaternion(
        data.pose.orientation.w,
        data.pose.orientation.x, 
        data.pose.orientation.y, 
        data.pose.orientation.z
        )

        if self.calibrating:
            self.calibration_readings[0].append(tmp_back)
            return

        if self.initial_references[0] == 0:
            #print('Prima orientazione', self.sensor_names[0], 'ricevuta')
            print('MC3 Calibration starting from', len(self.calibration_readings[0]), 'samples')
            self.initial_references[0] = self.averageQuaternions(self.calibration_readings[0])
            # print(self.initial_references[0])

        self.current_back = tmp_back


    def callback_child(self, data, topic_name):

        topic_name=topic_name.replace('Pose','')
        s_idx = self.sensor_names.index(topic_name)
        # print(topic_name)

        finger_orientation = Quaternion(
        data.pose.orientation.w,
        data.pose.orientation.x, 
        data.pose.orientation.y, 
        data.pose.orientation.z
        )

        if self.calibrating:
            self.calibration_readings[s_idx].append(finger_orientation)
            return
        if self.initial_references[0] == 0 or self.current_back == 0:
            return       

        offset = Quaternion(0,1,0,0)
        if topic_name[-1] == '1':
            offset = Quaternion(0,0.7071068, 0, 0.7071068)

        # print(offset)


        if self.initial_rotations[s_idx] == 0:     #global_first_finger == 0:
            print(topic_name, 'Calibration starting from', len(self.calibration_readings[s_idx]), 'samples')
            self.initial_rotations[s_idx] = finger_orientation

        if self.initial_rotations[s_idx] != 0 and self.initial_references[s_idx] == 0:
            avg = self.averageQuaternions(self.calibration_readings[s_idx])
            self.initial_references[s_idx] = self.initial_references[0] * offset * avg.inverse   # global_first_back *



        rotated_finger = self.initial_references[s_idx] * finger_orientation 
        rotated_finger = rotated_finger 
        rotated_back = self.current_back


        diff =  rotated_back.inverse * rotated_finger


        msg_parent = Imu9()
        msg_parent.header.frame_id = 'MC3'
        msg_parent.header.stamp = rospy.Time.now()
        msg_parent.pose.orientation.x = rotated_back.x
        msg_parent.pose.orientation.y = rotated_back.y
        msg_parent.pose.orientation.z = rotated_back.z
        msg_parent.pose.orientation.w = rotated_back.w
        msg_child = Imu9()
        msg_child.header.frame_id = topic_name
        msg_child.header.stamp = rospy.Time.now()
        msg_child.pose.orientation.x = rotated_finger.x
        msg_child.pose.orientation.y = rotated_finger.y
        msg_child.pose.orientation.z = rotated_finger.z
        msg_child.pose.orientation.w = rotated_finger.w
        msg_diff = Imu9()
        msg_diff.header.frame_id = topic_name + '_diff'
        msg_diff.header.stamp = rospy.Time.now()
        msg_diff.pose.orientation.x = diff.x
        msg_diff.pose.orientation.y = diff.y
        msg_diff.pose.orientation.z = diff.z
        msg_diff.pose.orientation.w = diff.w

        self.pub_parent.publish(msg_parent)
        self.pub_child.publish(msg_child)
        self.pub_diff.publish(msg_diff)


    def main(self): 
        rospy.init_node('imu_debug', disable_signals=True)

        for s in self.sensor_names:
            s_topic = s + 'Pose'

            if s == 'MC3':
                rospy.Subscriber( s_topic, Imu9, self.callback_parent)
            else:
                rospy.Subscriber( s_topic, Imu9, self.callback_child, s_topic)

        print('Listening...')
        print('Timer', self.calibrating)
        self.timer.start()

        rospy.spin()
        


'''CHIAMIAMO IL MAIN()'''
if __name__ == '__main__':
    try:
        imu_c = IMU_calibration()
        imu_c.main()
    except rospy.ROSInterruptException:
        pass
