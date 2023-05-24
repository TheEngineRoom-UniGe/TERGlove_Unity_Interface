#!/usr/bin/env python3
import struct
import socket
import argparse
import os
import json, requests
import numpy as np
from pyquaternion import Quaternion 
import threading 
from sksurgerycore.algorithms.averagequaternions import average_quaternions
from IMU_class import IMU
from IMU_class import Quaternion as DataQuat
import keyboard
import pickle

class IMU_calibration:

    def __init__(self):
        self.global_first_back = 0
        self.global_first_finger = 0
        self.initial_offset = 0
        self.current_back = 0

        self.json_messages = [None,None,None,None,None,None,None,None,None,None]

        self.sensor_names = ['MC3', 'MC1', 'PD1', 'PP2', 'PM2', 'PP3', 'PM3', 'PP4', 'PM4', 'PP5', 'PM5']
        self.initial_rotations = [0,0,0,0,0,0,0,0,0,0,0]
        self.initial_references = [0,0,0,0,0,0,0,0,0,0,0]

        self.calibration_readings = [[],[],[],[],[],[],[],[],[],[],[]]

        # self.pub_parent = rospy.Publisher("/MC3", Imu9, queue_size=0)
        # self.pub_child = rospy.Publisher("/Child", Imu9, queue_size=0)
        # self.pub_diff = rospy.Publisher("/Diff", Imu9, queue_size=0)

        self.timer = threading.Timer(5,lambda: self.stopCalibration())

        self.calibrating = True

        self.in_udp_ip = '130.251.13.118'
        self.in_udp_port = 2395
        self.out_udp_ip = '130.251.13.118'
        self.out_udp_port = 2400

        self.in_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.in_sock.bind((self.in_udp_ip, self.in_udp_port))
        self.in_sock.settimeout(1)

        self.out_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)



        with open("config/driver_config.txt") as f:
            lines = f.readlines()
        self.names = {}
        for line in lines:
            values = line.strip().split(" ")
            self.names[int(values[0])] = values[1]
            
        self.IMU = IMU(-1, DataQuat(0,0,0,0))

    def stopCalibration(self):
        self.calibrating = False
        print('Timer', self.calibrating)

    def schedule_udp_message_sending(self):
        # Schedule the initial UDP message sending
        self.send_udp_message()

        # Schedule subsequent UDP message sending every 60 seconds
        threading.Timer(60, self.schedule_udp_message_sending).start()

    def send_udp_message(self):
        return
        # print(json.dumps(self.json_messages))
        self.out_sock.sendto(json.dumps(self.imu_encoder(self.json_messages)).encode(), (self.out_udp_ip, self.out_udp_port))

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
        # print('callback_parent', self.calibrating)
        tmp_back = Quaternion(
        data.orientation.w,
        data.orientation.x, 
        data.orientation.y, 
        data.orientation.z
        )

        if self.calibrating:
            self.calibration_readings[0].append(tmp_back)
            return

        if self.initial_references[0] == 0:
            #print('Prima orientazione', self.sensor_names[0], 'ricevuta')
            print('MC3 Calibration starting from', len(self.calibration_readings[0]), 'samples')
            self.initial_references[0] = self.averageQuaternions(self.calibration_readings[0])
            # print(self.initial_references[0])

        msg_parent = IMU(0, orientation=(tmp_back.x, tmp_back.y, tmp_back.z, tmp_back.w))
        self.out_sock.sendto(json.dumps(self.imu_encoder(msg_parent)).encode(), (self.out_udp_ip, self.out_udp_port))

        self.current_back = tmp_back


    def callback_child(self, data, topic_name):

        # topic_name=topic_name.replace('Pose','')
        s_idx = topic_name
        # print(topic_name)

        finger_orientation = Quaternion(
        data.orientation.w,
        data.orientation.x, 
        data.orientation.y, 
        data.orientation.z
        )

        if self.calibrating:
            self.calibration_readings[s_idx].append(finger_orientation)
            return
        if self.initial_references[0] == 0 or self.current_back == 0:
            return       

        offset = Quaternion(0,1,0,0)
        if topic_name in [1,2]:
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

        # msg_parent = IMU(0, orientation=(rotated_back.x, rotated_back.y, rotated_back.z, rotated_back.w))
        msg_child = IMU(s_idx, orientation=(rotated_finger.x, rotated_finger.y, rotated_finger.z, rotated_finger.w))

        # print(json.dumps(self.imu_encoder(msg_parent)).encode())
        # print(json.dumps(self.imu_encoder(msg_child)).encode())
        # self.out_sock.sendto(json.dumps(self.imu_encoder(msg_parent)).encode(), (self.out_udp_ip, self.out_udp_port))
        self.out_sock.sendto(json.dumps(self.imu_encoder(msg_child)).encode(), (self.out_udp_ip, self.out_udp_port))
        # self.json_messages[0] = msg_parent
        # self.json_messages[s_idx] = msg_child
        # print(json_messages)


    # Custom encoder function for Quaternion class
    def quaternion_encoder(self, obj):
        if isinstance(obj, Quaternion):
            return obj.__dict__
        raise TypeError(f"Object of type '{type(obj)}' is not JSON serializable")


    # Custom encoder function for IMU class
    def imu_encoder(self, obj):
        if isinstance(obj, IMU):
            return obj.__dict__
        raise TypeError(f"Object of type '{type(obj)}' is not JSON serializable")


    def buffer_unpack(self, data, verbose = False):
        imu_msg = IMU(-1, DataQuat(0,0,0,0))
        ID = data[28+6]

        imu_msg.orientation.x = struct.unpack('<f', data[0:4])[0]
        imu_msg.orientation.y = struct.unpack('<f', data[4:8])[0]
        imu_msg.orientation.z = struct.unpack('<f', data[8:12])[0]
        imu_msg.orientation.w = struct.unpack('<f', data[12:16])[0]
        imu_msg.sensor_id = self.sensor_names.index(self.names[ID])

        if verbose: self.print_incoming_data(imu_msg)
        return imu_msg


    def print_incoming_data(self, data):
        tmp = ( str(data.sensor_id) + "," + 
                str(data.orientation.x) + "," +
                str(data.orientation.y) + "," +
                str(data.orientation.z) + "," +
                str(data.orientation.w) + "\n")
        print(tmp, end='\r')



    def listener(self): 
        while True:
            if keyboard.is_pressed('shift+q'):
                break
            try:
                data = self.in_sock.recv(29+6)  # (29) # buffer size is 1024 bytes
            except socket.timeout as se:
                print("timeout, press shift+q to exit", se)
                continue
            if not data:
                print("no data")
            else:
                self.IMU = self.buffer_unpack(data, verbose=False)

            if self.IMU.sensor_id == 0:
                self.callback_parent(self.IMU)
            else:
                self.callback_child(self.IMU, self.IMU.sensor_id)
                
            if self.calibrating and not self.timer.is_alive():
                print('Listening...')
                print('Timer', self.calibrating)
                self.timer.start()
            
            # '''start a new thread'''
            # thread = threading.Thread(target=self.schedule_udp_message_sending)
            # thread.start()


'''CHIAMIAMO IL MAIN()'''
if __name__ == '__main__':
    imu_c = IMU_calibration()
    imu_c.listener()

