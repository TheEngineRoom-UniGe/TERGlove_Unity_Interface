#!/usr/bin/env python3
from tkinter.font import names
import rospy
import struct
import socket
import argparse
import rospkg
import os
import json, requests

from sensor_msgs.msg import Imu
from baxter_unity.msg import Imu9

'''LA CLASSE CHE GESTISCE LA COMUNICAZIONE SOCKET E LA RICEZIONE DEI DATI'''
class ImuDriver:

    def __init__(self, udp_port):
        # print(json.loads(requests.get("https://ip.seeip.org/jsonip?").text)["ip"])
        # ipaddr = json.loads(requests.get("https://ip.seeip.org/jsonip?").text)["ip"]
        ipaddr = '130.251.13.131'
        self.udp_ip = ipaddr #'130.251.13.158' #STATIC IP ETHERNET CABLE; '192.168.0.101' #Hard-coded static IP
        self.udp_port = udp_port
        # print(ipaddr)
        self.counter = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('TERGlove_ROS_driver')
        with open("config/driver_config.txt") as f:
            lines = f.readlines()

        self.names = {}
        for line in lines:
            values = line.strip().split(" ")
            self.names[int(values[0])] = values[1]

        # print(sorted([n for n in self.names.values()]))
        self.namelist = sorted([n for n in self.names.values() if 'wrist' not in n])
        self.connectedIMUs = {n:0 for n in self.namelist}
        # print(self.connectedIMUs)
        self.msgCounter = 0


        '''CREAZIONE E CONNESSIONE AL SOCKET'''
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.udp_ip, self.udp_port))

        rospy.loginfo("IMU listener initialized on IP {0} with port {1}".format(self.udp_ip, self.udp_port))
        while not rospy.is_shutdown():
            '''LETTURA DA SOCKET DI UN PACCHETTO DI DIMENSIONE 29+6 BYTE'''
            data = sock.recv(29+6)  # (29) # buffer size is 1024 bytes
            if not data:
                '''MESSAGGIO VUOTO'''
                print("no data")
            else:

                ''' IL MESSAGGIO NON È VUOTO E PROCEDIAMO A SPACCHETTARLO'''
                ID = data[28+6]
                # Fix to ensure topic is published under same name with both devices
                # print('ID: ', ID, ' \t - \t', self.names[ID])
                mux = int(ID/100)
                channel = int((ID-mux*100)/10)
                address = int(ID-mux*100-channel*10)

                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.orientation.x = struct.unpack('<f', data[0:4])[0]
                msg.orientation.y = struct.unpack('<f', data[4:8])[0]
                msg.orientation.z = struct.unpack('<f', data[8:12])[0]
                msg.orientation.w = struct.unpack('<f', data[12:16])[0]
                msg.header.frame_id = 'base_link'

                msg.linear_acceleration.x = struct.unpack('<h', data[16:18])[0]
                msg.linear_acceleration.y = struct.unpack('<h', data[18:20])[0]
                msg.linear_acceleration.z = struct.unpack('<h', data[20:22])[0]

                msg.angular_velocity.x = struct.unpack('<h', data[22:24])[0]
                msg.angular_velocity.y = struct.unpack('<h', data[24:26])[0]
                msg.angular_velocity.z = struct.unpack('<h', data[26:28])[0]

                msg2 = Imu9()
                msg2.header.frame_id = 'base_link'
                msg2.header.stamp = msg.header.stamp
                msg2.pose.orientation = msg.orientation
                msg2.pose.position.x = 0
                msg2.pose.position.y = 0
                msg2.pose.position.z = 0

                msg2.linear_acceleration = msg.linear_acceleration
                msg2.angular_velocity = msg.angular_velocity
                msg2.magnetometer.x = struct.unpack('<h', data[28:30])[0]
                msg2.magnetometer.y = struct.unpack('<h', data[30:32])[0]
                msg2.magnetometer.z = struct.unpack('<h', data[32:34])[0]

                '''QUESTA VARIABILE CONTIENE IL NOME DEL SENSORE E VA USATO PER SMISTARE I MESSAGGI SU FILE DIVERSI (IDEALMENTE VOGLIAMO UNA CARTELLA /recordings AL CUI
                    INTERNO VENGANO CREATI 11 FILE DIVERSI, UNO PER OGNI SENSORE'''
                sensor_name = self.names[ID] + "-Pose"

                '''TMP CONTIENE UNA STRINGA CON TIMESTAMP, ORIENTAZIONE (X,Y,Z,W), ACCELERAZIONI (X,Y,Z) E MAGNETOMETRO (X,Y,Z) SEPARATE DA VIRGOLA'''
                tmp = (str(msg2.header.stamp) + "," + 
                    str(msg2.pose.orientation.x) + "," + str(msg2.pose.orientation.y) + "," + str(msg2.pose.orientation.z) + "," + str(msg2.pose.orientation.w) + "," + 
                    str(msg2.linear_acceleration.x) + "," + str(msg2.linear_acceleration.y) + "," + str(msg2.linear_acceleration.z) + "," +
                    str(msg2.angular_velocity.x) + "," + str(msg2.angular_velocity.y) + "," + str(msg2.angular_velocity.z) + "," +
                    str(msg2.magnetometer.x) + "," + str(msg2.magnetometer.y) + "," + str(msg2.magnetometer.z) + "\n")

                '''QUI BISOGNA INSERIRE LA STAMPA SU FILE DELLA STRINGA TMP CHE CONTIENE TUTTE LE INFORMAZIONI CHE CI SERVONO'''
                
                #//////////////////////////////////////

                folderpath = "/home/simone/Desktop/daler/data"#"C:/recordings"
                # try:
                #     os.mkdir(folderpath)
                # except FileExistsError:
                #     pass

                filename = folderpath + "/" + sensor_name  +  ".txt"
                f = open(filename, "a+")
                f.write(tmp)
                f.close()

                #/////////////////////////////////////

                
                self.connectedIMUs[self.names[ID]] = 1
                self.msgCounter += 1
                if self.msgCounter == 100:
                    self.connectedIMUs = {n:0 for n in self.namelist}
                    self.msgCounter = 0
                elif  self.msgCounter > 30:
                    print(self.connectedIMUs, end='\r')
                '''QUESTA È LA PARTE CHE COMUNICAVA CON ROS E NON CI SERVE PIÙ'''
                pub2 = rospy.Publisher(self.names[ID]+"Pose", Imu9, queue_size=0)
                pub2.publish(msg2)


'''QUI SI ISTANZIA IL NODO ROS CHE NON CI SERVE PIÙ E SI LEGGONO I PARAMETRI PASSATI DA LINEA DI COMANDO (X ES. LA PORTA SU CUI APRIRE IL SOCKET)'''
def main(): 
    rospy.init_node('imu_driver', disable_signals=True)

    # Parse arguments from launch file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-p', '--port', required=True, type=int,
        help='UDP port'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    imu_driver = ImuDriver(args.port)
    imu_driver.listener()


'''CHIAMIAMO IL MAIN()'''
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
