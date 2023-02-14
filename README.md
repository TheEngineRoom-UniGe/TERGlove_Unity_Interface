# Glove_Unity_Interface
Unity Interface to display the glove motion in realtime


CONFIG -- contains a dictionary to dispatch the packets from the glove to the correct finger handler
DATA and OLD_DATA -- contain recordings of the imu data
MODELS -- contains the blend and fbx files for the left and right skeleton hands and the osim file for opensim

========================

glove_driver.py:
------------
opens the socket, read the messages from UDP and send them over ROS. It can also record the data.

========================

stoCreator.py:
------------
can read the recordings of the imus and join them in a single sto file that is compatible with opensim IMU placer and IMU IK tools

========================

testRPY.py:
------------
reads the ROS messages from the glove driver, manipulates them to calibrate the initial state of the hand and can remove the rotation of the back of the hand from the fingers. The messages are then remapped to the /Parent and /Child topics to communicate with the Unity application. Each /Child message has the link name in the field base_link

PER LANCIARE LA DEMO:

./glove_driver.py -p 2395
./testRPY.py
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=130.251.13.131 tcp_port:=11111
UNITY
