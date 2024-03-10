# ximu3_ros_driver

This is a ROS driver for connecting multiple XIMU3 devices via TCP or UDP connection and stream the IMU data to the ROS server

Usage:


0. Open a terminal and execute the following commands
1. mkdir -p ~/catkin_ws/src
2. cd ~/catkin_ws
3. catkin init
4. cd src
5. git clone https://github.com/sanketlokhandegithub/ximu3_ros_driver.git
6. cd ximu3_ros_driver/src/ximu3_ros_driver/
7. mkdir lib && cd lib
8. wget https://github.com/xioTechnologies/x-IMU3-Software/releases/latest/download/x-IMU3-API-x86_64-unknown-linux-gnu.zip
9. unzip x-IMU3-API-x86_64-unknown-linux-gnu.zip
10. roscore (This should start your ros server)
11. open a new terminal and execute the following
12. cd ~/catkin_ws
13. catkin_make
14. source devel/setup.bash
15. rosrun ximu3_ros_driver ximu3_node 0 (If you replace 0 with 1 or 2, it will connect to the other XIMU3 devices on the wifi network )
17. You should see your device flash the LED a few times before it starts to transmit data to the ros server
18. open a new terminal and execute cd ~/catkin_ws
19. source devel/setup.bash
20. rostopic list (You should see the list of topics that are broadcasting)
21. rostopic echo /sensor0/accel should display your transmitted accelerometer data
