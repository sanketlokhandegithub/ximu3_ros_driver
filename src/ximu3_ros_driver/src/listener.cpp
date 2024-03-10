/*
* Author : Sanket Lokhande
* Date Created : 21 Feb 2024
* Last Modified : 9 March 2024
* Description : This ros listener will listen to accelerometer values broadcasted by topic sensor0/accel. modify to see rest of the topics
* Usage: $rosrun ximu3_ros_driver listener
*/


#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ximu3_ros_driver/Accel.h"


void accelCallback(const ximu3_ros_driver::Accel & msg)
{
  ROS_INFO("I heard: [%ld] [%f] [%f] [%f] ", msg.timestamp, msg.accel_x, msg.accel_y, msg.accel_z);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("sensor0/accel", 1000, accelCallback);

  ros::spin();


  return 0;
}
