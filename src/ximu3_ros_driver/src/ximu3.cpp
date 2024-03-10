/*
* Author : Sanket Lokhande
* Date Created : 21 Feb 2024
* Last Modified : 9 March 2024
* Description: This ros driver was created to connect 3 XIMU3 devices and broadcast the data simultaneously to ROS network
* Usage: $rosrun ximu3_ros_driver ximu3_node 0  #0 /1 /2 for devices as indicated in connection.hpp file
*/



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ximu3_ros_driver/connection.hpp"


#include "ximu3_ros_driver/Accel.h"
#include "ximu3_ros_driver/Gyro.h"
#include "ximu3_ros_driver/Quaternion.h"
#include "ximu3_ros_driver/Euler.h"
#include "ximu3_ros_driver/LinearAccel.h"
#include "ximu3_ros_driver/Mag.h"
#include "ximu3_ros_driver/RotMatrix.h"


int main(int argc, char **argv)
{
    int inputDevice = 0;
    if(argv[argc-1] == ZERO )
    {
        inputDevice = 0;
    }
    else if(argv[argc-1] == ONE)
    {
        inputDevice = 1;;
    }
    else if(argv[argc-1] == TWO)
    {
        inputDevice = 2;
    }
    else 
    {
        printf("No input given. Will connect the first device found\n");
        inputDevice = 0;
    }

    XIMU3_SensorMessage sensorMessage;
    memset(&sensorMessage, 0, sizeof(XIMU3_SensorMessage));
    
    connection connection(connection_type::tcpconnection, sensorMessage, inputDevice);


    std::string nodeName = "ximu3sensor" + std::to_string(inputDevice);
    ros::init(argc, argv, nodeName);

    ros::NodeHandle n;

    std::string topicName = "/sensor" + std::to_string(inputDevice) + "/";

    ros::Publisher accel_pub =    n.advertise<ximu3_ros_driver::Accel>(topicName.append("accel"), 10000);
    topicName = "/sensor" + std::to_string(inputDevice) + "/";
    ros::Publisher gyro_pub =     n.advertise<ximu3_ros_driver::Gyro>(topicName.append("gyro"), 10000);
    topicName = "/sensor" + std::to_string(inputDevice) + "/";
    ros::Publisher mag_pub =      n.advertise<ximu3_ros_driver::Mag>(topicName.append("mag"), 10000);
    topicName = "/sensor" + std::to_string(inputDevice) + "/";
    ros::Publisher quat_pub =     n.advertise<ximu3_ros_driver::Quaternion>(topicName.append("quat"), 10000);
    topicName = "/sensor" + std::to_string(inputDevice) + "/";
    ros::Publisher rot_pub =      n.advertise<ximu3_ros_driver::RotMatrix>(topicName.append("rot"), 10000);
    topicName = "/sensor" + std::to_string(inputDevice) + "/";
    ros::Publisher euler_pub =    n.advertise<ximu3_ros_driver::Euler>(topicName.append("euler"), 10000);
    topicName = "/sensor" + std::to_string(inputDevice) + "/";
    ros::Publisher linaccel_pub = n.advertise<ximu3_ros_driver::LinearAccel>(topicName.append("linaccel"), 10000);  

    ximu3_ros_driver::Accel         accelMsg;
    ximu3_ros_driver::Gyro          gyroMsg;
    ximu3_ros_driver::Mag           magMsg;
    ximu3_ros_driver::Euler         eulerMsg;
    ximu3_ros_driver::LinearAccel   linAccMsg;
    ximu3_ros_driver::RotMatrix     rotMatMsg;
    ximu3_ros_driver::Quaternion    quatMsg;

    ros::Rate loop_rate(200);

    while (ros::ok())
    {

        accelMsg.timestamp = sensorMessage.inertialMessage.timestamp;
        accelMsg.accel_x = sensorMessage.inertialMessage.accelerometer_x;
        accelMsg.accel_y = sensorMessage.inertialMessage.accelerometer_y;
        accelMsg.accel_z = sensorMessage.inertialMessage.accelerometer_z;
        accel_pub.publish(accelMsg);

        gyroMsg.timestamp = sensorMessage.inertialMessage.timestamp;
        gyroMsg.gyro_x = sensorMessage.inertialMessage.gyroscope_x;
        gyroMsg.gyro_y = sensorMessage.inertialMessage.gyroscope_y;
        gyroMsg.gyro_z = sensorMessage.inertialMessage.gyroscope_z;
        gyro_pub.publish(gyroMsg);

        magMsg.timestamp = sensorMessage.magnetometerMessage.timestamp;
        magMsg.mag_x =      sensorMessage.magnetometerMessage.x;
        magMsg.mag_y =      sensorMessage.magnetometerMessage.y;
        magMsg.mag_z =      sensorMessage.magnetometerMessage.z;
        mag_pub.publish(magMsg);

        eulerMsg.timestamp = sensorMessage.eulerMessage.timestamp;
        eulerMsg.roll = sensorMessage.eulerMessage.roll;
        eulerMsg.pitch = sensorMessage.eulerMessage.pitch;
        eulerMsg.yaw = sensorMessage.eulerMessage.yaw;
        euler_pub.publish(eulerMsg);

        linAccMsg.timestamp = sensorMessage.lineaccelMessage.timestamp;
        linAccMsg.quat_w = sensorMessage.lineaccelMessage.quaternion_x;
        linAccMsg.quat_x = sensorMessage.lineaccelMessage.quaternion_w;
        linAccMsg.quat_y = sensorMessage.lineaccelMessage.quaternion_y;
        linAccMsg.quat_z = sensorMessage.lineaccelMessage.quaternion_z;
        linAccMsg.accel_x = sensorMessage.lineaccelMessage.acceleration_x;
        linAccMsg.accel_y = sensorMessage.lineaccelMessage.acceleration_y;
        linAccMsg.accel_z = sensorMessage.lineaccelMessage.acceleration_z;
        linaccel_pub.publish(linAccMsg);
        

        rotMatMsg.timestamp = sensorMessage.rotationMatrixMessage.timestamp;
        rotMatMsg.rot_xx = sensorMessage.rotationMatrixMessage.xx;
        rotMatMsg.rot_xy = sensorMessage.rotationMatrixMessage.xy;
        rotMatMsg.rot_xz = sensorMessage.rotationMatrixMessage.xz;
        rotMatMsg.rot_yx = sensorMessage.rotationMatrixMessage.yx;
        rotMatMsg.rot_yy = sensorMessage.rotationMatrixMessage.yy;
        rotMatMsg.rot_yz = sensorMessage.rotationMatrixMessage.yz;
        rotMatMsg.rot_zx = sensorMessage.rotationMatrixMessage.zx;
        rotMatMsg.rot_zy = sensorMessage.rotationMatrixMessage.zy;
        rotMatMsg.rot_zz = sensorMessage.rotationMatrixMessage.zz;
        rot_pub.publish(rotMatMsg);

        quatMsg.timestamp = sensorMessage.quaternionMessage.timestamp;
        quatMsg.quat_w = sensorMessage.quaternionMessage.w;
        quatMsg.quat_x = sensorMessage.quaternionMessage.x;
        quatMsg.quat_y = sensorMessage.quaternionMessage.y;
        quatMsg.quat_z = sensorMessage.quaternionMessage.z;
        quat_pub.publish(quatMsg);
        
        ros::spinOnce();

        loop_rate.sleep();

    }

    return 0;
}

