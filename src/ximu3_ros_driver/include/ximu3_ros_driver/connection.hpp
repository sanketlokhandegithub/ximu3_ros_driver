#pragma once
#ifndef __CONNECTION_HPP__
#define __CONNECTION_HPP__

#include <unistd.h>
#include "Ximu3.h"
#include <functional>
#include <string>
#include <cstring>
#include <type_traits>
#include <vector>
#include <chrono>
#include <inttypes.h> // PRIu64
#include <iostream>
#include <stdio.h>
#include <thread>


#define TIMESTAMP_FORMAT "%8" PRIu64 " us"
#define UINT32_FORMAT " %8" PRIu32
#define UINT64_FORMAT " %8" PRIu64
#define FLOAT_FORMAT " %8.3f"
#define STRING_FORMAT " \"%s\""

std::string DeviceAP0 = "65577xxx";
std::string DeviceCL1 = "65577xxx";
std::string DeviceCL2 = "65577xxx";

std::string ZERO = "0";
std::string ONE = "1";
std::string TWO = "2";

enum class connection_type : uint8_t
{
    tcpconnection,
    udpconnection
};

typedef struct XIMU3_SensorMessage
{
    ximu3::XIMU3_EulerAnglesMessage         eulerMessage;
    ximu3::XIMU3_InertialMessage            inertialMessage;
    ximu3::XIMU3_MagnetometerMessage        magnetometerMessage;
    ximu3::XIMU3_RotationMatrixMessage      rotationMatrixMessage;
    ximu3::XIMU3_QuaternionMessage          quaternionMessage;
    ximu3::XIMU3_LinearAccelerationMessage  lineaccelMessage;
} XIMU3_SensorMessage;

XIMU3_SensorMessage* this_senMsg;

class connection
{
    private:
        ximu3::XIMU3_Connection* this_connection;
        
    public:
        
        connection(const connection&) = delete;
        connection(connection&&) = delete;
        connection(connection_type tcp_udp, XIMU3_SensorMessage &senMsg, int _inputDevice) noexcept(false)
        {
            this_senMsg = &senMsg;
            ximu3::XIMU3_NetworkAnnouncement* const networkAnnouncement = ximu3::XIMU3_network_announcement_new();
            if(ximu3::XIMU3_network_announcement_get_result(networkAnnouncement) != ximu3::XIMU3_ResultOk)
            {
                std::cout << "Unable to open network announcement socket" << std::endl;
                ximu3::XIMU3_network_announcement_free(networkAnnouncement);
                return;
            }

            struct ximu3::XIMU3_NetworkAnnouncementMessages messages = ximu3::XIMU3_network_announcement_get_messages_after_short_delay(networkAnnouncement);
            if(messages.length == 0) 
            {
                std::cout << "No connections available" << std::endl;
                return;
            }
            std::string inputName = "";
            switch(_inputDevice)
            {
                case 0: {inputName.append(DeviceAP0); break;}
                case 1: {inputName.append(DeviceCL1); break;}
                case 2: {inputName.append(DeviceCL2); break;}
                default: 
                {
                    std::cout << "Incorrect device number will connect to the first device found" << std::endl;
                    //return;
                    break;
                } 
            }
            _inputDevice = 0;
            for(int i=0 ; i< messages.length; i++)
            {
                printf("Found %s %s\n", messages.array[i].device_name, messages.array[i].serial_number);
                
                if(messages.array[i].serial_number == inputName)
                {
                    std::cout << "Found needed device at " << i << std::endl;
                    _inputDevice = i;
                    break;       
                }   
            }
            switch(tcp_udp)
            {
                case connection_type::tcpconnection:
                {
                    const ximu3::XIMU3_TcpConnectionInfo connectionInfo = ximu3::XIMU3_network_announcement_message_to_tcp_connection_info(messages.array[_inputDevice]);
                    this_connection = ximu3::XIMU3_connection_new_tcp(connectionInfo);
                    const char* const connectionInfoString = ximu3::XIMU3_tcp_connection_info_to_string(connectionInfo);
                    std::cout << "Connection to " << connectionInfoString << std::endl;
                    break;
                }
                case connection_type::udpconnection:
                {
                    const ximu3::XIMU3_UdpConnectionInfo connectionInfo = ximu3::XIMU3_network_announcement_message_to_udp_connection_info(messages.array[_inputDevice]);
                    this_connection = ximu3::XIMU3_connection_new_udp(connectionInfo);
                    const char* const connectionInfoString = ximu3::XIMU3_udp_connection_info_to_string(connectionInfo);
                    std::cout << "Connection to " << connectionInfoString << std::endl;
                    break;
                }
                default:
                    std::cout << "Incorrect connection type given" << std::endl; 
            }

            if (ximu3::XIMU3_connection_open(this_connection) != ximu3::XIMU3_ResultOk)
            {
                printf("Unable to open connection\n");
                ximu3::XIMU3_connection_free(this_connection);
                return;
            }
                
            printf("Connection successful\n");
            ximu3::XIMU3_network_announcement_free(networkAnnouncement);
            
            // Send command to strobe LED
            const char* const commands[] = { "{\"strobe\":null}" };
            const ximu3::XIMU3_CharArrays responses = ximu3::XIMU3_connection_send_commands(this_connection, commands, 1, 2, 500);
            ximu3::XIMU3_char_arrays_free(responses);    

            this->addInertialCallback(inertialCallback);
            this->addMagnetometerCallback(magnetometerCallback);
            this->addQuaternionCallback(quaternionCallback);
            this->addRotationMatrixCallback(rotationMatrixCallback);
            this->addEulerAnglesCallback(eulerAnglesCallback);
            this->addLinearAccelerationCallback(linearAccelerationCallback);
            
        }

        ~connection(void) noexcept(false)
        {
            ximu3::XIMU3_connection_close(this_connection);
            ximu3::XIMU3_connection_free(this_connection);
        } 

        connection& operator = (const connection&) = delete;
        connection& operator = (connection&&) = delete;

    private:

    template<typename... T, typename Callable>
    static auto wrapCallable(Callable const&) -> void (*)(T..., void*)
    {
        return +[](T... data, void* context)
        {
            (*static_cast<Callable*>(context))(data...);
        };
    }

    std::function<void(ximu3::XIMU3_InertialMessage message)> inertialCallback = [](auto message)
    {
        // printf(TIMESTAMP_FORMAT FLOAT_FORMAT " deg/s" FLOAT_FORMAT " deg/s" FLOAT_FORMAT " deg/s" FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
        //        message.timestamp,
        //        message.gyroscope_x,
        //        message.gyroscope_y,
        //        message.gyroscope_z,
        //        message.accelerometer_x,
        //        message.accelerometer_y,
        //        message.accelerometer_z);
        memcpy(&this_senMsg->inertialMessage , &message, sizeof(message));
        

    };

    std::function<void(ximu3::XIMU3_MagnetometerMessage message)> magnetometerCallback = [](auto message)
    {
        // printf(TIMESTAMP_FORMAT FLOAT_FORMAT " a.u." FLOAT_FORMAT " a.u." FLOAT_FORMAT " a.u.\n",
        //        message.timestamp,
        //        message.x,
        //        message.y,
        //        message.z);
        memcpy(&this_senMsg->magnetometerMessage , &message, sizeof(message));
    };

    std::function<void(ximu3::XIMU3_QuaternionMessage message)> quaternionCallback = [](auto message)
    {
        // printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT "\n",
        //        message.timestamp,
        //        message.w,
        //        message.x,
        //        message.y,
        //        message.z);
        memcpy(&this_senMsg->quaternionMessage , &message, sizeof(message));
    };

    std::function<void(ximu3::XIMU3_RotationMatrixMessage message)> rotationMatrixCallback = [](auto message)
    {
        // printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT "\n",
        //        message.timestamp,
        //        message.xx,
        //        message.xy,
        //        message.xz,
        //        message.yx,
        //        message.yy,
        //        message.yz,
        //        message.zx,
        //        message.zy,
        //        message.zz);
        memcpy(&this_senMsg->rotationMatrixMessage , &message, sizeof(message));
    };

    std::function<void(ximu3::XIMU3_EulerAnglesMessage message)> eulerAnglesCallback = [](auto message)
    {
        // printf(TIMESTAMP_FORMAT FLOAT_FORMAT " deg" FLOAT_FORMAT " deg" FLOAT_FORMAT " deg\n",
        //        message.timestamp,
        //        message.roll,
        //        message.pitch,
        //        message.yaw);
        memcpy(&this_senMsg->eulerMessage , &message, sizeof(message));
    };

    std::function<void(ximu3::XIMU3_LinearAccelerationMessage message)> linearAccelerationCallback = [](auto message)
    {
        // printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
        //        message.timestamp,
        //        message.quaternion_w,
        //        message.quaternion_x,
        //        message.quaternion_y,
        //        message.quaternion_z,
        //        message.acceleration_x,
        //        message.acceleration_y,
        //        message.acceleration_z);
        memcpy(&this_senMsg->lineaccelMessage , &message, sizeof(message));
    };

    uint64_t addInertialCallback(std::function<void(ximu3::XIMU3_InertialMessage)>& callback)
    {
        return XIMU3_connection_add_inertial_callback(this_connection, wrapCallable<ximu3::XIMU3_InertialMessage>(callback), &callback);
    }

    uint64_t addMagnetometerCallback(std::function<void(ximu3::XIMU3_MagnetometerMessage)>& callback)
    {
        return XIMU3_connection_add_magnetometer_callback(this_connection, wrapCallable<ximu3::XIMU3_MagnetometerMessage>(callback), &callback);
    }

    uint64_t addQuaternionCallback(std::function<void(ximu3::XIMU3_QuaternionMessage)>& callback)
    {
        return XIMU3_connection_add_quaternion_callback(this_connection, wrapCallable<ximu3::XIMU3_QuaternionMessage>(callback), &callback);
    }

    uint64_t addRotationMatrixCallback(std::function<void(ximu3::XIMU3_RotationMatrixMessage)>& callback)
    {
        return XIMU3_connection_add_rotation_matrix_callback(this_connection, wrapCallable<ximu3::XIMU3_RotationMatrixMessage>(callback), &callback);
    }

    uint64_t addEulerAnglesCallback(std::function<void(ximu3::XIMU3_EulerAnglesMessage)>& callback)
    {
        return XIMU3_connection_add_euler_angles_callback(this_connection, wrapCallable<ximu3::XIMU3_EulerAnglesMessage>(callback), &callback);
    }

    uint64_t addLinearAccelerationCallback(std::function<void(ximu3::XIMU3_LinearAccelerationMessage)>& callback)
    {
        return XIMU3_connection_add_linear_acceleration_callback(this_connection, wrapCallable<ximu3::XIMU3_LinearAccelerationMessage>(callback), &callback);
    }

};

#endif
