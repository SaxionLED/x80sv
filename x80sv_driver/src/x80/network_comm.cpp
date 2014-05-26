


#include <DrRobotMotionSensorDriver.hpp>
#include <ros/ros.h>
#include "DrRobotCommConst.hpp"


//#define DEBUG_ERROR           //set printf out error message
#undef DEBUG_ERROR


using namespace std;
using namespace DrRobot_MotionSensorDriver;

namespace DrRobot_MotionSensorDriver
{


    int DrRobotNetworkDriver::vali_ip(const char* ip_str)
    {
        unsigned int n1, n2, n3, n4;
        if (sscanf(ip_str, "%u.%u.%u.%u", &n1, &n2, &n3, &n4) != 4) return 1;
        if ((n1 != 0) && (n1 <= 255) && (n2 <= 255) && (n3 <= 255) && (n4 <= 255)) {
            char buf[64];
            sprintf(buf, "%u.%u.%u.%u", n1, n2, n3, n4);
            if (strcmp(buf, ip_str)) return 1;
            return 0;
        }

        return 1;
    }


    int DrRobotNetworkDriver::openNetwork(const char* robotIP, const int portNum)
    {
        char temp[512];
        //check the parameter first
        if (portNum <= 0) {

            debug_ouput(temp);
            return -1;
        }

        if (vali_ip(robotIP) == 1) {
            sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", robotIP);
            debug_ouput(temp);
            return -2;
        }
        _robotConfig->commMethod = Network;
        _robotConfig->portNum = portNum;

        sprintf(_robotConfig->robotIP, "%s", robotIP);
        bzero(&_addr, sizeof (_addr));
        _addr.sin_family = AF_INET;
        _addr.sin_port = htons(_robotConfig->portNum);

        if (inet_aton(_robotConfig->robotIP, &_addr.sin_addr) == 0) {
            sprintf(temp, "DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", _robotConfig->robotIP);
            debug_ouput(temp);
            return -3;
        }

        _stopComm = false;
        _numbytes = sendAck();
        if (_numbytes < 0) {
            _stopComm = true;
            perror("sendto");
            return -4;
        }

        ROS_INFO("TCP listener: waiting for robot server, starting receiving...");
        _eCommState = Connected;

        _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DrRobotMotionSensorDriver::commWorkingThread, this)));
        return 0;
    }


    void DrRobotNetworkDriver::close()
    {
        DrRobotMotionSensorDriver::close();

        //for UDP , do we need close socket?

        if (_sockfd > 0)
        {
            ::close(_sockfd);
            _sockfd = -1;
        }
    }


    int DrRobotNetworkDriver::sendCommand(const unsigned char* msg, const int nLen)
    {
        ssize_t retval = 0;
        if (!_stopComm)
        {
            if (_sockfd > 0)
            {
                int retval = sendto(_sockfd, msg, nLen, 0, (const struct sockaddr *) &_addr, sizeof (_addr));
                if (retval > 0) {
                    return retval;
                } else {
                    perror("sendto");
                    return -1;
                }
            }
        }
        return -1;
    }

}

