


#include <NetworkCommInterface.hpp>
#include <ros/ros.h>
#include "DrRobotCommConst.hpp"


using namespace std;
using namespace DrRobot;

namespace DrRobot
{

    NetworkDriver::NetworkDriver()
    {
        // TODO: fill port and ip

        bzero(&_addr, sizeof (_addr));
        _addr.sin_family = AF_INET;
        _addr.sin_port = htons(_portNum);
        _addr_len = sizeof _addr;

        _sockfd = socket(AF_INET, SOCK_DGRAM, 0);

        _tv.tv_sec = 0;
        _tv.tv_usec = 200; //200us ?


        // _robotConfig->portNum = DEFAULT_PORT;
        // sprintf(_robotConfig->robotIP, "192.168.0.201");
    }

    NetworkDriver::~NetworkDriver()
    {
    }

    int NetworkDriver::vali_ip(const char* ip_str)
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


    int NetworkDriver::openNetwork(const char* robotIP, const int portNum)
    {
        char temp[512];
        //check the parameter first
        if (portNum <= 0)
        {
            ROS_ERROR("Port < 0");
            return -1;
        }

        if (vali_ip(robotIP) == 1)
        {
            ROS_ERROR("DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", robotIP);
            return -2;
        }
        _portNum = portNum;

        sprintf(_robotIP, "%s", robotIP);
        bzero(&_addr, sizeof (_addr));
        _addr.sin_family = AF_INET;
        _addr.sin_port = htons(_portNum);

        if (inet_aton(_robotIP, &_addr.sin_addr) == 0)
        {
            ROS_ERROR("DrRobot Motion/Sensor Driver Error Message: invalid IP address: %s\n", _robotIP);
            return -3;
        }

        _stopComm = false;
        int numbytes = 0; // TODO: sendAck();
        if (numbytes < 0)
        {
            _stopComm = true;
            perror("sendto");
            return -4;
        }

        ROS_INFO("TCP listener: waiting for robot server, starting receiving...");
        CommBase::open();
        return 0;
    }


    void NetworkDriver::open()
    {
    }

    void NetworkDriver::close()
    {
        CommBase::close();

        //for UDP , do we need close socket?

        if (_sockfd > 0)
        {
            ::close(_sockfd);
            _sockfd = -1;
        }
    }

//communication thread here

    void NetworkDriver::commWorkingThread()
    {
        while (!_stopComm)
        {
            FD_ZERO(&_readfds);
            FD_SET(_sockfd, &_readfds);
            select(_sockfd + 1, &_readfds, NULL, NULL, &_tv);
            if (FD_ISSET(_sockfd, &_readfds))
            {
                int numbytes;
                if ((numbytes = recvfrom(_sockfd, _recBuf, MAXBUFLEN - 1, 0, (struct sockaddr *) &_addr, &_addr_len)) == -1) 
                {
                    perror("recvfrom");
                    return;
                }
#ifdef DEBUG_ERROR
                printf("listener: packet is %d bytes long\n", _numbytes);
#endif
                _comCnt = 0;
                _handler->handleComData(_recBuf, numbytes);
            }
            else
            {
                _comCnt++;

                usleep(10000); //10ms
                if (_comCnt > COMM_LOST_TH)
                {
                    ROS_ERROR("Communication is lost, need close all. IP address %s, Port: %d", _robotIP, _portNum);
                    _stopComm = true;                    
                    return;
                }
            }
        }
        return;
    }


    int NetworkDriver::sendCommand(const unsigned char* msg, const int nLen)
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

