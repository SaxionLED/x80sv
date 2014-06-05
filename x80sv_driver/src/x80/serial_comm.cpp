
#include <SerialCommInterface.hpp>

#include <ros/ros.h>
#include "DrRobotCommConst.hpp"
#include <stdio.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


using namespace std;


namespace DrRobot
{
    SerialDriver::SerialDriver(const char* serialPort, const long BAUD)
        : CommBase()
    {
        sprintf(_serialPortName, "%s", serialPort);
        _serialfd = -1;
    }

    void SerialDriver::open()
    {
        if (isOpen())
        {
            close();
        }

        _serialfd = ::open(_serialPortName, O_RDWR | O_NONBLOCK | O_NOCTTY);
        if (_serialfd > 0)
        {
            int res = 0;
            struct termios newtio;

            res = tcgetattr(_serialfd, &newtio);
            if (res != 0)
            {
                ROS_ERROR("tcgetattr failed");
            }

            memset(&newtio.c_cc, 0, sizeof (newtio.c_cc));
    //        newtio.c_cflag = BAUD | CS8 | CLOCAL | CREAD;
            newtio.c_cflag = CS8 | CLOCAL | CREAD;
            newtio.c_iflag = IGNPAR;
            newtio.c_oflag = 0;
            newtio.c_lflag = 0;
            newtio.c_cc[VMIN] = 0; //VMIN = 0, VTIME = 0, read will return immediately
            newtio.c_cc[VTIME] = 0;


            res = cfsetispeed(&newtio, B115200);        // to fix minicom start issue, but can often cause massive data discrepancies...  
            if (res != 0)
            {
                ROS_ERROR("tcsetattr failed");
            }

            res = cfsetospeed(&newtio, B115200);
            if (res != 0)
            {
                ROS_ERROR("tcsetattr failed");
            }

            tcflush(_serialfd, TCIFLUSH);
            res = tcsetattr(_serialfd, TCSANOW, &newtio);
            if (res != 0)
            {
                ROS_ERROR("tcsetattr failed");
            }

            ROS_INFO("Serial listener at %s: waiting for robot server, starting receiving...", _serialPortName);
            CommBase::open();
        }
        else
        {
            const char *extra_msg = "";
            switch (errno)
            {
                case EACCES:
                    ROS_ERROR("You probably don't have permission to open the port for reading and writing.");
                    break;
                case ENOENT:
                    ROS_ERROR("The request port does not exit. Was the port name misspelled?");
                    break;
            }
        }
    }

    void SerialDriver::close()
    {
        CommBase::close();

        if (_serialfd > 0)
        {
            ::close(_serialfd);
            _serialfd = -1;
        }
    }


    void SerialDriver::commWorkingThread()
    {
        while (!_stopComm)
        {
            int numbytes = read(_serialfd, _recBuf, sizeof (_recBuf));
            if (numbytes <= 0) //( (_numbytes == -1) && (errno != EAGAIN) && (errno != EWOULDBLOCK) )
            {
                //read erro,
                _comCnt++;
                //printf ("Serial time out\n");
                usleep(10000);
                if (_comCnt > COMM_LOST_TH)
                {
                    ROS_ERROR("Communication is lost, need close all. Serial Port is %s", _serialPortName);

                    _stopComm = true;
                    _eCommState = Disconnected;
                    ::close(_serialfd);
                    _serialfd = -1;
                }
            }
            else
            {
                _comCnt = 0;
                _handler->handleComData(_recBuf, numbytes);
            }
        }
    }


    int SerialDriver::sendCommand(const unsigned char* msg, const int nLen)
    {
        ssize_t retval = 0;
        if (!_stopComm)
        {
            if (_serialfd > 0)
            {
                int origflags = fcntl(_serialfd, F_GETFL, 0);
                fcntl(_serialfd, F_SETFL, origflags & ~O_NONBLOCK);

                retval = write(_serialfd, msg, nLen);
                int fputserrno = errno;
                fcntl(_serialfd, F_SETFL, origflags | O_NONBLOCK);
                errno = fputserrno;
                if (retval != -1) {
                    return retval;
                } else {
                    return -1;
                }

            }
        }
        return -1;
    }

}
