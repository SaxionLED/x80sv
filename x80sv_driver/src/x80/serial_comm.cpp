

#include <DrRobotMotionSensorDriver.hpp>
#include <ros/ros.h>
#include "DrRobotCommConst.hpp"



using namespace std;


namespace DrRobot_MotionSensorDriver
{
    DrRobotSerialDriver::DrRobotSerialDriver()
    {
    }

    int DrRobotSerialDriver::openSerial(const char* serialPort, const long BAUD)
    {

        if (portOpen())
            close();

        _robotConfig->commMethod = Serial;

        sprintf(_robotConfig->serialPortName, "%s", serialPort);

        _serialfd = ::open(_robotConfig->serialPortName, O_RDWR | O_NONBLOCK | O_NOCTTY);
        //_serialfd = ::open("/dev/ttyS0", O_RDWR | O_NONBLOCK | O_NOCTTY);
        if (_serialfd > 0)
        {
            int res = 0;
            struct termios newtio;

            res = tcgetattr(_serialfd, &newtio);
            if (res != 0)
            {
                ROS_ERROR("tcgetattr failed");
                return -1;
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
                return -1;
            }

            res = cfsetospeed(&newtio, B115200);
            if (res != 0)
            {
                ROS_ERROR("tcsetattr failed");
                return -1;
            }

            tcflush(_serialfd, TCIFLUSH);
            res = tcsetattr(_serialfd, TCSANOW, &newtio);
            if (res != 0)
            {
                ROS_ERROR("tcsetattr failed");
                return -1;
            }

            ROS_INFO("Serial listener at %s: waiting for robot server, starting receiving...", _robotConfig->serialPortName);
            _eCommState = Connected;
            _stopComm = false;
            _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DrRobotMotionSensorDriver::commWorkingThread, this)));
            return 0;
        }
        else
        {
            const char *extra_msg = "";
            switch (errno) {
                case EACCES:
                    extra_msg = "You probably don't have permission to open the port for reading and writing.\n";
                    debug_ouput(extra_msg);
                    break;
                case ENOENT:
                    extra_msg = "The request port does not exit. Was the port name misspelled?\n";
                    debug_ouput(extra_msg);
                    break;
            }
            _stopComm = true;
            _eCommState = Disconnected;
            return errno;
        }
    }


    void DrRobotSerialDriver::close()
    {
        DrRobotMotionSensorDriver::close();

        if (_serialfd > 0)
        {
            ::close(_serialfd);
            _serialfd = -1;
        }
    }


    void DrRobotSerialDriver::commWorkingThread()
    {
        while (!_stopComm)
        {
                _numbytes = read(_serialfd, _recBuf, sizeof (_recBuf));
                if (_numbytes <= 0) //( (_numbytes == -1) && (errno != EAGAIN) && (errno != EWOULDBLOCK) )
                {
                    //read erro,
                    _comCnt++;
                    //printf ("Serial time out\n");
                    usleep(10000);
                    if (_comCnt > COMM_LOST_TH) {
                        ROS_ERROR("Communication is lost, need close all. Serial Port is %s", _robotConfig->serialPortName);
                        _stopComm = true;
                        _eCommState = Disconnected;
                        ::close(_serialfd);
                        _serialfd = -1;
                    }

                } else {
    #ifdef DEBUG_ERROR
                    printf("listener: packet is %d bytes long\n", _numbytes);
    #endif
                    _comCnt = 0;
                    handleComData(_recBuf, _numbytes);
                }



                /*
               struct pollfd ufd[1];
               int retval;
               int timeout = 0;
               ufd[0].fd = _serialfd;
               ufd[0].events = POLLIN;
               //below will block to wait event

               //if (timeout == 0)
               //  timeout = -1;



               retval = poll(ufd, 1, timeout);
               if (retval < 0)
               {
                 //poll fialed -- error
                 printf("DrRobot Serial Communication error, eroor no is %d: %s", errno, strerror(errno));
                _stopComm = true;
                _eCommState = Disconnected;
                ::close(_sockfd);
                _sockfd = -1;
                return;
               }
               else if (retval  == 0)
               {
                 //timeout,
                 _comCnt ++;
                 printf ("Serial time out\n");
                 usleep(10000);
                 if (_comCnt > COMM_LOST_TH)
                 {
                   printf("communication is lost, need close all\n");
                   _stopComm = true;
                   _eCommState = Disconnected;
                   ::close(_serialfd);
                   _serialfd = -1;
                 }
               }
               else
               {
                 _numbytes = read(_serialfd,_recBuf, sizeof(_recBuf));
                 if ( (_numbytes == -1) && (errno != EAGAIN) && (errno != EWOULDBLOCK) )
                 {
                   //read erro,
                   _comCnt ++;
                 }
                 else
                 {
         #ifdef DEBUG_ERROR
                printf("listener: packet is %d bytes long\n", _numbytes);
          #endif
                _comCnt = 0;
                handleComData(_recBuf,_numbytes);
                 }
               }
                 */

        }
        return;
    }


    int DrRobotSerialDriver::sendCommand(const unsigned char* msg, const int nLen)
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
