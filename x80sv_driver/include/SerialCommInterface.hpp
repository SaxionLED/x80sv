
#ifndef SERIAL_COMM_INTERFACE_H
#define SERIAL_COMM_INTERFACE_H

#include <CommBase.hpp>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>

#define CHAR_BUF_LEN 255

#define MAXBUFLEN 512

namespace DrRobot
{

    // Serial implementation
    class SerialDriver : public CommBase
    {
        public:
            SerialDriver(const char* serialPort, const long BAUD);
            virtual void open();
            virtual void close();
            virtual void commWorkingThread();
            virtual int sendCommand(const unsigned char* msg, const int nLen);
        private:
            int _serialfd;
            char _serialPortName[CHAR_BUF_LEN]; //!< serial port name
            unsigned char _recBuf[MAXBUFLEN];
    };

}

#endif

