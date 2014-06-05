#ifndef NETWORK_COMM_INTERFACE_H
#define NETWORK_COMM_INTERFACE_H

#include <CommBase.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define CHAR_BUF_LEN 255
#define MAXBUFLEN 512

namespace DrRobot
{

    class NetworkDriver : public CommBase
    {
        public:
            NetworkDriver();
            virtual ~NetworkDriver();

            virtual void open();
            virtual void close();

            /*! @brief
             *  If the driver is configured as using network communication, this function could open UDP port to connect with robot
             *  and start communication
             * @param[in]   robotIP, should be as dot format, such as "192.168.0.201"
             * @param[in]   portNum, port number, 10001 or 10002
             * @return 0  port opened and starting communication
             *         others  something wrong there
             */
            int openNetwork(const char*  robotIP, const int portNum);
            virtual void commWorkingThread();
            virtual int sendCommand(const unsigned char* msg, const int nLen);

        private:
            int vali_ip(const char* ip_str);
            struct sockaddr_in _addr;
            char _sAddr[INET6_ADDRSTRLEN];
            socklen_t _addr_len;
            int _sockfd;
            fd_set _readfds;
            struct timeval _tv;

            char _robotIP[CHAR_BUF_LEN];        //!< robot main WiFi module IP address, you could get it by manual
            int _portNum;                       //!< robot main WiFi module port number, default is power system on 10001 port, motion system on 10002
            unsigned char _recBuf[MAXBUFLEN];
    };

}

#endif


