
#ifndef DRROBOTCOMMINTERFACE_H
#define DRROBOTCOMMINTERFACE_H

namespace DrRobot
{

  /*! \enum CommState
   *  Driver communication status
   */
  enum CommState { Disconnected, Connected};

    class CommInterfaceHandler
    {
        public:
            virtual void handleComData(const unsigned char* msg, const int nLen) = 0;
    };

    class CommInterface
    {
        public:
            virtual bool isOpen() = 0;

            virtual void close() = 0;

            virtual void open() = 0;

            virtual CommState getCommunicationState() = 0;
            virtual int getComCnt() = 0;

            virtual int sendCommand(const unsigned char* msg, const int nLen) = 0;

            virtual void registerHandler(CommInterfaceHandler* handler) = 0;
    };

}

#endif

