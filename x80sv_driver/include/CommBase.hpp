#ifndef COMMBASE_H
#define COMMBASE_H

#include <DrRobotCommInterface.hpp>
#include <boost/thread/thread.hpp>


namespace DrRobot
{

    class CommBase : public CommInterface
    {
        public:
            // Get communication state.
            virtual CommState getCommunicationState();
            virtual bool isOpen();
            virtual void open();
            virtual void close();
            CommBase();
            virtual ~CommBase();
            virtual void commWorkingThread() = 0;

            virtual void registerHandler(CommInterfaceHandler* handler);

            virtual int getComCnt();

        protected:
            bool _stopComm;
            int _comCnt;
            CommState _eCommState;
            CommInterfaceHandler* _handler;
        private:
            boost::shared_ptr<boost::thread> _pCommThread;
    };

}

#endif

