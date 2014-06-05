

#include <ros/ros.h>
#include "DrRobotCommConst.hpp"
#include <CommBase.hpp>



using namespace std;


namespace DrRobot
{
    CommBase::CommBase()
    {
        _eCommState = Disconnected;
        _stopComm = true;
        _comCnt = 0;
    }

    CommBase::~CommBase()
    {
        if (isOpen())
        {
            close();
        }
    }

    void CommBase::open()
    {
        _eCommState = Connected;
        _stopComm = false;
        _pCommThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CommBase::commWorkingThread, this)));
    }

    bool CommBase::isOpen()
    {
        if ((_eCommState == Connected) && (!_stopComm))
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    void CommBase::close()
    {
        _stopComm = true;
        _pCommThread->join();
        _eCommState = Disconnected;
    }

    CommState CommBase::getCommunicationState()
    {
        return _eCommState;
    }

    void CommBase::registerHandler(CommInterfaceHandler* handler)
    {
        _handler = handler;
    }

    int CommBase::getComCnt()
    {
        return _comCnt;
    }

}

