#include <ros/ros.h>
#include <x80sv_driver/RangeArray.h>

ros::Publisher pubSensors;

void subSensorIRCallback(const x80sv_driver::RangeArray::ConstPtr& msg)
{
    
    //pubSensors.publish(msg);
}

void subSensorSonarCallback(const x80sv_driver::RangeArray::ConstPtr& msg)
{
    
    //pubSensors.publish(msg);
}

//TODO add Revo callback

//TODO figure out way to check if laser sensor is down/malfunctioning, and publish IR/sonar if so

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "sensor_safety");

    ros::NodeHandle n("/x80sv");
    
    // pubs
    //pubSensors = n.advertise<x80sv_driver::RangeArray>("sensorsafe", 1024);
    
    // subs
    ros::Subscriber subSensorIR = n.subscribe("drrobot_ir", 1024, subSensorIRCallback);
    ros::Subscriber subSensorSonar = n.subscribe("drrobot_sonar", 1024, subSensorSonarCallback);
    

    ros::spin();
    
    return 0;
}

