#include <ros/ros.h>
#include "../msg_gen/cpp/include/skynav_robotsimulator/TimedPose2D.h"
#include "../msg_gen/cpp/include/skynav_robotsimulator/RangeArray.h"
#include <geometry_msgs/Pose2D.h>


ros::Publisher pubOdometry;


void subTargetPoseCallback(const skynav_robotsimulator::TimedPose2D::ConstPtr& targetPose) {

    // add optional delay here
    
    pubOdometry.publish(targetPose->pose);

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "simulator");

    ros::NodeHandle n("/robot");
    ros::NodeHandle n_control("/control");

    //pubs
    pubOdometry = n.advertise<geometry_msgs::Pose2D>("odometry", 32);
    ros::Publisher pubSensors = n.advertise<skynav_robotsimulator::RangeArray>("sensors", 1024);

    //subs
    ros::Subscriber subTargetPose = n_control.subscribe("target_pose", 32, subTargetPoseCallback);

    ros::spin();

    return 0;

}
