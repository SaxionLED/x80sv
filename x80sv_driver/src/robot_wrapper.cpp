#include <ros/ros.h>
#include <skynav_msgs/RangeArray.h>
#include <sensor_msgs/Range.h>
#include <skynav_msgs/RangeDefined.h>
#include <skynav_msgs/RangeDefinedArray.h>
#include <x80sv_driver/MotorInfoArray.h>
#include <skynav_msgs/TimedPose.h>
#include <geometry_msgs/Twist.h>
#include <list>
#include <std_msgs/UInt8.h>

using namespace std;
using namespace geometry_msgs;

ros::Publisher pubActuationPose, pubActuationVelocity, pubOdometry, pubSensors;

int mEncoderPreviousLeft = -1; //TODO confirm left and right are correct
int mEncoderPreviousRight = -1;

int mActuationTime = 0; // time in seconds the current actuation needs to finish
int currentStep = 0; // loop rate steps used to measure time since actuation started

double mWheelRadius = 0.080; //default values as per x80 source file
double mEncoderCircleCount = 756;
double mWheelDistance = 0.305;

ros::Timer mActuationTimer;

list<skynav_msgs::TimedPose> mSubPoseList;

void actuationTimeoutCallback(const ros::TimerEvent&) {

    // when calculated time based linear or angular velocity has passed
    mActuationTimer.stop();

    Twist robotActuation;

    // velocity full stop
    robotActuation.linear.x = 0;
    robotActuation.angular.z = 0;
    
    pubActuationVelocity.publish(robotActuation);
}

void subSensorSafeCallback(const skynav_msgs::RangeArray::ConstPtr& msg) {

    // convert range arrays here to RangeDefinedArray (this means adding the angle and distance toward centroid of robot)

    skynav_msgs::RangeDefinedArray rangeDefinedArray;

    for (uint32_t i = 0; i < msg->ranges.size(); i++) {

        skynav_msgs::RangeDefined rangeDefined; // the new range msg, angle/distance needs to be filled in


        if (msg->ranges.at(i).radiation_type == sensor_msgs::Range::ULTRASOUND) {


            rangeDefined.range = msg->ranges.at(i); // copy old sensor data

            // get sensor id number from the header.frame_id string
            string frame_id = msg->ranges.at(i).header.frame_id;

            uint8_t sensorID = frame_id.at(14) - 48; // drrobot_sonar_# where # is the number we want, the prefix is defined in the drrobot_player.cpp file

            float angleFromCenterDegrees = 0;
            float distanceFromCenterMM = 0;
            float yOffsetFromCenterMM = 0;

            switch (sensorID) { // add angle and distance from centroid in this switchcase

                case 0: // front left
                    angleFromCenterDegrees = 45; //in degrees
                    distanceFromCenterMM = 194.08; // in mm
                    yOffsetFromCenterMM = -2.36;
                    break;
                case 1: // middle
                    angleFromCenterDegrees = 0;
                    distanceFromCenterMM = 191.50;
                    yOffsetFromCenterMM = 0;
                    break;
                case 2: // front right
                    angleFromCenterDegrees = -45;
                    distanceFromCenterMM = 194.08;
                    yOffsetFromCenterMM = 2.36;
                    break;
                default:
                    ROS_ERROR("invalid SONAR sensor ID found: %u", sensorID);
                    return;
            }

            rangeDefined.angleFromCenter = angleFromCenterDegrees / 180 * M_PI; // save as rads
            rangeDefined.distanceFromCenter = distanceFromCenterMM / 1000; // save as meters
            rangeDefined.yOffsetFromCenter = yOffsetFromCenterMM / 1000; // as meters
            
            rangeDefinedArray.ranges.push_back(rangeDefined);

            
            
            
            
            

        } else if (msg->ranges.at(i).radiation_type == sensor_msgs::Range::INFRARED) {

            rangeDefined.range = msg->ranges.at(i); // copy old sensor data

            // get sensor id number from the header.frame_id string
            string frame_id = msg->ranges.at(i).header.frame_id;

            uint8_t sensorID = frame_id.at(11) - 48; // drrobot_ir_# where # is the number we want, the prefix is defined in the drrobot_player.cpp file

            float angleFromCenterDegrees = 0;
            float distanceFromCenterMM = 0;
            float yOffsetFromCenterMM = 0;

            switch (sensorID) { // add angle and distance from centroid in this switchcase

                    // angle based on front (so straight ahead = 0 degrees)

                case 0: // front left
                    angleFromCenterDegrees = 33.75; //in degrees
                    distanceFromCenterMM = 190.82; // in mm
                    yOffsetFromCenterMM = -6.57;
                    break;
                case 1: // front left mid
                    angleFromCenterDegrees = 11.25;
                    distanceFromCenterMM = 186.76;
                    yOffsetFromCenterMM = 4.34;
                    break;
                case 2: // front right mid
                    angleFromCenterDegrees = -11.25;
                    distanceFromCenterMM = 186.76;
                    yOffsetFromCenterMM = -4.34;
                    break;
                case 3: // front right
                    angleFromCenterDegrees = -33.75;
                    distanceFromCenterMM = 190.82;
                    yOffsetFromCenterMM = 6.57;
                    break;
                case 4: // right side
                    angleFromCenterDegrees = -90;
                    distanceFromCenterMM = 121;
                    yOffsetFromCenterMM = 0;
                    break;
                case 5: // back side
                    angleFromCenterDegrees = -180;
                    distanceFromCenterMM = 186;
                    yOffsetFromCenterMM = 0;
                    break;
                case 6: // left side
                    angleFromCenterDegrees = 90;
                    distanceFromCenterMM = 121;
                    yOffsetFromCenterMM = 0;
                    break;
                default:
                    ROS_ERROR("invalid IR sensor ID found: %u", sensorID);
                    return;
            }

//            if (sensorID == 0) { //////// TEMP HACK !!!!!!!!! to only process sensor 0
                rangeDefined.angleFromCenter = angleFromCenterDegrees / 180 * M_PI; // save as rads
                rangeDefined.distanceFromCenter = distanceFromCenterMM / 1000; // save as meters
                rangeDefined.yOffsetFromCenter = yOffsetFromCenterMM / 1000;

                rangeDefinedArray.ranges.push_back(rangeDefined);
//            }
        } else
            ROS_ERROR("invalid or NYI sensor type found");
    }

    pubSensors.publish(rangeDefinedArray);
}

void subTargetPoseCallback(const skynav_msgs::TimedPose::ConstPtr& targetPose) {
    
    
    if (targetPose->pose.position.x != 0 && targetPose->pose.orientation.z != 0) {
        ROS_ERROR("not allowed to move forward and turn at the same time");
        return;
    }

    if (targetPose->actuationSeconds > 32) {
        ROS_ERROR("X80SV cannot accept time periods larger than 32s");
        return;
    }

    if (targetPose->pose.position.x / targetPose->actuationSeconds > 1) {
        ROS_ERROR("Safety limit! Attempted to move faster than 1m/s (attempted speed: %f)", targetPose->pose.position.x / targetPose->actuationSeconds);
        return;
    }

    skynav_msgs::TimedPose adjustedPose; // new object because targetPose is read-only and needs to be because of ROS
    adjustedPose = (*targetPose);

    //    if (targetPose->pose.theta > 0) {
    //
    //        adjustedPose.pose.theta += (M_PI / 180) * 2; // because the x80 has difficult with small turns, roughly below 20 degrees, we add 2 degrees. In practice this makes turns > 5 and < 20 more accurate
    //    } else if (targetPose->pose.x > 0.2) {      // 0.2 is an estimation
    //        adjustedPose.pose.x -= (adjustedPose.pose.x * 0.06);    // with current PID settings, the x80 seems to always move 5cm too far, try and prevent this
    //    }

    pubActuationPose.publish(adjustedPose);


    // uncomment the follow lines to use velocity instead of position control
    //    Twist twist;
    //    twist.linear.x = targetPose->pose.x / targetPose->actuationSeconds;
    //    twist.angular.z = ((mWheelDistance / 2) * targetPose->pose.theta) / targetPose->actuationSeconds;
    //
    //    pubActuationVelocity.publish(twist);
    //
    //    mActuationTimer = n.createTimer(ros::Duration(targetPose->actuationSeconds), actuationTimeoutCallback);

}

void subEncoderCallback(const x80sv_driver::MotorInfoArray::ConstPtr& motorInfo) {

    double movementDelta0 = 0, movementDelta1 = 0;

    const uint encoderMax = 32768;

    x80sv_driver::MotorInfo mtr0 = motorInfo->motorInfos.at(0);
    x80sv_driver::MotorInfo mtr1 = motorInfo->motorInfos.at(1);

    //    ROS_INFO("Encoder values: %u, %u", mtr0.encoder_pos, mtr1.encoder_pos);

    { //motor 0

        if (mEncoderPreviousLeft != -1) {
            if (mEncoderPreviousLeft < 5000 && mtr0.encoder_pos > 25000) {
                mEncoderPreviousLeft = mtr0.encoder_pos - mEncoderPreviousLeft;
            } else if (mEncoderPreviousLeft > 25000 && mtr0.encoder_pos < 5000) {
                mEncoderPreviousLeft = mtr0.encoder_pos + (encoderMax - mEncoderPreviousLeft);
            }

            int encoderDelta0 = mEncoderPreviousLeft - mtr0.encoder_pos;

            movementDelta0 = (((mWheelRadius * 2 * M_PI) / mEncoderCircleCount) * encoderDelta0);
        }

        mEncoderPreviousLeft = mtr0.encoder_pos;
    }


    { //motor 1

        if (mEncoderPreviousRight != -1) {
            if (mEncoderPreviousRight < 5000 && mtr1.encoder_pos > 25000) {
                mEncoderPreviousRight = mtr1.encoder_pos - mEncoderPreviousRight;
            } else if (mEncoderPreviousRight > 25000 && mtr0.encoder_pos < 5000) {
                mEncoderPreviousRight = mtr0.encoder_pos + (encoderMax - mEncoderPreviousRight);
            }

            int encoderDelta1 = mtr1.encoder_pos - mEncoderPreviousRight;

            movementDelta1 = (((mWheelRadius * 2 * M_PI) / mEncoderCircleCount) * encoderDelta1);
        }

        mEncoderPreviousRight = mtr1.encoder_pos;
    }

    // now calculate the angle relative to the last position (which is always 0,0)

    double averageDistance = (movementDelta1 + movementDelta0) / 2; // average distance between 2 wheels = actual distance from center
    double deltaAngle = atan((movementDelta1 - movementDelta0) / mWheelDistance); // difference in angle from last encoder values, 0.285 is distance between wheel centers in m


    if (averageDistance != 0.0 || deltaAngle != 0.0) { // no need to publish empty poses

        Pose poseOut; // not a pose, x = distance travelleed and theta = delta angle
        poseOut.position.x = averageDistance;
        poseOut.orientation.z = deltaAngle;

        pubOdometry.publish(poseOut);
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "robot_wrapper");

    ros::NodeHandle n("/x80sv");
    ros::NodeHandle n_control("/control");

    //pubs
    pubOdometry = n.advertise<geometry_msgs::Pose>("odometry", 32);
    pubSensors = n.advertise<skynav_msgs::RangeDefinedArray>("sensors", 1024);
    pubActuationPose = n.advertise<skynav_msgs::TimedPose>("drrobot_cmd_pose", 32); // drrobot specific topic
    pubActuationVelocity = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 32); // only for small, precise turns 

    //subs
    ros::Subscriber subSensorSafe = n.subscribe("sensorsafe", 1024, subSensorSafeCallback);
    //TODO IMU, optionally a battery voltage warning
    ros::Subscriber subEncoder = n.subscribe("drrobot_motor", 256, subEncoderCallback);
    ros::Subscriber subTargetPose = n_control.subscribe("target_motion", 32, subTargetPoseCallback);

    // read x80 parameter file
    n.getParam("x80_config/WheelRadius", mWheelRadius);
    n.getParam("x80_config/EncoderCircleCnt", mEncoderCircleCount);
    n.getParam("x80_config/WheelDistance", mWheelDistance);

    ros::Rate loop_rate(100);

    while (ros::ok()) {

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}

