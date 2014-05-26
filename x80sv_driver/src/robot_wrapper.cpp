#include <ros/ros.h>
#include <x80sv_driver/RangeArray.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

#include <x80sv_driver/RangeDefined.h>
#include <x80sv_driver/RangeDefinedArray.h>

#include <x80sv_driver/MotorInfoArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <list>
#include <std_msgs/UInt8.h>

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

ros::Publisher pubActuationVelocity, pubOdometry, pubSensors;
ros::Publisher pubSensorData;

int mEncoderPreviousLeft = -1; //TODO confirm left and right are correct
int mEncoderPreviousRight = -1;

int mActuationTime = 0; // time in seconds the current actuation needs to finish
int currentStep = 0; // loop rate steps used to measure time since actuation started

double mWheelRadius = 0.080; //default values as per x80 source file
double mEncoderCircleCount = 756;
double mWheelDistance = 0.305;

ros::Timer mActuationTimer;

void actuationTimeoutCallback(const ros::TimerEvent&) {

    // when calculated time based linear or angular velocity has passed
    mActuationTimer.stop();

    Twist robotActuation;

    // velocity full stop
    robotActuation.linear.x = 0;
    robotActuation.angular.z = 0;
    
    pubActuationVelocity.publish(robotActuation);
}

void subSensorCallback(const x80sv_driver::RangeDefinedArray& msg);


void subSensorSafeCallback(const x80sv_driver::RangeArray::ConstPtr& msg)
{

    // convert range arrays here to RangeDefinedArray (this means adding the angle and distance toward centroid of robot)

    x80sv_driver::RangeDefinedArray rangeDefinedArray;

    for (uint32_t i = 0; i < msg->ranges.size(); i++) {

        x80sv_driver::RangeDefined rangeDefined; // the new range msg, angle/distance needs to be filled in


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

                rangeDefined.angleFromCenter = angleFromCenterDegrees / 180 * M_PI; // save as rads
                rangeDefined.distanceFromCenter = distanceFromCenterMM / 1000; // save as meters
                rangeDefined.yOffsetFromCenter = yOffsetFromCenterMM / 1000;

                rangeDefinedArray.ranges.push_back(rangeDefined);
        } else
            ROS_ERROR("invalid or NYI sensor type found");
    }

    // 
    // pubSensors.publish(rangeDefinedArray);
    subSensorCallback(rangeDefinedArray);
}


//truncate values (in meters) to certain precision
float truncateValue(const float value){
	//return floorf(value*1000)/1000; //mm
	return floorf(value*100)/100; //cm 
}

void subSensorCallback(const x80sv_driver::RangeDefinedArray& msg)
{	// for x80 sonar/IR sensors


    PointCloud pointCloud;

    for (uint i = 0; i < msg.ranges.size(); i++) {

        x80sv_driver::RangeDefined rangeMsg = msg.ranges.at(i);

        if (rangeMsg.range.range > rangeMsg.range.min_range && rangeMsg.range.range < rangeMsg.range.max_range) { // skip if not within limits

            double alpha = rangeMsg.angleFromCenter; // sensor angle + current pose angle
            double distance = rangeMsg.distanceFromCenter + rangeMsg.range.range; // sensor range + offset from center

            double objectX = (cos(alpha) * distance); // add current pose x
            double objectY = (sin(alpha) * distance) + rangeMsg.yOffsetFromCenter; // y

            Point32 p;
            p.x = truncateValue(objectX); 
            p.y = truncateValue(objectY);

            pointCloud.points.push_back(p);
        }
    }

    if (pointCloud.points.size() > 0)
    {

        pointCloud.header.stamp = ros::Time::now();
        pointCloud.header.frame_id = "/base_link";

        // TODO: publish in custom topic of type pointcloud2
        pubSensorData.publish(pointCloud);
    }
}



void subEncoderCallback(const x80sv_driver::MotorInfoArray::ConstPtr& motorInfo)
{

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


    if (averageDistance != 0.0 && deltaAngle != 0.0) { // no need to publish empty poses

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
    pubSensors = n.advertise<sensor_msgs::PointCloud>("sensors", 32);
    pubActuationVelocity = n.advertise<geometry_msgs::Twist>("drrobot_cmd_vel", 32); // only for small, precise turns 

    //subs
    ros::Subscriber subSensorSafe = n.subscribe("sensorsafe", 32, subSensorSafeCallback);
    //TODO IMU, optionally a battery voltage warning
    ros::Subscriber subEncoder = n.subscribe("drrobot_motor", 256, subEncoderCallback);


    // read x80 parameter file
    n.getParam("x80_config/WheelRadius", mWheelRadius);
    n.getParam("x80_config/EncoderCircleCnt", mEncoderCircleCount);
    n.getParam("x80_config/WheelDistance", mWheelDistance);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;

}

