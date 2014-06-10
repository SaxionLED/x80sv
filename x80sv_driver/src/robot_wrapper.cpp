#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud.h>

#include <x80sv_driver/RangeArray.h>
#include <x80sv_driver/RangeDefined.h>
#include <x80sv_driver/RangeDefinedArray.h>

#include <list>
#include <std_msgs/UInt8.h>

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

ros::Publisher pubSensors;
ros::Publisher pubSensorData;


void subSensorCallback(const x80sv_driver::RangeDefinedArray& msg);


void subSensorSafeCallback(const x80sv_driver::RangeArray::ConstPtr& msg)
{

    // convert range arrays here to RangeDefinedArray (this means adding the angle and distance toward centroid of robot)

    x80sv_driver::RangeDefinedArray rangeDefinedArray;

    for (uint32_t i = 0; i < msg->ranges.size(); i++) {

        x80sv_driver::RangeDefined rangeDefined; // the new range msg, angle/distance needs to be filled in


        if (msg->ranges.at(i).radiation_type == sensor_msgs::Range::ULTRASOUND)
        {


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
        }
        else if (msg->ranges.at(i).radiation_type == sensor_msgs::Range::INFRARED)
        {

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
        }
        else
        {
            ROS_ERROR("invalid or NYI sensor type found");
        }
    }

    // 
    subSensorCallback(rangeDefinedArray);
}


//truncate values (in meters) to certain precision
float truncateValue(const float value)
{
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
        // pubSensorData.publish(pointCloud);
        //// pubSensors.publish(rangeDefinedArray);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_wrapper");
    ros::NodeHandle n("/x80sv");

    pubSensors = n.advertise<sensor_msgs::PointCloud>("sensors", 32);

    ros::Subscriber subSensorSafe = n.subscribe("sensorsafe", 32, subSensorSafeCallback);
    //TODO IMU, optionally a battery voltage warning

    ros::Rate loop_rate(100);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}

