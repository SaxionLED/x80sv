#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


using namespace sensor_msgs;
using namespace std;

ros::Publisher pubLaserFiltered;


void subLaserCallback(const LaserScan::ConstPtr& scan) {

	LaserScan scan_filtered;

	scan_filtered.header.frame_id = scan->header.frame_id;
	scan_filtered.header.stamp = scan->header.stamp;

	scan_filtered.angle_min = scan->angle_min;
	scan_filtered.angle_max = scan->angle_max;
	scan_filtered.angle_increment = scan->angle_increment;
	scan_filtered.scan_time = scan->scan_time;
	scan_filtered.range_min = scan->range_min;
	scan_filtered.range_max = scan->range_max;	


	for(uint i = 0; i < scan->ranges.size(); ++i)	{

		if( scan->ranges.at(i) > (scan->range_max - 0.1) || scan->ranges.at(i) < scan->range_min)	{	
			scan_filtered.ranges.push_back( 0 );	// add 0 if outside limit or it ruins the rest of the angles (relient on array size)
			scan_filtered.intensities.push_back( 0 );
			continue;
		}

		scan_filtered.ranges.push_back( scan->ranges.at(i) );
		scan_filtered.intensities.push_back( scan->intensities.at(i) );
	}
    
    pubLaserFiltered.publish(scan_filtered);

}



int main(int argc, char **argv) {

    ros::init(argc, argv, "laser_filter");

    ros::NodeHandle n("/x80sv");

    //pubs
    pubLaserFiltered = n.advertise<LaserScan>("laser/scan_filtered", 1000);

    //subs
    ros::Subscriber subLaser = n.subscribe("laser/scan", 1000, subLaserCallback);

    ros::spin();

    return 0;

}
