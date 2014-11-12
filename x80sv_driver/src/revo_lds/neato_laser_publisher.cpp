/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <xv_11_laser_driver/xv11_laser.h>
#include <diagnostic_updater/diagnostic_updater.h>

using namespace xv_11_laser_driver;


class LaserUpdater
{
    public:
        LaserUpdater() : _connected(false), _retries(0)
        {
        }

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
        {
            if (_connected)
            {
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Laser OK");
            }
            else
            {
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Laser not connected!");
            }

            stat.add("Retries", _retries);
        }

        void setConnected(bool connected)
        {
            _connected = connected;
        }

        void incRetries()
        {
            _retries++;
        }

    private:
        bool _connected;
        int _retries;
};


void filter_scan_between_angles(sensor_msgs::LaserScan::Ptr& scan, int from, int to)
{
    for (int i = from; i < to; i++)
    {
        scan->ranges[i] = 0;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "neato_laser_publisher");
    ros::NodeHandle n;
    ros::NodeHandle priv_nh("");

    std::string port;
    int baud_rate;
    std::string frame_id;
    int firmware_number;

    priv_nh.param("port", port, std::string("/dev/ttyLASER"));
    priv_nh.param("baud_rate", baud_rate, 115200);
    priv_nh.param("frame_id", frame_id, std::string("/laser_scanner"));
    priv_nh.param("firmware_version", firmware_number, 2);

    boost::asio::io_service io;
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("laser/scan", 1000);

    diagnostic_updater::Updater updater;
    updater.setHardwareID("none");
    LaserUpdater laserUpdater;
    updater.add("Laser driver", &laserUpdater, &LaserUpdater::produce_diagnostics);

    while (ros::ok())
    {
        updater.update();
        try
        {
            XV11Laser laser(port, baud_rate, firmware_number, io);

            laserUpdater.setConnected(true);
            while (ros::ok())
            {
                sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
                scan->header.frame_id = frame_id;
                scan->header.stamp = ros::Time::now();
                laser.poll(scan);
                // Filter out back of robot:
                filter_scan_between_angles(scan, 0, 180);
                laser_pub.publish(scan);
                updater.update();
            }

            laser.close();
        }
        catch (boost::system::system_error ex)
        {
            laserUpdater.setConnected(false);
            ROS_ERROR("Error instantiating laser object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
            ROS_INFO("Retrying to open laser in 60 seconds");
            ros::Duration(60.0).sleep();
            laserUpdater.incRetries();
        }
    }
}
