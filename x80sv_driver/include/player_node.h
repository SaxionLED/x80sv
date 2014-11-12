#ifndef PLAYERNODE_H
#define PLAYERNODE_H

#include <DrRobotMotionSensorDriver.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

// Generated config file:
#include <x80sv_driver/x80svConfig.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <x80sv_driver/MotorInfo.h>
#include <x80sv_driver/MotorInfoArray.h>
#include <x80sv_driver/RangeArray.h>
#include <sensor_msgs/Range.h>
#include <x80sv_driver/PowerInfo.h>
#include <x80sv_driver/StandardSensor.h>
#include <x80sv_driver/CustomSensor.h>
#include <x80sv_driver/WheelVelocities.h>

#define MOTOR_NUM       6
#define IR_NUM          7
#define US_NUM          3

namespace DrRobot
{

    class PlayerNode
    {
        public:

            ros::NodeHandle node_;

            tf::TransformBroadcaster tf_;

            ros::Publisher motorInfo_pub_;
            ros::Publisher powerInfo_pub_;
            ros::Publisher ir_pub_;
            ros::Publisher sonar_pub_;
            ros::Publisher standardSensor_pub_;
            ros::Publisher customSensor_pub_;

            ros::Subscriber cmd_vel_sub_;
            ros::Subscriber pwm_left_sub_;
            std::string robot_prefix_;

            PlayerNode(); 
            ~PlayerNode();

            int start();
            int stop();
            void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel);
            void wheelVelReceived(const x80sv_driver::WheelVelocities::ConstPtr& wheel_velocities);
            void leftPwmValueReceived(const std_msgs::Int32::ConstPtr& left_pwm);
            void publishOdometry(const x80sv_driver::MotorInfoArray& motorInfo);
            void doUpdate();
            void produce_motion_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

        private:
            CommInterface *_comm_interface;

            MotionSensorDriver* drrobotMotionDriver_;
            MotionSensorDriver* drrobotPowerDriver_;
            struct DrRobotMotionConfig robotConfig1_;
            struct DrRobotMotionConfig robotConfig2_;

            std::string odom_frame_id_;
            float m_x;
            float m_y;
            float m_theta;
            tf::TransformBroadcaster m_odom_broadcaster;
            ros::Publisher m_odom_pub;
            ros::Publisher m_joint_state;

            ros::Subscriber cmd_wheel_velocities_sub_;
            ros::Publisher actual_wheel_velocities_pub_;

            // Dynamic reconfigure part:
            dynamic_reconfigure::Server<x80sv_driver::x80svConfig> _dyn_reconf_server;
            void dynamic_reconfigure_callback(x80sv_driver::x80svConfig &config, uint32_t level);


            struct MotorSensorData motorSensorData_;
            struct RangeSensorData rangeSensorData_;
            struct PowerSensorData powerSensorData_;
            struct StandardSensorData standardSensorData_;
            struct CustomSensorData customSensorData_;

            std::string robotType_;
            std::string robotID_;
            std::string robotIP_;
            bool enable_ir_;
            bool enable_sonar_;
            int encoderOneCircleCnt_;
            double wheelDis_;
            double wheelRadius_;
            int motorDir_;
            double minSpeed_;
            double maxSpeed_;

            int cntNum_;
            double ad2Dis(int adValue);
            void calculateMovementDelta(x80sv_driver::MotorInfo& mtr, int& encoderPrevious, double& movementDelta, double& joint_angle, double scale_factor);
            double encoder2rad(int enc_value);

    };

}


#endif

