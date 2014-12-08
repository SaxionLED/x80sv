
#include <player_node.h>

#include <SerialCommInterface.hpp>
#include <NetworkCommInterface.hpp>

#include <x80sv_driver/MotorInfoArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>


using namespace std;

namespace DrRobot
{

    double PlayerNode::ad2Dis(int adValue)
    {
        double temp = 0;
        double irad2Dis = 0;

        if (adValue <= 0)
        {
            temp = -1;
        }
        else
        {
            temp = 21.6 / ((double) adValue * 3 / 4096 - 0.17);
        }

        if ((temp > 80) || (temp < 0))
        {
            irad2Dis = 0.81;
        }
        else if ((temp < 10) && (temp > 0))
        {
            irad2Dis = 0.09;
        }
        else
        {
            irad2Dis = temp / 100;
        }

        return irad2Dis;
    }

    // Convert encoder ticks to radians:
    double PlayerNode::encoder2rad(int enc_value)
    {
        return (enc_value * 2 * M_PI) / encoderOneCircleCnt_;
    }


    PlayerNode::PlayerNode()
    {
        ros::NodeHandle private_nh("");

        robotID_ = "drrobot1";
        private_nh.getParam("x80_config/RobotID", robotID_);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        robotType_ = "X80";
        private_nh.getParam("x80_config/RobotType", robotType_);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        std::string robotCommMethod_;
        robotCommMethod_ = "Network";
        private_nh.getParam("x80_config/RobotCommMethod", robotCommMethod_);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        std::string robotSerialPort_;
        robotSerialPort_ = "/dev/ttyS0";
        private_nh.getParam("x80_config/RobotSerialPort", robotSerialPort_);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        enable_ir_ = true;
        private_nh.getParam("x80_config/Enable_IR", enable_ir_);
        if (enable_ir_)
            ROS_INFO("I get Enable_IR: true");
        else
            ROS_INFO("I get Enable_IR: false");


        enable_sonar_ = true;
        private_nh.getParam("x80_config/Enable_US", enable_sonar_);
        if (enable_sonar_)
            ROS_INFO("I get Enable_US: true");
        else
            ROS_INFO("I get Enable_US: false");

        motorDir_ = 1;
        private_nh.getParam("x80_config/MotorDir", motorDir_);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);

        wheelRadius_ = 0.085;
        private_nh.getParam("x80_config/WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        // Wheel distance is 28.2 cm +/- 0.2 cm
        wheelDis_ = 0.283;
        private_nh.getParam("x80_config/WheelDistance", wheelDis_);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        minSpeed_ = 0.1;
        private_nh.getParam("x80_config/MinSpeed", minSpeed_);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        maxSpeed_ = 1.0;
        private_nh.getParam("x80_config/MaxSpeed", maxSpeed_);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        encoderOneCircleCnt_ = 756;
        private_nh.getParam("x80_config/EncoderCircleCnt", encoderOneCircleCnt_);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

        if (robotCommMethod_ == "Network")
        {
            _comm_interface = new NetworkDriver();
        }
        else
        {
            _comm_interface = new SerialDriver(robotSerialPort_.c_str(), 115200);
        }

        if (robotType_ == "Jaguar")
        {
            robotConfig1_.boardType = Jaguar;
        }
        else if (robotType_ == "X80")
        {
            robotConfig1_.boardType = X80SV;
            robotConfig2_.boardType = X80SV;
        }

        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<x80sv_driver::MotorInfoArray>("drrobot_motor", 1);
        powerInfo_pub_ = node_.advertise<x80sv_driver::PowerInfo>("drrobot_powerinfo", 1);
        ir_pub_ = node_.advertise<x80sv_driver::RangeArray>("drrobot_ir", 1);
        sonar_pub_ = node_.advertise<x80sv_driver::RangeArray>("drrobot_sonar", 1);
        standardSensor_pub_ = node_.advertise<x80sv_driver::StandardSensor>("drrobot_standardsensor", 1);
        customSensor_pub_ = node_.advertise<x80sv_driver::CustomSensor>("drrobot_customsensor", 1);
        actual_wheel_velocities_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("actual_wheel_velocities", 1);
        requested_wheel_velocities_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("requested_wheel_velocities", 1);
        smoothed_wheel_velocities_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("filtered_wheel_velocities", 1);
	actual_wheel_positions_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("actual_wheel_positions", 1);

        m_odom_pub = node_.advertise<nav_msgs::Odometry>("odom", 1);
        m_joint_state = node_.advertise<sensor_msgs::JointState>("joint_states", 1);

        drrobotPowerDriver_ = new MotionSensorDriver(*_comm_interface);
        drrobotMotionDriver_ = new MotionSensorDriver(*_comm_interface);
        drrobotPowerDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig2_);
        cntNum_ = 0;

        // Keep track of robot:
        m_x = 0;
        m_y = 0;
        m_theta = 0;

        // Register dynamic reconfigure:
        dynamic_reconfigure::Server<x80sv_driver::x80svConfig>::CallbackType dyn_reconf_callback(
            boost::bind(&PlayerNode::dynamic_reconfigure_callback, this, _1, _2));
        _dyn_reconf_server.setCallback(dyn_reconf_callback);
    }


    PlayerNode::~PlayerNode()
    {
    }


    int PlayerNode::start()
    {
        _comm_interface->open();

        // TODO: why subscribe here?
        cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&PlayerNode::cmdVelReceived, this, _1));
        // pwm_left_sub_ = node_.subscribe<std_msgs::Int32>("/pwm_left", 1, boost::bind(&PlayerNode::leftPwmValueReceived, this, _1));
        cmd_wheel_velocities_sub_ = node_.subscribe<x80sv_driver::WheelVelocities>("cmd_wheel_velocities", 1, boost::bind(&PlayerNode::wheelVelReceived, this, _1));


        // Gain tuning 12 november 10:49:
        // Kp=22, Ki=170, Kd=0 yields reasonable results. Not optimal, but better
        // Kp=10, Ki=170, Kd=0 yields better results. Still not optimal.

        // Gain tuning on 3 december 2014 by Abeje and Windel:
        // Kp=20 and Ki=0 and Kd=0, not using of Kd and Ki yields ok results.
        // Kp =25 is at the margin of stability.

        // Wbo at 8 december 2014:
        // Gain kp=20 is sometimes shaky

        // only needed when using velocity (1, 0, 170))
        drrobotMotionDriver_->setMotorVelocityCtrlPID(0, 15, 0, 0); // channel, p, d, i
        drrobotMotionDriver_->setMotorVelocityCtrlPID(1, 15, 0, 0);

        // PID default is 1000, 5, 10000 (taken from C# src)
        drrobotMotionDriver_->setMotorPositionCtrlPID(0, 500, 5, 10000);
        drrobotMotionDriver_->setMotorPositionCtrlPID(1, 500, 5, 10000);
        // 500, 2, 510 has smoother motion but does not take friction in account

        return (0);
    }

    // 
    void PlayerNode::dynamic_reconfigure_callback(x80sv_driver::x80svConfig &config, uint32_t level)
    {
        ROS_INFO("Reconfigure Request: Kp=%d, Ki=%d, Kd=%d", config.Kp, config.Ki, config.Kd);

        // Set kp, kd and ki:
        // call signature: channel, kp, kd, ki:
        drrobotMotionDriver_->setMotorVelocityCtrlPID(0, config.Kp, config.Kd, config.Ki);
        drrobotMotionDriver_->setMotorVelocityCtrlPID(1, config.Kp, config.Kd, config.Ki);
    }

    // Apply a raw pwm value on the wheels:
    void PlayerNode::leftPwmValueReceived(const std_msgs::Int32::ConstPtr& left_pwm)
    {
        ROS_INFO("Applying raw PWM value to left wheel");
        int leftPwm = left_pwm->data;
        drrobotMotionDriver_->sendMotorCtrlAllCmd(PWM, leftPwm, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL);
    }

    void PlayerNode::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        double g_vel = cmd_vel->linear.x;
        double t_vel = cmd_vel->angular.z;

        // Extract left and right wheel speeds:
        double leftWheel = (2 * g_vel - t_vel * wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel * wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

        // Call lower level function, we could also publish the topic.
        x80sv_driver::WheelVelocities *wheel_velocities = new x80sv_driver::WheelVelocities();
        wheel_velocities->left = -leftWheel;  // If the left wheel rotates, the robot moves backwards.
        wheel_velocities->right = rightWheel;
        wheelVelReceived(x80sv_driver::WheelVelocities::ConstPtr(wheel_velocities));
    }

    // Apply rotation to the wheels in terms of radians per second:
    void PlayerNode::wheelVelReceived(const x80sv_driver::WheelVelocities::ConstPtr& wheel_velocities)
    {
        // Echo topic to outside world:
        requested_wheel_velocities_pub_.publish(wheel_velocities);

        // Echo smoothed value:
        x80sv_driver::WheelVelocities filtered_wheel_velocities;
        //filtered_wheel_velocities.left = _left_wheel_filter.feed(wheel_velocities->left);
        //filtered_wheel_velocities.right = _right_wheel_filter.feed(wheel_velocities->right);
        filtered_wheel_velocities.left = wheel_velocities->left;
        filtered_wheel_velocities.right = wheel_velocities->right;
        smoothed_wheel_velocities_pub_.publish(filtered_wheel_velocities);

        // seems the right wheel needs a minor offset to prevent an angle when going straight
        int leftWheelCmd = motorDir_ * filtered_wheel_velocities.left * encoderOneCircleCnt_ / (2 * M_PI);
        int rightWheelCmd = motorDir_ * filtered_wheel_velocities.right * encoderOneCircleCnt_ / (2 * M_PI);

        // ROS_INFO("Received control command: [%d, %d]", leftWheelCmd, rightWheelCmd);
        drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity, leftWheelCmd, rightWheelCmd, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL);
    }


    // Calculate the rotation of the wheel relative to a previous time:
    // movement delta in radians.
    void PlayerNode::calculateMovementDelta(x80sv_driver::MotorInfo& mtr, int& encoderPrevious, double& movementDelta, double& joint_angle)
    {
        const uint encoderMax = 32768;

        if (encoderPrevious != -1)
        {
            int encoderDelta = 0;

            if (encoderPrevious < 1000 && mtr.encoder_pos > 31000)
            {
                // When the encoder wrap around from lower to upper value
                // moving backwards. Encoderdelta must be negative number:
                encoderDelta = -((encoderMax - mtr.encoder_pos) + encoderPrevious);
            }
            else if (encoderPrevious > 31000 && mtr.encoder_pos < 1000)
            {
                // When moving from top to bottom, we move forward, so
                // delta must be positive:
                encoderDelta = (encoderMax - encoderPrevious) + mtr.encoder_pos;
            }
            else
            {
                // The 'normal' case:
                encoderDelta = mtr.encoder_pos - encoderPrevious;
            }

            movementDelta = encoder2rad(encoderDelta);
        }
        else
        {
            movementDelta = 0;
        }

        joint_angle = encoder2rad(mtr.encoder_pos);
        encoderPrevious = mtr.encoder_pos;
    }


    /*
        This function calculates odometry information from the encoderdata.  
        It creates a transform from 'odom' to 'base_footprint'

        motor 0 is the left wheel, motor 1 the right wheel.

        The x axis is forward, the y axis is to the left, and the z is upwards.

        The base footprint is exactly between the wheels. The base link is the center of the robot.
    */
    void PlayerNode::publishOdometry(const x80sv_driver::MotorInfoArray& motorInfo)
    {
        static int mEncoderPreviousLeft = -1; //TODO confirm left and right are correct
        static int mEncoderPreviousRight = -1;
        static ros::Time prev_time(0);

        ros::Time nu = ros::Time::now();

        double time_delta = (nu - prev_time).toSec();
        prev_time = nu;
        if (time_delta < 0.0001)
        {
            time_delta = 0.0001;
        }

        double left_joint_angle;
        double right_joint_angle;

        x80sv_driver::MotorInfo mtr0 = motorInfo.motorInfos.at(0); // Left motor
        x80sv_driver::MotorInfo mtr1 = motorInfo.motorInfos.at(1); // Right motor

        // ROS_INFO("Encoder values: %u, %u", mtr0.encoder_pos, mtr1.encoder_pos);

        double left_movement_delta;
        double right_movement_delta;

        calculateMovementDelta(mtr0, mEncoderPreviousLeft, left_movement_delta, left_joint_angle);
        calculateMovementDelta(mtr1, mEncoderPreviousRight, right_movement_delta, right_joint_angle);

        {
            // Publish wheel positions:
    	    x80sv_driver::WheelVelocities actual_wheel_positions;
            actual_wheel_positions.left = left_joint_angle;
            actual_wheel_positions.right = right_joint_angle;
            actual_wheel_positions_pub_.publish(actual_wheel_positions);
        }

        {
            // Calculate and publish wheel velocities:
            x80sv_driver::WheelVelocities actual_wheel_velocities;
            actual_wheel_velocities.left = left_movement_delta / time_delta;
            actual_wheel_velocities.right = right_movement_delta / time_delta;
            actual_wheel_velocities_pub_.publish(actual_wheel_velocities);
        }

        // TODO: take into account the wheel direction:
        double d_left = -left_movement_delta * wheelRadius_;
        double d_right = right_movement_delta * wheelRadius_;

        // average distance between 2 wheels = actual distance from center
        double averageDistance = (d_left + d_right) / 2.0;

        // difference in angle from last encoder values, distance between wheel centers in m
        // When the right wheel moves forward, the angle
        double deltaAngle = atan2((d_right - d_left), wheelDis_);
        // ROS_INFO("Left movement %f, right %f, angle=%f", d_left, d_right, deltaAngle);
        double delta_x = averageDistance * cos(m_theta);
        double delta_y = averageDistance * sin(m_theta);

        // x is in forward direction
        // y is sideways.
        // Angle is zero when facing forwards.

        // TODO: retrieve velocities:
        double vx = averageDistance / time_delta;
        double vy = 0;
        double vth = deltaAngle / time_delta;

        // update pose:
        m_theta += deltaAngle;
        m_x += delta_x;
        m_y += delta_y;

        // ROS_INFO("Robot pos: %f, %f", m_x, m_y);

        // Send out joint state via topic:
        // The joint name is a combination of the erratic wheel and the xacro description.
        // This is published into static tf via the robot_state_publisher.
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = nu;
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name[0] = "base_link_left_wheel_joint";
        joint_state.position[0] = left_joint_angle;
        joint_state.name[1] = "base_link_right_wheel_joint";
        joint_state.position[1] = right_joint_angle;

        // These are two joints we cannot measure:
        joint_state.name[2] = "base_caster_support_joint";
        joint_state.position[2] = 0;
        joint_state.name[3] = "caster_wheel_joint";
        joint_state.position[3] = 0;
        m_joint_state.publish(joint_state);


        // Grab some constant things:
        tf::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, m_theta);

        // Construct tf message:
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(m_x, m_y, 0.0));
        transform.setRotation(odom_quat);

        // Send via tf system:
        m_odom_broadcaster.sendTransform(tf::StampedTransform(transform, nu, "odom", "base_footprint"));

        // Construct odometry message:
        nav_msgs::Odometry odom;
        odom.header.stamp = nu;
        odom.header.frame_id = "odom";

        // Set position:
        odom.pose.pose.position.x = m_x;
        odom.pose.pose.position.y = m_y;

        geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(m_theta);
        odom.pose.pose.orientation = odom_quat2;

        // Set velocity:
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // Publish the message:
        m_odom_pub.publish(odom);
    }

    int PlayerNode::stop()
    {
        int status = 0;
        _comm_interface->close();
        usleep(1000000);
        return (status);
    }

    void PlayerNode::doUpdate()
    {

        if (_comm_interface->isOpen())
        {
            drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
            drrobotMotionDriver_->readRangeSensorData(&rangeSensorData_);
            drrobotMotionDriver_->readStandardSensorData(&standardSensorData_);
            drrobotMotionDriver_->readCustomSensorData(&customSensorData_);

            // Translate from driver data to ROS data
            cntNum_++;
            x80sv_driver::MotorInfoArray motorInfoArray;
            motorInfoArray.motorInfos.resize(MOTOR_NUM);
            for (uint32_t i = 0; i < MOTOR_NUM; ++i)
            {
                motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
                motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
                motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
                motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
                motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
                motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
                motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
                motorInfoArray.motorInfos[i].motor_current = (float) motorSensorData_.motorSensorCurrent[i] / 728;
                motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
            }


            //ROS_INFO("publish motor info array");
            motorInfo_pub_.publish(motorInfoArray);
            publishOdometry(motorInfoArray);

            x80sv_driver::RangeArray rangerArray;
            rangerArray.ranges.resize(US_NUM);
            if (enable_sonar_)
            {
                for (uint32_t i = 0; i < US_NUM; ++i)
                {
                    rangerArray.ranges[i].header.stamp = ros::Time::now();
                    rangerArray.ranges[i].header.frame_id = string("drrobot_sonar_");
                    rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
                    rangerArray.ranges[i].range = (float) rangeSensorData_.usRangeSensor[i] / 100; //to meters

                    // around 30 degrees
                    rangerArray.ranges[i].field_of_view = 0.5236085;
                    rangerArray.ranges[i].max_range = 0.81;
                    rangerArray.ranges[i].min_range = 0;
                    rangerArray.ranges[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
                }

                sonar_pub_.publish(rangerArray);
            }

            if (enable_ir_)
            {
                rangerArray.ranges.resize(IR_NUM);
                for (uint32_t i = 0; i < IR_NUM; ++i)
                {
                    rangerArray.ranges[i].header.stamp = ros::Time::now();
                    rangerArray.ranges[i].header.frame_id = string("drrobot_ir_");
                    rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
                    rangerArray.ranges[i].range = ad2Dis(rangeSensorData_.irRangeSensor[i]);
                    rangerArray.ranges[i].radiation_type = sensor_msgs::Range::INFRARED;
                }

                ir_pub_.publish(rangerArray);
            }

            x80sv_driver::StandardSensor standardSensor;
            standardSensor.humanSensorData.resize(4);
            standardSensor.tiltingSensorData.resize(2);
            standardSensor.overHeatSensorData.resize(2);
            standardSensor.header.stamp = ros::Time::now();
            standardSensor.header.frame_id = string("drrobot_standardsensor");
            for (uint32_t i = 0; i < 4; i++)
                standardSensor.humanSensorData[i] = standardSensorData_.humanSensorData[i];
            for (uint32_t i = 0; i < 2; i++)
                standardSensor.tiltingSensorData[i] = standardSensorData_.tiltingSensorData[i];
            for (uint32_t i = 0; i < 2; i++)
                standardSensor.overHeatSensorData[i] = standardSensorData_.overHeatSensorData[i];

            standardSensor.thermoSensorData = standardSensorData_.thermoSensorData;

            standardSensor.boardPowerVol = (double) standardSensorData_.boardPowerVol * 9 / 4095;
            standardSensor.servoPowerVol = (double) standardSensorData_.servoPowerVol * 9 / 4095;

            standardSensor.motorPowerVol = (double) standardSensorData_.motorPowerVol * 24 / 4095;
            standardSensor.refVol = (double) standardSensorData_.refVol / 4095 * 6;
            standardSensor.potVol = (double) standardSensorData_.potVol / 4095 * 6;
            standardSensor_pub_.publish(standardSensor);

            x80sv_driver::CustomSensor customSensor;
            customSensor.customADData.resize(8);
            customSensor.header.stamp = ros::Time::now();
            customSensor.header.frame_id = string("drrobot_customsensor");

            for (uint32_t i = 0; i < 8; i++)
            {
                customSensor.customADData[i] = customSensorData_.customADData[i];
            }

            customSensor.customIO = (uint8_t) (customSensorData_.customIO & 0xff);
            customSensor_pub_.publish(customSensor);
        }
    }

    void PlayerNode::produce_motion_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
    {
            CommState communication_state = _comm_interface->getCommunicationState();

            if (communication_state == Connected)
            {
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Motion driver connected");
            }
            else
            {
                stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motion driver disconnected");
            }

            stat.add("com cnt", _comm_interface->getComCnt());

            int packets_ok, packets_error;
            drrobotMotionDriver_->get_packet_stats(packets_ok, packets_error);
            stat.add("motion packets ok", packets_ok);
            stat.add("motion packets error", packets_error);

            drrobotPowerDriver_->get_packet_stats(packets_ok, packets_error);
            stat.add("power packets ok", packets_ok);
            stat.add("power packets error", packets_error);
    }

}
