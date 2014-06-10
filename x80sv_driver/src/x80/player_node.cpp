
#include <player_node.h>

#include <SerialCommInterface.hpp>
#include <NetworkCommInterface.hpp>

#include <x80sv_driver/MotorInfoArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace DrRobot
{

    class EncoderFilter
    {
        public:
            EncoderFilter();

            void reset()
            {
                initialized = false;
            }

            void update()
            {
                vx = 0;
            }

            float vx;

        private:
            bool initialized;
    };

    EncoderFilter::EncoderFilter()
    {
        reset();
    }


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

        robotIP_ = "192.168.0.201";
        private_nh.getParam("x80_config/RobotBaseIP", robotIP_);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

        int commPortNum_;

        commPortNum_ = 10001;
        private_nh.getParam("x80_config/RobotPortNum", commPortNum_);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

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

        wheelRadius_ = 0.080;
        private_nh.getParam("x80_config/WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        wheelDis_ = 0.305;
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
        else if (robotType_ == "I90")
        {
            robotConfig1_.boardType = I90_Power;
            robotConfig2_.boardType = I90_Motion;
        }
        else if (robotType_ == "Sentinel3")
        {
            robotConfig1_.boardType = Sentinel3_Power;
            robotConfig2_.boardType = Sentinel3_Motion;
        }
        else if (robotType_ == "Hawk_H20")
        {
            robotConfig1_.boardType = Hawk_H20_Power;
            robotConfig2_.boardType = Hawk_H20_Motion;
        }
        else if (robotType_ == "X80")
        {
            robotConfig1_.boardType = X80SV;
            robotConfig2_.boardType = X80SV;
        }


        //robotConfig1_.portNum = commPortNum_;
        // robotConfig2_.portNum = commPortNum_ + 1;

        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<x80sv_driver::MotorInfoArray>("drrobot_motor", 1);
        powerInfo_pub_ = node_.advertise<x80sv_driver::PowerInfo>("drrobot_powerinfo", 1);
        if (enable_ir_) {
            ir_pub_ = node_.advertise<x80sv_driver::RangeArray>("drrobot_ir", 1);
        }
        if (enable_sonar_) {
            sonar_pub_ = node_.advertise<x80sv_driver::RangeArray>("drrobot_sonar", 1);
        }
        standardSensor_pub_ = node_.advertise<x80sv_driver::StandardSensor>("drrobot_standardsensor", 1);
        customSensor_pub_ = node_.advertise<x80sv_driver::CustomSensor>("drrobot_customsensor", 1);

        m_odom_pub = node_.advertise<nav_msgs::Odometry>("odom", 50);

        drrobotPowerDriver_ = new MotionSensorDriver(*_comm_interface);
        drrobotMotionDriver_ = new MotionSensorDriver(*_comm_interface);
        drrobotPowerDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig2_);
        cntNum_ = 0;

        // Keep track of robot:
        m_x = 0;
        m_y = 0;
        m_theta = 0;
    }


    PlayerNode::~PlayerNode()
    {
    }


    int PlayerNode::start()
    {
        _comm_interface->open();

        cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&PlayerNode::cmdVelReceived, this, _1));

        drrobotMotionDriver_->setMotorVelocityCtrlPID(0, 1, 5, 170);  // only needed when using velocity (1, 0, 170))
        drrobotMotionDriver_->setMotorVelocityCtrlPID(1, 1, 5, 170);

        drrobotMotionDriver_->setMotorPositionCtrlPID(0, 500, 5, 10000); // PID default is 1000, 5, 10000 (taken from C# src)
        drrobotMotionDriver_->setMotorPositionCtrlPID(1, 500, 5, 10000);       // 500, 2, 510 has smoother motion but does not take friction in account

        return (0);
    }


    void PlayerNode::cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
        double g_vel = cmd_vel->linear.x;
        double t_vel = cmd_vel->angular.z;

        double leftWheel = (2 * g_vel - t_vel * wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel * wheelDis_ + 2 * g_vel) / (2 * wheelRadius_); // seems the right wheel needs a minor offset to prevent an angle when going straight

        int leftWheelCmd = -motorDir_ * leftWheel * encoderOneCircleCnt_ / (2 * M_PI);
        int rightWheelCmd = motorDir_ * rightWheel * encoderOneCircleCnt_ / (2 * M_PI);

        // ROS_INFO("Received control command: [%d, %d]", leftWheelCmd, rightWheelCmd);
        drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity, leftWheelCmd, rightWheelCmd, NOCONTROL, NOCONTROL, NOCONTROL, NOCONTROL);
    }


    // Calculate the rotation of the wheel relative to a previous time:
    void PlayerNode::calculateMovementDelta(x80sv_driver::MotorInfo& mtr, int& encoderPrevious, double& movementDelta)
    {
        const uint encoderMax = 32768;

        if (encoderPrevious != -1)
        {
            if (encoderPrevious < 5000 && mtr.encoder_pos > 25000)
            {
                encoderPrevious = mtr.encoder_pos - encoderPrevious;
            }
            else if (encoderPrevious > 25000 && mtr.encoder_pos < 5000)
            {
                encoderPrevious = mtr.encoder_pos + (encoderMax - encoderPrevious);
            }

            int encoderDelta = encoderPrevious - mtr.encoder_pos;
            // double wheel_angle = ((encoderDelta * 2 * M_PI) / encoderOneCircleCnt_);

            movementDelta = (((wheelRadius_ * 2 * M_PI) / encoderOneCircleCnt_) * encoderDelta);
        }
        else
        {
            movementDelta = 0;
        }

        encoderPrevious = mtr.encoder_pos;
    }


    void PlayerNode::publishOdometry(const x80sv_driver::MotorInfoArray& motorInfo)
    {
        static int mEncoderPreviousLeft = -1; //TODO confirm left and right are correct
        static int mEncoderPreviousRight = -1;

        double d_left = 0;
        double d_right = 0;

        x80sv_driver::MotorInfo mtr0 = motorInfo.motorInfos.at(0);
        x80sv_driver::MotorInfo mtr1 = motorInfo.motorInfos.at(1);

        // ROS_INFO("Encoder values: %u, %u", mtr0.encoder_pos, mtr1.encoder_pos);

        calculateMovementDelta(mtr0, mEncoderPreviousLeft, d_left);
        calculateMovementDelta(mtr1, mEncoderPreviousRight, d_right);

        // average distance between 2 wheels = actual distance from center
        double averageDistance = (d_left + d_right) / 2.0;

        // difference in angle from last encoder values, distance between wheel centers in m
        double deltaAngle = atan2((d_right - d_left), wheelDis_);
        double delta_x = averageDistance * cos(m_theta);
        double delta_y = averageDistance * sin(m_theta);

        // TODO: retrieve velocities:
        double vx = 0;
        double vy = 0;
        double vth = 0;

        // update pose:
        m_theta += deltaAngle;
        m_x += delta_x;
        m_y += delta_y;

        // Grab some constant things:
        tf::Quaternion odom_quat;
        odom_quat.setRPY(m_theta, 0, 0);
        ros::Time nu = ros::Time::now();

        // Construct tf message:
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(m_x, m_y, 0.0));
        transform.setRotation(odom_quat);

        // Send via tf system:
        m_odom_broadcaster.sendTransform(tf::StampedTransform(transform, nu, "odom", "base_link"));

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
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.x = vy;
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

        if ((robotConfig1_.boardType == I90_Power) || (robotConfig1_.boardType == Sentinel3_Power)
                || (robotConfig1_.boardType == Hawk_H20_Power))
        {
            if (_comm_interface->isOpen())
            {
                drrobotPowerDriver_->readPowerSensorData(&powerSensorData_);
                x80sv_driver::PowerInfo powerInfo;
                powerInfo.ref_vol = 1.5 * 4095 / (double) powerSensorData_.refVol;

                powerInfo.bat1_vol = (double) powerSensorData_.battery1Vol * 8 / 4095 * powerInfo.ref_vol;
                powerInfo.bat2_vol = (double) powerSensorData_.battery2Vol * 8 / 4095 * powerInfo.ref_vol;

                powerInfo.bat1_temp = powerSensorData_.battery1Thermo;
                powerInfo.bat2_temp = powerSensorData_.battery2Thermo;

                powerInfo.dcin_vol = (double) powerSensorData_.dcINVol * 8 / 4095 * powerInfo.ref_vol;
                powerInfo.charge_path = powerSensorData_.powerChargePath;
                powerInfo.power_path = powerSensorData_.powerPath;
                powerInfo.power_status = powerSensorData_.powerStatus;

                powerInfo_pub_.publish(powerInfo);
            }
        }

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
            for (uint32_t i = 0; i < MOTOR_NUM; ++i) {
                motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
                motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
                motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
                motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
                motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
                motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
                motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
                if (robotConfig1_.boardType == Hawk_H20_Motion) {
                    motorInfoArray.motorInfos[i].motor_current = (float) motorSensorData_.motorSensorCurrent[i] * 3 / 4096;
                    ;
                } else if (robotConfig1_.boardType != Jaguar) {
                    motorInfoArray.motorInfos[i].motor_current = (float) motorSensorData_.motorSensorCurrent[i] / 728;
                } else {
                    motorInfoArray.motorInfos[i].motor_current = 0.0;
                }
                motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
            }

            //ROS_INFO("publish motor info array");
            motorInfo_pub_.publish(motorInfoArray);
            publishOdometry(motorInfoArray);


            x80sv_driver::RangeArray rangerArray;
            rangerArray.ranges.resize(US_NUM);
            if (enable_sonar_) {
                for (uint32_t i = 0; i < US_NUM; ++i) {

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


            if (enable_ir_) {
                rangerArray.ranges.resize(IR_NUM);
                for (uint32_t i = 0; i < IR_NUM; ++i) {
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

            if (robotConfig1_.boardType != Jaguar) {
                standardSensor.motorPowerVol = (double) standardSensorData_.motorPowerVol * 24 / 4095;
            } else {
                standardSensor.motorPowerVol = (double) standardSensorData_.motorPowerVol * 34.498 / 4095;
            }
            standardSensor.refVol = (double) standardSensorData_.refVol / 4095 * 6;
            standardSensor.potVol = (double) standardSensorData_.potVol / 4095 * 6;
            standardSensor_pub_.publish(standardSensor);

            x80sv_driver::CustomSensor customSensor;
            customSensor.customADData.resize(8);
            customSensor.header.stamp = ros::Time::now();
            customSensor.header.frame_id = string("drrobot_customsensor");

            for (uint32_t i = 0; i < 8; i++) {
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
