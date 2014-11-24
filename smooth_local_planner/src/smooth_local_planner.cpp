
#include <smooth_local_planner.h>
#include <pluginlib/class_list_macros.h>
#include <math.h>


PLUGINLIB_EXPORT_CLASS(smooth_local_planner::SmoothLocalPlanner, nav_core::BaseLocalPlanner)

namespace smooth_local_planner
{

    // Distance is some weird stl function with iterators...
    double my_distance(geometry_msgs::Pose& p1, geometry_msgs::Pose& p2)
    {
        double dx = p1.position.x - p2.position.x;
        double dy = p1.position.y - p2.position.y;

        // Use pythagoras!
        double dst = sqrt(dx*dx + dy*dy);
        // ROS_INFO("Distance: %f", dst);
        return dst;
    }

    // Returns the required angle towards the goal
    double angle(geometry_msgs::Pose& p1, geometry_msgs::Pose& p2)
    {
        double dx = p2.position.x - p1.position.x;
        double dy = p2.position.y - p1.position.y;

        return atan2(dy, dx);
    }

    // Normalize angle to -pi .. pi
    double clip_angle(double x)
    {
        if (x > M_PI)
        {
            return clip_angle(x - 2*M_PI);
        }
        if (x < -M_PI)
        {
            return clip_angle(x + 2*M_PI);
        }
        return x;
    }

    SmoothLocalPlanner::SmoothLocalPlanner()
        :
        _has_plan(false)
    {
    }

    void SmoothLocalPlanner::initialize(std::string name,
        tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros)
    {
        _costmap_ros = costmap_ros;
        ros::NodeHandle private_nh("smooth_local_planner");
        _next_pose_pub = private_nh.advertise<geometry_msgs::PoseStamped>("next_pose", 1);
    }

    bool SmoothLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
    {
        ROS_INFO("Plan set!");
        _has_plan = true;
        _plan_index = 0;
        _global_plan = &global_plan;

        return true;
    }

    bool SmoothLocalPlanner::isGoalReached()
    {
        if (!_has_plan)
        {
            return false;
        }

        // If we processed all points of the plan, we are complete:
        if (_plan_index < _global_plan->size())
        {
            return false;
        }

        return true;
    }

    // Compute velocity based on current orientation and velocity.
    bool SmoothLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        double fs = 3.0;
        double dt = 1.0 / fs;
        double omega_max = 1.0;

        // ROS_INFO("Compute velocity");
        if (!_has_plan)
        {
            return false;
        }

        if (isGoalReached())
        {
            return false;
        }

        // Determine current position:
        tf::Stamped<tf::Pose> global_pose;
        if (!_costmap_ros->getRobotPose(global_pose))
        {
            return false;
        }

        // Convert tf pose to msg pose:
        geometry_msgs::Pose current;
        tf::poseTFToMsg(global_pose, current);
        double current_angle = tf::getYaw(global_pose.getRotation());

        // Determine the closest nearby waypoint that is not passed yet:
        geometry_msgs::Pose target = (*_global_plan)[_plan_index].pose;

        // Check if we are close enough to head to the next point:
        while (my_distance(current, target) < 0.2)
        {
            ROS_INFO("Going to next point");
            _plan_index++;
            if (isGoalReached())
            {
                return false;
            }

            // Update the current target:
            target = (*_global_plan)[_plan_index].pose;
        }

        // Publish next pose:
        geometry_msgs::PoseStamped next_tgt;
        next_tgt.pose = target;
        next_tgt.header.frame_id = "/map";
        next_tgt.header.stamp = ros::Time::now();

        _next_pose_pub.publish(next_tgt);

        // Determine error to next point:
        double pos_error = my_distance(current, target);
        double tgt_angle = angle(current, target);
        double angle_diff = clip_angle(tgt_angle - current_angle);
        // ROS_INFO("Current angle: %f, target angle %f", current_angle, tgt_angle);

        // Determine the speed (Kp control):
        double v = pos_error * 3;
        double omega = angle_diff * 0.2;

        // If the angle difference is large, first rotate in place, do not speed up:
        if (abs(angle_diff) > 0.6)
        {
            v = 0;
        }

        // v = 0;
        // Limit output speeds:
        limit(v, -0.5, 0.5);
        limit(omega, -0.5, 0.5);

        // limit rate:
        _omega_limiter.setLimit(omega_max * dt);
        _v_limiter.setInput(v);
        _omega_limiter.setInput(omega);

        _v_limiter.calculate();
        _omega_limiter.calculate();

        // Calculate cmd vel commands:
        cmd_vel.linear.x = _v_limiter.getOutput();
        cmd_vel.angular.z = _omega_limiter.getOutput();

        return true;
    }

}

