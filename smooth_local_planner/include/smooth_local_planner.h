
#ifndef SMOOTH_LOCAL_PLANNER_H
#define SMOOTH_LOCAL_PLANNER_H

#include <nav_core/base_local_planner.h>
#include <base_local_planner/costmap_model.h>

namespace smooth_local_planner
{

    void limit(double& x, double lower, double upper)
    {
        if (x > upper)
        {
            x = upper;
        }

        if (x < lower)
        {
            x = lower;
        }
    }


    class RateLimiter
    {
        public:
            RateLimiter() : 
                _limit(0.1),
                _current(0.0),
                _target(0.0)
            {
            }

            void setInput(double x)
            {
                _target = x;
            }

            void setLimit(double x)
            {
                _limit = x;
            }

            void calculate()
            {
                double err = _target - _current;
                limit(err, -_limit, _limit);
                _current += err;
            }

            double getOutput()
            {
                return _current;
            }

        private:
            double _current;
            double _target;
            double _limit;
    };

    class SmoothLocalPlanner : public nav_core::BaseLocalPlanner
    {
        public:
            SmoothLocalPlanner();
            virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
            virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
            virtual bool isGoalReached();
        private:
            bool pathFree(double x, double y, double theta, double v);

            // TODO 
            costmap_2d::Costmap2DROS*           _costmap_ros;
            base_local_planner::CostmapModel*   _costmap_model;
            ros::Publisher              _next_pose_pub;

            // Local copy of plan:
            bool _has_plan;
            int _plan_index;
            const std::vector<geometry_msgs::PoseStamped>* _global_plan;

            // Rate limiters for outputs:
            RateLimiter _v_limiter;
            RateLimiter _omega_limiter;
    };
}

#endif

