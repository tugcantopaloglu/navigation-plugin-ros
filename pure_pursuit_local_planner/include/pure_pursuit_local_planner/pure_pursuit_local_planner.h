#ifndef PURE_PURSUIT_LOCAL_PLANNER_H_
#define PURE_PURSUIT_LOCAL_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <string>
#include <angles/angles.h>

namespace pure_pursuit_local_planner
{

    class PurePursuitLocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        PurePursuitLocalPlanner();

        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);
        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
        bool isGoalReached();

    private:
        double getEuclideanDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);

        costmap_2d::Costmap2DROS *costmap_ros_;
        tf2_ros::Buffer *tf_buffer_;
        bool initialized_;
        std::vector<geometry_msgs::PoseStamped> global_plan_;

        double lookahead_distance_;
        double linear_velocity_;
        double max_angular_velocity_;
        double goal_dist_tolerance_;
        int current_waypoint_idx_;
        std::string global_frame_id_;
        std::string base_frame_id_;
    };

}
#endif