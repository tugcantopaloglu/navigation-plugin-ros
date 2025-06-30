#include <pure_pursuit_local_planner/pure_pursuit_local_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(pure_pursuit_local_planner::PurePursuitLocalPlanner, nav_core::BaseLocalPlanner)

namespace pure_pursuit_local_planner
{

    PurePursuitLocalPlanner::PurePursuitLocalPlanner()
        : costmap_ros_(NULL), tf_buffer_(NULL), initialized_(false),
          lookahead_distance_(0.5), linear_velocity_(0.2), max_angular_velocity_(1.0), goal_dist_tolerance_(0.1), current_waypoint_idx_(0) {}

    void PurePursuitLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            tf_buffer_ = tf;

            ros::NodeHandle nh;
            std::string base_param_path = "move_base/" + name + "/";

            if (!nh.getParam(base_param_path + "lookahead_distance", lookahead_distance_))
            {
                ROS_WARN("PurePursuitLocalPlanner: Param '%s' not set, using default 0.5", (base_param_path + "lookahead_distance").c_str());
                lookahead_distance_ = 0.5;
            }
            if (!nh.getParam(base_param_path + "linear_velocity", linear_velocity_))
            {
                ROS_WARN("PurePursuitLocalPlanner: Param '%s' not set, using default 0.2", (base_param_path + "linear_velocity").c_str());
                linear_velocity_ = 0.2;
            }
            if (!nh.getParam(base_param_path + "max_angular_velocity", max_angular_velocity_))
            {
                ROS_WARN("PurePursuitLocalPlanner: Param '%s' not set, using default 1.0", (base_param_path + "max_angular_velocity").c_str());
                max_angular_velocity_ = 1.0;
            }
            if (!nh.getParam(base_param_path + "goal_dist_tolerance", goal_dist_tolerance_))
            {
                ROS_WARN("PurePursuitLocalPlanner: Param '%s' not set, using default 0.1", (base_param_path + "goal_dist_tolerance").c_str());
                goal_dist_tolerance_ = 0.1;
            }

            global_frame_id_ = costmap_ros_->getGlobalFrameID();
            base_frame_id_ = costmap_ros_->getBaseFrameID();

            initialized_ = true;
            ROS_INFO("PurePursuitLocalPlanner initialized. Lookahead: %.2f m, Linear Vel: %.2f m/s, Max Angular Vel: %.2f rad/s, Goal Tol: %.2f m",
                     lookahead_distance_, linear_velocity_, max_angular_velocity_, goal_dist_tolerance_);
        }
        else
        {
            ROS_WARN("PurePursuitLocalPlanner has already been initialized.");
        }
    }

    bool PurePursuitLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("PurePursuitLocalPlanner has not been initialized.");
            return false;
        }
        global_plan_ = orig_global_plan;
        current_waypoint_idx_ = 0;
        if (global_plan_.empty())
        {
            ROS_WARN("PurePursuitLocalPlanner: Received empty plan.");
            return true;
        }
        ROS_INFO("PurePursuitLocalPlanner: New plan received with %zu points.", global_plan_.size());
        return true;
    }

    bool PurePursuitLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        if (!initialized_)
        {
            ROS_ERROR("PurePursuitLocalPlanner has not been initialized.");
            return false;
        }

        if (global_plan_.empty())
        {
            ROS_DEBUG_THROTTLE(1.0, "PurePursuitLocalPlanner: Global plan is empty.");
            return true;
        }

        geometry_msgs::PoseStamped robot_pose_global;
        if (!costmap_ros_->getRobotPose(robot_pose_global))
        {
            ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: Could not get robot pose.");
            return false;
        }

        if (current_waypoint_idx_ >= global_plan_.size())
        {
            double dist_to_final_goal = getEuclideanDistance(robot_pose_global.pose.position, global_plan_.back().pose.position);
            if (dist_to_final_goal <= goal_dist_tolerance_)
            {
                ROS_INFO_THROTTLE(1.0, "PurePursuitLocalPlanner: Reached end of plan and goal tolerance.");
            }
            else
            {
                ROS_WARN_THROTTLE(1.0, "PurePursuitLocalPlanner: End of plan but not at goal. Stopping. Dist: %.2f", dist_to_final_goal);
            }
            return true;
        }

        geometry_msgs::PoseStamped target_waypoint_global = global_plan_[current_waypoint_idx_];
        bool lookahead_point_found_on_segment = false;

        for (int i = current_waypoint_idx_; i < global_plan_.size(); ++i)
        {
            target_waypoint_global = global_plan_[i];
            double dist_to_robot = getEuclideanDistance(robot_pose_global.pose.position, target_waypoint_global.pose.position);

            if (dist_to_robot >= lookahead_distance_)
            {
                if (i > 0)
                {
                    geometry_msgs::Point p_prev = global_plan_[i - 1].pose.position;
                    geometry_msgs::Point p_curr = global_plan_[i].pose.position;
                    geometry_msgs::Point robot_pos = robot_pose_global.pose.position;

                    double dx = p_curr.x - p_prev.x;
                    double dy = p_curr.y - p_prev.y;

                    if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6)
                    {
                        lookahead_point_found_on_segment = false;
                    }
                    else
                    {
                        double a = dx * dx + dy * dy;
                        double b = 2 * (dx * (p_prev.x - robot_pos.x) + dy * (p_prev.y - robot_pos.y));
                        double c = (p_prev.x - robot_pos.x) * (p_prev.x - robot_pos.x) +
                                   (p_prev.y - robot_pos.y) * (p_prev.y - robot_pos.y) -
                                   lookahead_distance_ * lookahead_distance_;

                        double discriminant = b * b - 4 * a * c;

                        if (discriminant >= 0)
                        {
                            double t1 = (-b + std::sqrt(discriminant)) / (2 * a);
                            double t2 = (-b - std::sqrt(discriminant)) / (2 * a);

                            double t = -1.0;
                            if (t1 >= 0 && t1 <= 1.0)
                                t = t1;
                            if (t2 >= 0 && t2 <= 1.0)
                            {
                                if (t < 0 || t2 > t)
                                    t = t2;
                            }

                            if (t >= 0 && t <= 1.0)
                            {
                                target_waypoint_global.pose.position.x = p_prev.x + t * dx;
                                target_waypoint_global.pose.position.y = p_prev.y + t * dy;
                                target_waypoint_global.header = global_plan_[i].header;
                                lookahead_point_found_on_segment = true;
                            }
                        }
                    }
                }
                if (lookahead_point_found_on_segment)
                    break;
            }

            if (dist_to_robot < lookahead_distance_ * 0.25 && i < global_plan_.size() - 1)
            {
                current_waypoint_idx_ = i + 1;
            }
            else if (dist_to_robot < lookahead_distance_ && i == global_plan_.size() - 1)
            {
                current_waypoint_idx_ = i;
                break;
            }
            else if (dist_to_robot < lookahead_distance_)
            {
            }
        }

        if (!lookahead_point_found_on_segment && !global_plan_.empty())
        {
            target_waypoint_global = global_plan_.back();
            current_waypoint_idx_ = global_plan_.size() - 1;
        }

        geometry_msgs::PoseStamped target_waypoint_robot_frame;
        try
        {
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                base_frame_id_, target_waypoint_global.header.frame_id, ros::Time(0), ros::Duration(0.1));
            tf2::doTransform(target_waypoint_global, target_waypoint_robot_frame, transform_stamped);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "PurePursuitLocalPlanner: TF Exception: %s. Target frame: %s, Robot frame: %s", ex.what(), target_waypoint_global.header.frame_id.c_str(), base_frame_id_.c_str());
            return false;
        }

        double L_actual = getEuclideanDistance(robot_pose_global.pose.position, target_waypoint_global.pose.position);
        if (L_actual < 1e-3)
            L_actual = 1e-3;

        double y_error = target_waypoint_robot_frame.pose.position.y;
        double curvature = (2.0 * y_error) / (L_actual * L_actual);

        cmd_vel.linear.x = linear_velocity_;
        cmd_vel.angular.z = linear_velocity_ * curvature;
        cmd_vel.angular.z = std::max(-max_angular_velocity_, std::min(max_angular_velocity_, cmd_vel.angular.z));

        double dist_to_final_goal = getEuclideanDistance(robot_pose_global.pose.position, global_plan_.back().pose.position);
        if (dist_to_final_goal < goal_dist_tolerance_)
        {
            ROS_INFO_THROTTLE(1.0, "PurePursuitLocalPlanner: Goal reached within tolerance (%.2f m). Stopping.", goal_dist_tolerance_);
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            current_waypoint_idx_ = global_plan_.size();
        }
        else if (dist_to_final_goal < lookahead_distance_ * 1.5)
        {
            double speed_factor = dist_to_final_goal / (lookahead_distance_ * 1.5);
            cmd_vel.linear.x = linear_velocity_ * std::max(0.2, speed_factor);
        }

        if (getEuclideanDistance(robot_pose_global.pose.position, global_plan_[current_waypoint_idx_].pose.position) < lookahead_distance_ * 0.3 &&
            current_waypoint_idx_ < global_plan_.size() - 1)
        {
            current_waypoint_idx_++;
            ROS_DEBUG("PurePursuit: Advanced to waypoint %d", current_waypoint_idx_);
        }

        return true;
    }

    bool PurePursuitLocalPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            return true;
        }
        if (global_plan_.empty())
        {
            return true;
        }

        geometry_msgs::PoseStamped robot_pose;
        if (!costmap_ros_->getRobotPose(robot_pose))
        {
            ROS_ERROR_THROTTLE(1.0, "PurePursuitLocalPlanner: Could not get robot pose for goal check.");
            return false;
        }

        double dist_to_goal = getEuclideanDistance(robot_pose.pose.position, global_plan_.back().pose.position);

        if (dist_to_goal < goal_dist_tolerance_ && current_waypoint_idx_ >= global_plan_.size() - 1)
        {
            ROS_INFO_ONCE("PurePursuitLocalPlanner: Goal is reached (dist: %.2f, tol: %.2f)!", dist_to_goal, goal_dist_tolerance_);
            return true;
        }
        return false;
    }

    double PurePursuitLocalPlanner::getEuclideanDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

}