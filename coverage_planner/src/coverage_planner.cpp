#include <coverage_planner/coverage_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(coverage_planner::CoveragePlanner, nav_core::BaseGlobalPlanner)

namespace coverage_planner
{

    CoveragePlanner::CoveragePlanner() : costmap_ros_(NULL), initialized_(false), polygon_received_(false), robot_radius_(0.2), coverage_step_factor_(0.9) {}

    CoveragePlanner::CoveragePlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), initialized_(false), polygon_received_(false), robot_radius_(0.2), coverage_step_factor_(0.9)
    {
        initialize(name, costmap_ros);
    }

    void CoveragePlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();
            global_frame_id_ = costmap_ros_->getGlobalFrameID();

            ros::NodeHandle nh;
            std::string base_param_path = "move_base/" + name + "/";

            plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/" + name + "/coverage_plan", 1);

            std::string polygon_topic_value;
            if (!nh.getParam(base_param_path + "polygon_topic", polygon_topic_value))
            {
                ROS_WARN("CoveragePlanner: Param '%s' not set, using default '/coverage_polygon'", (base_param_path + "polygon_topic").c_str());
                polygon_topic_value = "/coverage_polygon";
            }
            polygon_sub_ = nh.subscribe(polygon_topic_value, 1, &CoveragePlanner::polygonCallback, this);

            if (!nh.getParam(base_param_path + "robot_radius", robot_radius_))
            {
                ROS_WARN("CoveragePlanner: Param '%s' not set, using default 0.2", (base_param_path + "robot_radius").c_str());
                robot_radius_ = 0.2;
            }

            if (!nh.getParam(base_param_path + "coverage_step_factor", coverage_step_factor_))
            {
                ROS_WARN("CoveragePlanner: Param '%s' not set, using default 0.9", (base_param_path + "coverage_step_factor").c_str());
                coverage_step_factor_ = 0.9;
            }

            ROS_INFO("CoveragePlanner initialized with polygon_topic: %s, robot_radius: %.3f, step_factor: %.2f",
                     polygon_topic_value.c_str(), robot_radius_, coverage_step_factor_);

            initialized_ = true;
        }
        else
        {
            ROS_WARN("CoveragePlanner has already been initialized.");
        }
    }

    void CoveragePlanner::polygonCallback(const geometry_msgs::Polygon::ConstPtr &msg)
    {
        ROS_INFO("CoveragePlanner: PolygonCallback ENTERED.");
        if (msg->points.size() == 4)
        {
            coverage_polygon_corners_ = msg->points;
            polygon_received_ = true;
            ROS_INFO("CoveragePlanner: Received 4-corner polygon with %zu points.", msg->points.size());
        }
        else
        {
            ROS_WARN("CoveragePlanner: Received polygon does not have 4 corners (has %zu). Ignoring.", msg->points.size());
            polygon_received_ = false;
        }
    }

    bool CoveragePlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                   const geometry_msgs::PoseStamped &goal,
                                   std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("CoveragePlanner has not been initialized, please call initialize() first");
            return false;
        }

        plan.clear();

        if (!polygon_received_ || coverage_polygon_corners_.size() != 4)
        {
            ROS_ERROR("CoveragePlanner: Polygon not received or invalid (corners: %zu). Cannot make plan.", coverage_polygon_corners_.size());
            return false;
        }

        double min_x = coverage_polygon_corners_[0].x;
        double max_x = coverage_polygon_corners_[0].x;
        double min_y = coverage_polygon_corners_[0].y;
        double max_y = coverage_polygon_corners_[0].y;

        for (size_t i = 1; i < coverage_polygon_corners_.size(); ++i)
        {
            min_x = std::min(min_x, (double)coverage_polygon_corners_[i].x);
            max_x = std::max(max_x, (double)coverage_polygon_corners_[i].x);
            min_y = std::min(min_y, (double)coverage_polygon_corners_[i].y);
            max_y = std::max(max_y, (double)coverage_polygon_corners_[i].y);
        }

        double effective_robot_radius = robot_radius_;
        if (effective_robot_radius <= 0.0)
        {
            ROS_ERROR("CoveragePlanner: Robot radius must be positive.");
            return false;
        }
        double sweep_width = 2.0 * effective_robot_radius * coverage_step_factor_;
        if (sweep_width <= 1e-3)
        {
            ROS_ERROR("CoveragePlanner: Sweep width is too small or zero (robot_radius: %.3f, step_factor: %.2f). Cannot make plan.", robot_radius_, coverage_step_factor_);
            return false;
        }

        double start_x_offset = min_x + effective_robot_radius;
        double end_x_offset = max_x - effective_robot_radius;
        double start_y_offset = min_y + effective_robot_radius;
        double end_y_offset = max_y - effective_robot_radius;

        if (start_x_offset >= end_x_offset || start_y_offset >= end_y_offset)
        {
            ROS_WARN("CoveragePlanner: Polygon is too small for the given robot_radius to create a valid coverage area.");
            return false;
        }

        double current_x = start_x_offset;
        bool sweep_up = true;

        geometry_msgs::PoseStamped waypoint;
        waypoint.header.stamp = ros::Time::now();
        waypoint.header.frame_id = global_frame_id_;
        waypoint.pose.orientation.w = 1.0;

        plan.push_back(start);

        while (current_x <= end_x_offset + 1e-3)
        {
            if (sweep_up)
            {
                waypoint.pose.position.x = current_x;
                waypoint.pose.position.y = start_y_offset;

                if (plan.empty() || this->getEuclideanDistance(plan.back().pose.position, waypoint.pose.position) > 0.01)
                    plan.push_back(waypoint);

                waypoint.pose.position.y = end_y_offset;

                if (plan.empty() || this->getEuclideanDistance(plan.back().pose.position, waypoint.pose.position) > 0.01)
                    plan.push_back(waypoint);
            }
            else
            {
                waypoint.pose.position.x = current_x;
                waypoint.pose.position.y = end_y_offset;

                if (plan.empty() || this->getEuclideanDistance(plan.back().pose.position, waypoint.pose.position) > 0.01)
                    plan.push_back(waypoint);

                waypoint.pose.position.y = start_y_offset;
                if (plan.empty() || this->getEuclideanDistance(plan.back().pose.position, waypoint.pose.position) > 0.01)
                    plan.push_back(waypoint);
            }
            current_x += sweep_width;
            sweep_up = !sweep_up;
        }

        if (plan.size() <= 1 && polygon_received_)
        {
            ROS_WARN("CoveragePlanner: Plan has only start point or is empty after generation. Polygon might be too small or start point is outside valid area for sweep.");
            plan.clear();
            return false;
        }

        if (!plan.empty())
        {
            for (size_t i = 0; i < plan.size() - 1; ++i)
            {
                double dx = plan[i + 1].pose.position.x - plan[i].pose.position.x;
                double dy = plan[i + 1].pose.position.y - plan[i].pose.position.y;
                double yaw = atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                plan[i].pose.orientation = tf2::toMsg(q);
            }
            if (plan.size() > 1)
            {
                plan.back().pose.orientation = plan[plan.size() - 2].pose.orientation;
            }
            else if (plan.size() == 1)
            {
                plan.back().pose.orientation = start.pose.orientation;
            }
        }
        else
        {
            ROS_WARN("CoveragePlanner: Plan is empty after generation attempt.");
            return false;
        }

        nav_msgs::Path gui_path;
        gui_path.poses = plan;
        gui_path.header.frame_id = global_frame_id_;
        gui_path.header.stamp = ros::Time::now();
        plan_pub_.publish(gui_path);

        ROS_INFO("CoveragePlanner: Plan generated with %zu points.", plan.size());
        return true;
    }

    double CoveragePlanner::getEuclideanDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

}