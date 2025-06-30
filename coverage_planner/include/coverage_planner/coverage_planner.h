#ifndef COVERAGE_PLANNER_H_
#define COVERAGE_PLANNER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>
#include <algorithm>
#include <tf2/utils.h>

namespace coverage_planner
{

    class CoveragePlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        CoveragePlanner();
        CoveragePlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

    private:
        void polygonCallback(const geometry_msgs::Polygon::ConstPtr &msg);
        bool isPointInPolygon(const geometry_msgs::Point32 &point, const std::vector<geometry_msgs::Point32> &polygon_corners);

        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;
        ros::Publisher plan_pub_;
        ros::Subscriber polygon_sub_;
        std::vector<geometry_msgs::Point32> coverage_polygon_corners_;
        bool initialized_;
        double robot_radius_;
        double coverage_step_factor_;
        std::string global_frame_id_;
        bool polygon_received_;

        double getEuclideanDistance(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2);
    };
}
#endif