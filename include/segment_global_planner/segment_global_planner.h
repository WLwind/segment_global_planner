#pragma once

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>

namespace segment_global_planner
{
class SegmentGlobalPlanner:public nav_core::BaseGlobalPlanner
{
public:
    SegmentGlobalPlanner();
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    virtual ~SegmentGlobalPlanner(){}

private:
    void trimTrajectory(const geometry_msgs::PoseStamped& start);//trim the points that are behind the robot
    double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);//dist^2
    double distPointToSegment(const geometry_msgs::PoseStamped& p0,const geometry_msgs::PoseStamped& s1, const geometry_msgs::PoseStamped& s2);//calculate diatance from a point(p0) to a line segment(s1 s2)
    double distPointToPoint(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);//calculate diatance from a point to a point
    void insertPoints();//insert points on the segment
    double m_threshold_point_on_line{0.2};//to determin whether a point is on the line
    double m_point_interval{0.05};//distance between two points on a segment
    std::vector<geometry_msgs::PoseStamped> m_trajectory_path;//stors current trajectory
    std::string global_frame_{"map"};//frame id
    ros::Publisher plan_pub_;
    geometry_msgs::PoseStamped current_goal;
};

}//end namespace