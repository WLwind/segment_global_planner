#pragma once

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <list>
#include <string>
#include <queue>
#include <segment_global_planner/SegmentGlobalPlannerConfig.h>
#include <std_srvs/Empty.h>

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
    bool isGoalReached();//has the robot reached the segment goal
    void reconfigureCB(segment_global_planner::SegmentGlobalPlannerConfig& config, uint32_t level);//dynamic_reconfigure callback
    bool clearTrajectoryCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);//clear trajectory server callback
    
    double m_threshold_point_on_line{0.3};//to determin whether a point is on the line
    double m_point_interval{0.05};//distance between two points on a segment
    double m_goal_threshold{0.15};//goal threthold
    bool m_got_first_goal{false};//has the planner received the first goal
    std::list<geometry_msgs::PoseStamped> m_trajectory_path;//stors current segment trajectory
    std::string global_frame_{"map"};//frame id
    ros::Publisher plan_pub_;//gui publisher
    geometry_msgs::PoseStamped m_current_pose,m_current_goal,m_segment_goal;//current robot pose, current trajectory goal and segment goal
    std::queue<geometry_msgs::PoseStamped> m_child_goals;//stors every child goals
    std::unique_ptr<dynamic_reconfigure::Server<SegmentGlobalPlannerConfig>> m_dynamic_config_server;//server pointer for dynamic reconfigure
    ros::ServiceServer m_clear_trajectory_server;//service for clear trajectory
};

}//end namespace