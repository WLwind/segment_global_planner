#pragma once

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <list>
#include <string>
#include <queue>
#include <segment_global_planner/SegmentGlobalPlannerConfig.h>
#include <std_srvs/Empty.h>

namespace segment_global_planner
{
    /**
    * This class is an implementation of nav_core::BaseGlobalPlanner from standard ROS navigation package. It can help users to directly set line segments using RViz and let the robot follow the segments sequentially.
    */
class SegmentGlobalPlanner:public nav_core::BaseGlobalPlanner
{
public:
    /**
    * @brief Constructor
    */
    SegmentGlobalPlanner();
    /**
    * @brief Override nav_core::BaseGlobalPlanner::makePlan()
    * @param start The start pose 
    * @param goal The goal pose 
    * @param plan The plan... filled by the planner
    * @return True if a valid plan was found, false otherwise
    */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;
    /**
    * @brief Override nav_core::BaseGlobalPlanner::initialize()
    * @param name The name of this planner
    * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
    */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
    /**
    * @brief Destructor
    */
    virtual ~SegmentGlobalPlanner(){}

private:
    /**
    * @brief Trim the points that are behind the robot
    * @param start Start pose of the trajectory
    */
    void trimTrajectory(const geometry_msgs::PoseStamped& start);
    /**
    * @brief Get squared the distance between p1 and p2
    * @param p1 p1 pose
    * @param p2 p2 pose
    * @return Squared distance
    */
    double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
    /**
    * @brief Calculate diatance from a point(p0) to a line segment(s1 s2)
    * @param p0 p0 pose
    * @param s1 s1 pose
    * @param s2 s2 pose
    * @return Distance
    */
    double distPointToSegment(const geometry_msgs::PoseStamped& p0,const geometry_msgs::PoseStamped& s1, const geometry_msgs::PoseStamped& s2);
    /**
    * @brief Calculate diatance from a point to a point
    * @param p1 p1 pose
    * @param p2 p2 pose
    * @return Distance
    */
    double distPointToPoint(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
    /**
    * @brief Insert points on the segment
    */
    void insertPoints();
    /**
    * @brief Judge whether the robot reached the segment goal or not
    * @return True if the robot reached the goal, false otherwise
    */
    bool isChildGoalReached();
    /*
    * @brief Set the angle to the pose
    * @param pose The pose that you want to set the ange for
    * @param angle The angle by rad
    */
    void setAngle(geometry_msgs::PoseStamped* pose, double angle);
    /**
    * @brief Dynamic_reconfigure callback function
    */
    void reconfigureCB(segment_global_planner::SegmentGlobalPlannerConfig& config, uint32_t level);
    /**
    * @brief Clear trajectory server callback function
    */
    bool clearTrajectoryCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    /**
    * @brief clicked_point topic subscriber callback function
    */
    void clickedPointCB(const geometry_msgs::PointStamped::ConstPtr& ptr);
    
    double m_threshold_point_on_line{0.3};//to determine whether a point is on the line
    double m_point_interval{0.05};//distance between two points on a segment
    double m_goal_threshold{0.2};//goal threshold
    bool m_got_first_goal{false};//whether the planner received the first goal
    std::list<geometry_msgs::PoseStamped> m_trajectory_path;//stores current segment trajectory
    std::string global_frame_{"map"};//global frame id
    ros::Publisher plan_pub_;//GUI publisher
    geometry_msgs::PoseStamped m_current_pose,m_current_goal,m_segment_goal;//current robot pose, current global planner goal (final goal) and current child goal (goal of a segment that is heading to)
    std::queue<geometry_msgs::PoseStamped> m_child_goals;//stores all child goals
    std::unique_ptr<dynamic_reconfigure::Server<SegmentGlobalPlannerConfig>> m_dynamic_config_server;//server pointer for dynamic reconfigure
    ros::ServiceServer m_clear_trajectory_server;//service for clearing trajectory
    ros::Publisher m_pose_from_clicked_point_pub{ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10)};//publish the goal pose when the clicked_point has been received
    ros::Subscriber m_clicked_point_sub{ros::NodeHandle().subscribe<geometry_msgs::PointStamped>("/clicked_point",10,&SegmentGlobalPlanner::clickedPointCB,this)};//rviz clicked_point subscriber
};

}//end namespace