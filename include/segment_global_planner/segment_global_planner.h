#pragma once

#include <vector>
#include <list>
#include <string>
#include <queue>
#include <atomic>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/costmap_model.h>
#include <tf2_ros/transform_listener.h>
#include <segment_global_planner/SegmentGlobalPlannerConfig.h>

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
    * @brief Destructor
    */
    ~SegmentGlobalPlanner();
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

private:
    /**
    * @brief Judge whether the robot reached the segment goal or not
    * @return True if the robot reached the goal, false otherwise
    */
    bool isChildGoalReached();
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
    /**
     * @brief Add a new segment to the segment_list_
     * @param new_plan New segment
     */
    void addNewSegment(std::vector<geometry_msgs::PoseStamped>& new_plan);
    /**
     * @brief Switch to next segment
     */
    void switchSegment();
    /**
     * @brief Find distance from a point to a segment less than threshold_point_on_line_
     * @param point The distance from
     * @param segment The distance to
     * @return True if find a less distance
     */
    bool findDistLessThresh(const geometry_msgs::PoseStamped& point, const std::vector<geometry_msgs::PoseStamped>& segment);
    /**
     * @brief To replan current segment when makePlan() is called
     */
    void replanCB(const std_msgs::Bool::ConstPtr& ptr);
    /**
     * @brief Publish path for display
     */
    void displayPath();

    double threshold_point_on_line_ { 0.1 }; //to determine whether a point is on the line
    double point_interval_ { 0.05 }; //distance between two points on a segment
    double goal_threshold_ { 0.2 }; //goal threshold
    std::string global_frame_ { "map" }; //global frame id
    ros::Publisher plan_pub_; //display path publisher
    geometry_msgs::PoseStamped final_goal_, segment_goal_; //current global planner goal (final goal) and current child goal (goal of a segment that is heading to)
    std::unique_ptr<dynamic_reconfigure::Server<SegmentGlobalPlannerConfig>> dynamic_config_server_; //server pointer for dynamic reconfigure
    ros::ServiceServer clear_trajectory_server_; //service for clearing trajectory
    ros::Publisher pose_from_clicked_point_pub_ { ros::NodeHandle().advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10) }; //publish the goal pose when the clicked_point has been received
    ros::Subscriber clicked_point_sub_ { ros::NodeHandle().subscribe("/clicked_point", 10, &SegmentGlobalPlanner::clickedPointCB, this) }; //rviz clicked_point subscriber
    tf2_ros::Buffer tf_buffer_ { ros::Duration(5.0) };
    tf2_ros::TransformListener tf_ { tf_buffer_ };
    std::list<std::vector<geometry_msgs::PoseStamped>> segment_list_; //stores segments going to be tracked
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_ { "nav_core", "nav_core::BaseGlobalPlanner" };
    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_implementation_; //real global planner for each segment
    bool final_goal_reached_ { true }; //whether final goal is reached
    bool got_first_goal { false }; //first goal mark
    std::vector<geometry_msgs::PoseStamped> last_segment_; //for reaching final goal, when segment_list_ is empty
    std::atomic_bool replan_ { false }; //replan flag
    ros::Subscriber replan_sub_; //to trigger replanning current segment
    bool clicked_a_new_goal_ { false }; //using topic "/clicked_point" setting a new goal
};
}
