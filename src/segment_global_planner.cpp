#include <cmath>
#include <algorithm>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/footprint.h>
#include <segment_global_planner/common.h>
#include <segment_global_planner/line_segment.h>
#include <segment_global_planner/segment_global_planner.h>

PLUGINLIB_EXPORT_CLASS(segment_global_planner::SegmentGlobalPlanner, nav_core::BaseGlobalPlanner)//register plugin

namespace segment_global_planner
{
namespace
{
/**
* @brief Calculate diatance from a point(p0) to a line segment(s1 s2)
* @param p0 p0 pose
* @param s1 s1 pose
* @param s2 s2 pose
* @return Distance
*/
double distPointToSegment(const geometry_msgs::PoseStamped& p0,const geometry_msgs::PoseStamped& s1, const geometry_msgs::PoseStamped& s2)
{
    double p0s1[2] { s1.pose.position.x - p0.pose.position.x, s1.pose.position.y - p0.pose.position.y }; //vectors
    double p0s2[2] { s2.pose.position.x - p0.pose.position.x, s2.pose.position.y - p0.pose.position.y };
    double s1s2[2] { s2.pose.position.x - s1.pose.position.x, s2.pose.position.y - s1.pose.position.y };
    if (s1s2[0] * p0s1[0] + s1s2[1] * p0s1[1] > 0 || s1s2[0] * p0s2[0] + s1s2[1] * p0s2[1] < 0) //dot product, obtuse angle
    {
        return 1.0 / 0.0; //the point p0 is not between point s1 and point s2
    }
    double A = s1.pose.position.y - s2.pose.position.y; //parameters of linear equation : Ax+By+C=0
    double B = s2.pose.position.x - s1.pose.position.x;
    double C = s1.pose.position.x * s2.pose.position.y - s1.pose.position.y * s2.pose.position.x;
    return std::abs(A * p0.pose.position.x + B * p0.pose.position.y + C) / std::sqrt(A * A + B * B); //distance form p0 to line s1s2 : |Ax+By+C|/√(A²+B²)
}
}

SegmentGlobalPlanner::SegmentGlobalPlanner():nav_core::BaseGlobalPlanner()
{
    ROS_INFO("Constructing segment_global_planner plugin!");
}

SegmentGlobalPlanner::~SegmentGlobalPlanner()
{
    planner_implementation_.reset();
}

bool SegmentGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (isChildGoalReached()) //update segment_list_
    {
        switchSegment();
    }
    if (!got_first_goal || final_goal_.pose.position.x != goal.pose.position.x
                            || final_goal_.pose.position.y != goal.pose.position.y) //whether the goal is a new one
    {
        if (final_goal_reached_ || !findDistLessThresh(start, segment_list_.front())) //need to start a new navigation
        {
            segment_list_.clear();
            final_goal_ = segment_goal_ = start;
        }
        std::vector<geometry_msgs::PoseStamped> new_plan;
        bool new_plan_result(false);
        new_plan_result = planner_implementation_->makePlan(final_goal_, goal, new_plan); //make a plan

        if (new_plan_result) //add a segment
        {
            addNewSegment(new_plan);
        }
        else //fail to get a new plan
        {
            return false;
        }

        nav_msgs::Path display_path; //display path
        display_path.header.frame_id = global_frame_;
        for (auto& segment : segment_list_)
        {
            std::copy(segment.begin(), segment.end(), std::back_inserter(display_path.poses));
        }
        plan_pub_.publish(display_path);
    }
    if (!segment_list_.empty())
    {
        plan = segment_list_.front(); //current segment as the plan
    }
    else
    {
        plan = last_segment_;
    }
    got_first_goal = true;
    return true;
}

void SegmentGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_INFO("Initializing segment_global_planner plugin!");
    ros::NodeHandle private_nh("~/" + name);
    std::string global_planner_name;
    if (private_nh.getParam("base_global_planner", global_planner_name)) //initialize planner implementation
    {
        try
        {
            planner_implementation_ = bgp_loader_.createInstance(global_planner_name);
            planner_implementation_->initialize(bgp_loader_.getName(global_planner_name), costmap_ros); //initialize real global planner
        }
        catch (const pluginlib::PluginlibException& ex)
        {
            ROS_WARN("The plugin name is not correct. Loading default LineSegment.");
            planner_implementation_ = boost::make_shared<segment_global_planner::LineSegment>();
            planner_implementation_->initialize("LineSegment", costmap_ros); //initialize implementation
        }
    }
    else
    {
        planner_implementation_ = boost::make_shared<segment_global_planner::LineSegment>();
        planner_implementation_->initialize("LineSegment", costmap_ros); //initialize implementation
    }

    plan_pub_=private_nh.advertise<nav_msgs::Path>("plan", 1);
    private_nh.param("child_goal_threshold", goal_threshold_, goal_threshold_);
    private_nh.param("point_interval", point_interval_, point_interval_);
    private_nh.param("threshold_point_on_line", threshold_point_on_line_, threshold_point_on_line_);
    dynamic_config_server_.reset(new dynamic_reconfigure::Server<SegmentGlobalPlannerConfig>(private_nh)); //setup dynamic reconfigure
    dynamic_config_server_->setCallback(boost::bind(&SegmentGlobalPlanner::reconfigureCB, this, _1, _2));
    clear_trajectory_server_=private_nh.advertiseService("clear_trajectory",
                                                         &SegmentGlobalPlanner::clearTrajectoryCB, this); //setup clear trajectory service
    global_frame_ = costmap_ros->getGlobalFrameID();
    return;
}

bool SegmentGlobalPlanner::isChildGoalReached()
{
    if (segment_list_.empty())
    {
        final_goal_reached_ = true;
        return true;
    }
    geometry_msgs::TransformStamped tf_map_to_base = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = tf_map_to_base.transform.translation.x;
    current_pose.pose.position.y = tf_map_to_base.transform.translation.y;
    current_pose.pose.position.z = tf_map_to_base.transform.translation.z;
    current_pose.pose.orientation = tf_map_to_base.transform.rotation;
    if (sq_distance(current_pose, segment_list_.front().back()) < goal_threshold_ * goal_threshold_) //close to segment_goal
    {
        if (segment_list_.size() == 1)
        {
            final_goal_reached_ = true;
        }
        else
        {
            final_goal_reached_ = false;
        }
        return true;
    }
    final_goal_reached_ = false;
    return false;
}

void SegmentGlobalPlanner::reconfigureCB(segment_global_planner::SegmentGlobalPlannerConfig& config, uint32_t level)
{
    ROS_INFO("dynamic_reconfigure updates.");
    threshold_point_on_line_=config.threshold_point_on_line;
    point_interval_=config.point_interval;
    goal_threshold_=config.child_goal_threshold;
    return;
}

bool SegmentGlobalPlanner::clearTrajectoryCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    final_goal_reached_ = true;
    segment_list_.clear();
    nav_msgs::Path display_path;
    display_path.header.frame_id = global_frame_;
    plan_pub_.publish(display_path); //publish an empty path
    ROS_WARN("Trajectory has been cleared!");
    return true;
}

void SegmentGlobalPlanner::clickedPointCB(const geometry_msgs::PointStamped::ConstPtr& ptr)
{
    geometry_msgs::PoseStamped publish_goal;
    publish_goal.pose.position = ptr->point;
    publish_goal.header = ptr->header;
    if (!final_goal_reached_)
    {
        setYaw(&publish_goal, atan2(publish_goal.pose.position.y - final_goal_.pose.position.y,
                                    publish_goal.pose.position.x - final_goal_.pose.position.x)); //set the orientation of the goal vector from the final goal to this one
    }
    else
    {
        try
        {
            geometry_msgs::TransformStamped tf_map_to_base = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
            setYaw(&publish_goal, std::atan2(publish_goal.pose.position.y - tf_map_to_base.transform.translation.y,
                                             publish_goal.pose.position.x - tf_map_to_base.transform.translation.x)); //robot position to new goal
        }
        catch (const tf2::TransformException& ex)
        {
            publish_goal.pose.orientation.w = 1.0; //default orientation
        }
    }
    pose_from_clicked_point_pub_.publish(publish_goal);
}

void SegmentGlobalPlanner::addNewSegment(const std::vector<geometry_msgs::PoseStamped>& new_plan)
{
    if (!segment_list_.empty()) //change the orientation of the last segment
    {
        segment_list_.back().back().pose.orientation = new_plan.front().pose.orientation;
    }
    else
    {
        segment_goal_ = new_plan.back();
    }
    segment_list_.emplace_back(new_plan); //add a new plan to the segment list
    final_goal_ = new_plan.back(); //update final goal
    return;
}

void SegmentGlobalPlanner::switchSegment()
{
    if (segment_list_.empty())
    {
        return;
    }
    last_segment_ = segment_list_.back();
    segment_list_.pop_front();
    if (!segment_list_.empty())
    {
        segment_goal_ = segment_list_.front().back();
    }
    return;
}

bool SegmentGlobalPlanner::findDistLessThresh(const geometry_msgs::PoseStamped& point, const std::vector<geometry_msgs::PoseStamped>& segment)
{
    for (auto itr = segment.begin(); itr != segment.end(); ++itr) //segments are one less than poses
    {
        auto itr_next = itr;
        ++itr_next;
        if (distPointToSegment(point, *itr, *itr_next) < threshold_point_on_line_) //find distance less than threshold
        {
            return true;
        }
    }
    return false;
}
}
