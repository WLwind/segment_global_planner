#include <cmath>
#include <algorithm>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/footprint.h>
#include <segment_global_planner/common.h>
#include <segment_global_planner/line_segment.h>
#include <segment_global_planner/segment_global_planner.h>

PLUGINLIB_EXPORT_CLASS(segment_global_planner::SegmentGlobalPlanner, nav_core::BaseGlobalPlanner)//register plugin

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

namespace segment_global_planner
{
SegmentGlobalPlanner::SegmentGlobalPlanner() : nav_core::BaseGlobalPlanner()
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
    bool new_plan = false;
    if (!got_first_goal || final_goal_.pose.position.x != goal.pose.position.x
                        || final_goal_.pose.position.y != goal.pose.position.y) //whether the goal is a new one
    {
        new_plan = true;
        if (final_goal_reached_ || !findDistLessThresh(start, segment_list_.front())) //need to start a new navigation
        {
            segment_list_.clear();
            final_goal_ = segment_goal_ = start;
        }
        std::vector<geometry_msgs::PoseStamped> new_plan;
        if (planner_implementation_->makePlan(final_goal_, goal, new_plan) && !new_plan.empty()) //make a plan
        {
            addNewSegment(new_plan); //add a segment
        }
        else //fail to make a new plan
        {
            return false;
        }
    }

    if (!segment_list_.empty()) //fill the plan reference
    {
        bool replan_segment = false;
        if (!replan_)
        {
            plan = segment_list_.front(); //current segment as the plan
        }
        else //request replanning
        {
            if (segment_list_.size() == 1 && new_plan) //no need to replan
            {
                plan = segment_list_.front();
            }
            else
            {
                std::vector<geometry_msgs::PoseStamped> replan_path;
                if (planner_implementation_->makePlan(start, segment_goal_, replan_path) && !replan_path.empty()) //replan
                {
                    plan = segment_list_.front() = replan_path; //replace first segment
                    replan_segment = true;
                    ROS_INFO("Current segment has been replaned.");
                }
                else //fail to replan
                {
                    return false;
                }
            }
            replan_ = false;
        }

        if (new_plan || replan_segment) //display path
        {
           displayPath();
        }
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
    clear_trajectory_server_ = private_nh.advertiseService("clear_trajectory",
                                                           &SegmentGlobalPlanner::clearTrajectoryCB, this); //setup clear trajectory service
    replan_sub_ = private_nh.subscribe("/replan_segment", 5, &SegmentGlobalPlanner::replanCB, this);
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
    clicked_a_new_goal_ = true;
    pose_from_clicked_point_pub_.publish(publish_goal);
}

void SegmentGlobalPlanner::addNewSegment(std::vector<geometry_msgs::PoseStamped>& new_plan)
{
    int point_number_to_estimate_orientation = std::min<int>(5, new_plan.size());
    if (clicked_a_new_goal_) //need to set new_plan's goal orientation
    {
        if (point_number_to_estimate_orientation >= 2) //estimate the orientation with the last several points of new_plan
        {
            double estimated_yaw = std::atan2(new_plan.back().pose.position.y - new_plan[new_plan.size() - point_number_to_estimate_orientation].pose.position.y,
                                              new_plan.back().pose.position.x - new_plan[new_plan.size() - point_number_to_estimate_orientation].pose.position.x);
            tf2::Quaternion tf2q(tf2::Vector3(0.0, 0.0, 1.0), estimated_yaw);
            new_plan.back().pose.orientation = tf2::toMsg(tf2q);
        }
        clicked_a_new_goal_ = false;
    }
    if (!segment_list_.empty()) //change the orientation of the goal of the last segment according to new_plan
    {
        if (point_number_to_estimate_orientation >= 2) //estimate the orientation with the first several points of new_plan
        {
            double estimated_yaw = std::atan2(new_plan[point_number_to_estimate_orientation - 1].pose.position.y - new_plan.front().pose.position.y,
                                              new_plan[point_number_to_estimate_orientation - 1].pose.position.x - new_plan.front().pose.position.x);
            tf2::Quaternion tf2q(tf2::Vector3(0.0, 0.0, 1.0), estimated_yaw);
            segment_list_.back().back().pose.orientation = tf2::toMsg(tf2q);
        }
        else //set the orientation with first pose orientation of new_plan
        {
            segment_list_.back().back().pose.orientation = new_plan.front().pose.orientation;
        }
    }
    else
    {
        segment_goal_ = new_plan.back();
    }
    segment_list_.push_back(new_plan); //add a new plan to the segment list
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

void SegmentGlobalPlanner::replanCB(const std_msgs::Bool::ConstPtr& ptr)
{
    if (ptr->data)
    {
        replan_ = true;
        ROS_INFO("Ready to replan segment.");
    }
    return;
}

void SegmentGlobalPlanner::displayPath()
{
    nav_msgs::Path display_path; //path to be published
    display_path.header.frame_id = global_frame_;
    for (auto& segment : segment_list_)
    {
        std::copy(segment.begin(), segment.end(), std::back_inserter(display_path.poses));
    }
    plan_pub_.publish(display_path);
    return;
}
}
