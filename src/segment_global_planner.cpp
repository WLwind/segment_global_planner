#include <tf/transform_datatypes.h>
#include <segment_global_planner/segment_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <cmath>
#include <algorithm>

PLUGINLIB_EXPORT_CLASS(segment_global_planner::SegmentGlobalPlanner, nav_core::BaseGlobalPlanner)//register plugin

namespace segment_global_planner
{
SegmentGlobalPlanner::SegmentGlobalPlanner():nav_core::BaseGlobalPlanner()
{
    ROS_INFO("Constructing segment_global_planner plugin!");
}

bool SegmentGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    m_current_pose=start;
    bool new_goal=!m_got_first_goal||m_current_goal.pose.position.x!=goal.pose.position.x||m_current_goal.pose.position.y!=goal.pose.position.y||m_current_goal.header.frame_id!=global_frame_;//whether the goal is a new one
    if(new_goal)
    {
        if(!m_child_goals.empty())
        {
            geometry_msgs::PoseStamped* old_last_child_goal=&m_child_goals.back();
            setAngle(old_last_child_goal,atan2(goal.pose.position.y-old_last_child_goal->pose.position.y,goal.pose.position.x-old_last_child_goal->pose.position.x));//set the orientation of the old goal vector from it to the new goal
        }
        else
        {
            setAngle(&m_segment_goal,atan2(goal.pose.position.y-m_segment_goal.pose.position.y,goal.pose.position.x-m_segment_goal.pose.position.x));//set the orientation of the segment goal vector from it to the new goal
        }
        m_child_goals.push(goal);//add new child goal
        m_current_goal=goal;
    }
    if(isChildGoalReached())//reaches child goals, need to switch to next segment goal
    {
        ROS_INFO("Reached child goal.");
        geometry_msgs::PoseStamped last_child_goal;
        last_child_goal.header.frame_id="empty";
        if(!m_trajectory_path.empty())
        {
            m_trajectory_path.clear();
        }
        if(!m_child_goals.empty())
        {
            if(m_got_first_goal)
            {
                last_child_goal=m_segment_goal;
            }
            m_segment_goal=m_child_goals.front();//update segment goal
            m_child_goals.pop();
        }
        m_trajectory_path.push_back(last_child_goal.header.frame_id!="empty"?last_child_goal:m_current_pose);
        m_trajectory_path.push_back(m_segment_goal);
        insertPoints();//fill the intervals of long segments with poses
    }
    else//doesn't reach child goals
    {
        if(m_trajectory_path.size()>=2)
        {
            trimTrajectory(start);//trim segment
        }
        if(m_trajectory_path.size()<2)
        {
            if(new_goal&&m_trajectory_path.empty())//robot is far from unreached segment, it was trimmed totally
            {
                while(!m_child_goals.empty())//clear the queue
                {
                    m_child_goals.pop();
                }
                m_segment_goal=goal;
            }
            ROS_INFO("New trajectory.");
            if(!m_trajectory_path.empty())
            {
                m_trajectory_path.clear();
            }
            m_trajectory_path.push_back(start);
            m_trajectory_path.push_back(m_segment_goal);
            insertPoints();//fill the intervals of long segments with poses
        }
    }
    m_got_first_goal=true;
    if(!plan.empty())
    {
        plan.clear();
    }
    std::copy(m_trajectory_path.begin(),m_trajectory_path.end(),std::back_inserter(plan));//copy the trajectory to plan

    if(new_goal)//for display
    {
        nav_msgs::Path gui_path;//path to display
        gui_path.poses.push_back(m_current_pose);
        gui_path.poses.push_back(m_segment_goal);
        geometry_msgs::PoseStamped pose_to_display;
        int queue_size=m_child_goals.size();
        if(queue_size!=0)
        {
            for(int i=0;i<queue_size;i++)//fill the display path
            {
                pose_to_display=m_child_goals.front();
                gui_path.poses.push_back(pose_to_display);
                m_child_goals.push(pose_to_display);//put it at the end
                m_child_goals.pop();
            }
        }
        gui_path.header.frame_id=global_frame_;
        plan_pub_.publish(gui_path);
    }
    return true;
}

void SegmentGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_INFO("Initializing segment_global_planner plugin!");
    ros::NodeHandle private_nh("~/" + name);
    plan_pub_=private_nh.advertise<nav_msgs::Path>("plan", 1);
    m_dynamic_config_server.reset(new dynamic_reconfigure::Server<SegmentGlobalPlannerConfig>(private_nh));//setup dynamic reconfigure
    m_dynamic_config_server->setCallback(boost::bind(&SegmentGlobalPlanner::reconfigureCB, this, _1, _2));
    m_clear_trajectory_server=private_nh.advertiseService("clear_trajectory",&SegmentGlobalPlanner::clearTrajectoryCB,this);//setup clear trajectory service
    global_frame_ = costmap_ros->getGlobalFrameID();
    return;
}

void SegmentGlobalPlanner::trimTrajectory(const geometry_msgs::PoseStamped& start)
{
    double p2p,p2l,min_p2l=9999.9;
    for(auto itr=m_trajectory_path.begin();itr!=m_trajectory_path.end();itr++)//segments are one less than poses 
    {
        if(itr==--m_trajectory_path.end())//the start point is far from all segments
        {
            ROS_INFO("Clear trajectory.");
            m_trajectory_path.clear();
            break;
        }
        p2p=sq_distance(start,*itr);
        auto itr_next=++itr;
        itr--;
        p2l=distPointToSegment(start,*itr,*itr_next);
        if(p2p<m_threshold_point_on_line*m_threshold_point_on_line||p2l<m_threshold_point_on_line)//distance is shorter than the threshold
        {
            p2p=std::sqrt(p2p);
            if(std::min(p2p,p2l)<min_p2l)//find min distance
            {
                min_p2l=std::min(p2p,p2l);
            }
            else//when distance becomes larger
            {
                ROS_INFO("Update trajectory.");
                m_trajectory_path.erase(m_trajectory_path.begin(),--itr);//erase the points behind the robot [begin,i-1)
                itr++;
                break;
            }
        }
    }
    return;
}

double SegmentGlobalPlanner::distPointToSegment(const geometry_msgs::PoseStamped& p0,const geometry_msgs::PoseStamped& s1, const geometry_msgs::PoseStamped& s2)
{
    double p0s1,p0s2,s1s2;
    p0s1=distPointToPoint(p0,s1);
    p0s2=distPointToPoint(p0,s2);
    s1s2=distPointToPoint(s1,s2);
    if(p0s2*p0s2-p0s1*p0s1-s1s2*s1s2>0||p0s1*p0s1-p0s2*p0s2-s1s2*s1s2>0)//The Law of Cosines
    {
        return 999.9;//the point p0 is not between point s1 and point s2
    }
    double HalfC=(p0s1+p0s2+s1s2)/2.0;//half perimeter
    double s = std::sqrt(HalfC*(HalfC-s1s2)*(HalfC-p0s1)*(HalfC-p0s2));//Heron's formula
    return 2.0*s/s1s2;//hight
}

double SegmentGlobalPlanner::distPointToPoint(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    return std::sqrt(sq_distance(p1,p2));
}

double SegmentGlobalPlanner::sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return dx*dx +dy*dy;
}

void SegmentGlobalPlanner::insertPoints()
{
    for(auto itr=m_trajectory_path.begin();itr!=--m_trajectory_path.end();itr++)//no need to judge the last pose
    {
        auto itr_next=++itr;
        itr--;
        double dist_2_point=sq_distance(*itr,*itr_next);
        if(dist_2_point>(m_point_interval+0.01)*(m_point_interval+0.01))//need to insert one pose
        {
            dist_2_point=std::sqrt(dist_2_point);
            double proportion=m_point_interval/dist_2_point;
            geometry_msgs::PoseStamped point_to_insert;
            point_to_insert.pose.position.x=itr->pose.position.x+(itr_next->pose.position.x-itr->pose.position.x)*proportion;
            point_to_insert.pose.position.y=itr->pose.position.y+(itr_next->pose.position.y-itr->pose.position.y)*proportion;
            auto itr_last_point=--m_trajectory_path.end();//last point
            setAngle(&point_to_insert,atan2(itr_last_point->pose.position.y-itr->pose.position.y,itr_last_point->pose.position.x-itr->pose.position.x));
            point_to_insert.header.frame_id=global_frame_;
            m_trajectory_path.insert(itr_next,point_to_insert);
        }
    }
    return;
}

bool SegmentGlobalPlanner::isChildGoalReached()
{
    if(!m_got_first_goal||sq_distance(m_current_pose,m_segment_goal)<=m_goal_threshold*m_goal_threshold)
    {
        return true;
    }
    return false;
}

void SegmentGlobalPlanner::setAngle(geometry_msgs::PoseStamped* pose, double angle)
{
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle);
}

void SegmentGlobalPlanner::reconfigureCB(segment_global_planner::SegmentGlobalPlannerConfig& config, uint32_t level)
{
    ROS_INFO("dynamic_reconfigure updates.");
    m_threshold_point_on_line=config.threshold_point_on_line;
    m_point_interval=config.point_interval;
    m_goal_threshold=config.child_goal_threshold;
    return;
}

bool SegmentGlobalPlanner::clearTrajectoryCB(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    m_current_goal.pose.position.x=m_current_goal.pose.position.y=0.0;
    m_current_goal.header.frame_id="empty";
    while(!m_child_goals.empty())//clear the queue
    {
        m_child_goals.pop();
    }
    m_trajectory_path.clear();//clear trajectory
    m_got_first_goal=false;//reset the first time mark
    ROS_WARN("Trajectory has been cleared!");
    return true;
}

void SegmentGlobalPlanner::clickedPointCB(const geometry_msgs::PointStamped::ConstPtr& ptr)
{
    geometry_msgs::PoseStamped publish_goal;
    publish_goal.pose.position=ptr->point;
    publish_goal.header=ptr->header;
    if(!m_child_goals.empty())
    {
        geometry_msgs::PoseStamped last_child_goal=m_child_goals.back();
        setAngle(&publish_goal,atan2(publish_goal.pose.position.y-last_child_goal.pose.position.y,publish_goal.pose.position.x-last_child_goal.pose.position.x));//set the orientation of the goal vector from the last goal to this one
    }
    else if(m_got_first_goal)//current child goal is the final goal
    {
        if(!isChildGoalReached())
        {
            setAngle(&publish_goal,atan2(publish_goal.pose.position.y-m_segment_goal.pose.position.y,publish_goal.pose.position.x-m_segment_goal.pose.position.x));//set the orientation of the goal vector from current robot position to the new goal
        }
        else
        {
            setAngle(&publish_goal,atan2(publish_goal.pose.position.y-m_current_pose.pose.position.y,publish_goal.pose.position.x-m_current_pose.pose.position.x));
        }
    }
    else
    {
        publish_goal.pose.orientation=tf::createQuaternionMsgFromYaw(0.0);//default orientation
    }
    m_pose_from_clicked_point_pub.publish(publish_goal);
}

}//namespace end
