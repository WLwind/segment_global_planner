#include <cmath>
#include <algorithm>
#include <tf2/utils.h>
#include <segment_global_planner/segment_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/footprint.h>

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
        m_feasibility=insertPoints();//fill the intervals of long segments with poses
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
            m_feasibility=insertPoints();//fill the intervals of long segments with poses
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
    return m_feasibility;
}

void SegmentGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_INFO("Initializing segment_global_planner plugin!");
    ros::NodeHandle private_nh("~/" + name);
    plan_pub_=private_nh.advertise<nav_msgs::Path>("plan", 1);
    private_nh.param("child_goal_threshold",m_goal_threshold,m_goal_threshold);
    private_nh.param("point_interval",m_point_interval,m_point_interval);
    private_nh.param("threshold_point_on_line",m_threshold_point_on_line,m_threshold_point_on_line);
    m_dynamic_config_server.reset(new dynamic_reconfigure::Server<SegmentGlobalPlannerConfig>(private_nh));//setup dynamic reconfigure
    m_dynamic_config_server->setCallback(boost::bind(&SegmentGlobalPlanner::reconfigureCB, this, _1, _2));
    m_clear_trajectory_server=private_nh.advertiseService("clear_trajectory",&SegmentGlobalPlanner::clearTrajectoryCB,this);//setup clear trajectory service
    m_costmap_ros=costmap_ros;
    global_frame_ = m_costmap_ros->getGlobalFrameID();
    m_costmap_model=std::make_shared<base_local_planner::CostmapModel>(*m_costmap_ros->getCostmap());//make a costmap model
    return;
}

void SegmentGlobalPlanner::trimTrajectory(const geometry_msgs::PoseStamped& start)
{
    double p2l,min_p2l=9999.9;
    for(auto itr=m_trajectory_path.begin();itr!=m_trajectory_path.end();itr++)//segments are one less than poses 
    {
        if(itr==--m_trajectory_path.end())//the start point is far from all segments
        {
            ROS_INFO("Clear trajectory.");
            m_trajectory_path.clear();
            break;
        }
        auto itr_next=++itr;
        itr--;
        p2l=distPointToSegment(start,*itr,*itr_next);
        if(p2l<m_threshold_point_on_line&&p2l<min_p2l)//find min distance
        {
            min_p2l=p2l;
        }
        else if(min_p2l<m_threshold_point_on_line&&p2l>=min_p2l)//when distance becomes larger
        {
            ROS_INFO("Update trajectory.");
            m_trajectory_path.erase(m_trajectory_path.begin(),--itr);//erase the points behind the robot [begin,i-1)
            itr++;
            break;
        }
    }
    return;
}

double SegmentGlobalPlanner::distPointToSegment(const geometry_msgs::PoseStamped& p0,const geometry_msgs::PoseStamped& s1, const geometry_msgs::PoseStamped& s2)
{
    double p0s1[2]{s1.pose.position.x-p0.pose.position.x,s1.pose.position.y-p0.pose.position.y};//vectors
    double p0s2[2]{s2.pose.position.x-p0.pose.position.x,s2.pose.position.y-p0.pose.position.y};
    double s1s2[2]{s2.pose.position.x-s1.pose.position.x,s2.pose.position.y-s1.pose.position.y};
    if(s1s2[0]*p0s1[0]+s1s2[1]*p0s1[1]>0||s1s2[0]*p0s2[0]+s1s2[1]*p0s2[1]<0)//dot product, obtuse angle
    {
        return 999.9;//the point p0 is not between point s1 and point s2
    }
    double A=s1.pose.position.y-s2.pose.position.y;//parameters of linear equation : Ax+By+C=0
    double B=s2.pose.position.x-s1.pose.position.x;
    double C=s1.pose.position.x*s2.pose.position.y-s1.pose.position.y*s2.pose.position.x;
    return (A*p0.pose.position.x+B*p0.pose.position.y+C)/std::sqrt(A*A+B*B);//distance form p0 to line s1s2 : |Ax+By+C|/√(A²+B²)
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

bool SegmentGlobalPlanner::insertPoints()
{
    unsigned int start_x,start_y,end_x,end_y;
    if(m_costmap_ros->getCostmap()->worldToMap(m_trajectory_path.begin()->pose.position.x,m_trajectory_path.begin()->pose.position.y,start_x,start_y)&&m_costmap_ros->getCostmap()->worldToMap((--m_trajectory_path.end())->pose.position.x,(--m_trajectory_path.end())->pose.position.y,end_x,end_y))
    {
        if(m_costmap_model->lineCost(start_x,end_x,start_y,end_y)<0.0)
        {
            ROS_ERROR("Some trajectory points are in lethal obstacle cell.");
            return false;
        }
    }
    else
    {
        ROS_ERROR("Some trajectory points are out of map.");
        return false;
    }
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
            double insert_point_yaw=atan2(itr_last_point->pose.position.y-itr->pose.position.y,itr_last_point->pose.position.x-itr->pose.position.x);
            setAngle(&point_to_insert,insert_point_yaw);
            point_to_insert.header.frame_id=global_frame_;
            m_trajectory_path.insert(itr_next,point_to_insert);
        }
    }
    return true;
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
    tf2::Quaternion tf2q;
    tf2q.setRPY(0.0,0.0,angle);
    pose->pose.orientation = tf2::toMsg(tf2q);
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
    nav_msgs::Path gui_path;
    gui_path.header.frame_id=global_frame_;
    plan_pub_.publish(gui_path);//publish an empty path
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
        publish_goal.pose.orientation.z=1;//default orientation
    }
    m_pose_from_clicked_point_pub.publish(publish_goal);
}

bool SegmentGlobalPlanner::feasibilityChecking()
{
    double feasibility_result;
    tf2::Quaternion tf2q;
    for(auto& each_pose:m_trajectory_path)
    {
        tf2::fromMsg(each_pose.pose.orientation,tf2q);
        double yaw=tf2::getYaw(tf2q);
        feasibility_result=m_costmap_model->footprintCost(each_pose.pose.position.x,each_pose.pose.position.y,yaw,m_costmap_ros->getRobotFootprint(),0.0,0.0);
        if(feasibility_result<0)
        {
            ROS_ERROR("Current segment trajectory is not feasible! error code: %f",feasibility_result);
            return false;
        }
    }
    return true;
}

}//namespace end
