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
    nav_msgs::Path gui_path;//path to display
    if(m_trajectory_path.size()!=0)
    {
        trimTrajectory(start);
        m_trajectory_path.insert(m_trajectory_path.begin(),start);//push start pose
        m_trajectory_path.push_back(goal);//push goal pose
    }
    else
    {
        ROS_INFO("New trajectory.");
        m_trajectory_path.push_back(start);
        m_trajectory_path.push_back(goal);
    }
    insertPoints();//fill the interval of long segments
    plan=m_trajectory_path;
    if(current_goal.pose.position.x!=goal.pose.position.x||current_goal.pose.position.y!=goal.pose.position.y)
    {
        current_goal.pose.position.x=goal.pose.position.x;//update display
        current_goal.pose.position.y=goal.pose.position.y;
        for(auto trj_pose:m_trajectory_path)//fill the display path
        {
            gui_path.poses.push_back(trj_pose);
        }
        gui_path.header.frame_id=global_frame_;
        plan_pub_.publish(gui_path);
    }
    return true;
}

void SegmentGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_INFO("Initializing clear_all_costmaps_recovery plugin!");
    ros::NodeHandle private_nh("~/" + name);
    plan_pub_=private_nh.advertise<nav_msgs::Path>("plan", 1);
    global_frame_ = costmap_ros->getGlobalFrameID();
    m_trajectory_path.clear();
    return;
}

void SegmentGlobalPlanner::trimTrajectory(const geometry_msgs::PoseStamped& start)
{
    double p2p,p2l,min_p2l=9999.9;
    for(int i=0;i<m_trajectory_path.size();i++)//segments are one less than poses 
    {
        p2p=sq_distance(start,m_trajectory_path[i]);
        p2l=distPointToSegment(start,m_trajectory_path[i],m_trajectory_path[i+1]);
        if(p2p<m_threshold_point_on_line*m_threshold_point_on_line||p2l<m_threshold_point_on_line)//distance shorter than the threshold
        {
            p2p=sqrt(p2p);
            if(std::min(p2p,p2l)<=min_p2l)//find min distance
            {
                min_p2l=std::min(p2p,p2l);
            }
            else//when distance becomes larger
            {
                ROS_INFO("Update trajectory.");
                m_trajectory_path.erase(m_trajectory_path.begin(),m_trajectory_path.begin()+i+1);//erase the points behind the robot [begin,i+1)
                break;
            }
        }
        else if(i==m_trajectory_path.size()-1)//the start point is far from all segments
        {
            ROS_INFO("Clear trajectory.");
            m_trajectory_path.clear();
        }
    }
    
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
    double s = sqrt(HalfC*(HalfC-s1s2)*(HalfC-p0s1)*(HalfC-p0s2));//Heron's formula
    return 2.0*s/s1s2;//hight
}

double SegmentGlobalPlanner::distPointToPoint(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    return sqrt(sq_distance(p1,p2));
}

double SegmentGlobalPlanner::sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return dx*dx +dy*dy;
}

void SegmentGlobalPlanner::insertPoints()
{
    bool last_corner=false;
    for(int i=0;i<m_trajectory_path.size()-1;i++)//no need to judge the last pose
    {
        double dist_2_point=sq_distance(m_trajectory_path[i],m_trajectory_path[i+1]);
        if(dist_2_point>(m_point_interval+0.01)*(m_point_interval+0.01))
        {
            dist_2_point=sqrt(dist_2_point);
            double proportion=m_point_interval/dist_2_point;
            geometry_msgs::PoseStamped point_to_insert;
            point_to_insert.pose.position.x=m_trajectory_path[i].pose.position.x+(m_trajectory_path[i+1].pose.position.x-m_trajectory_path[i].pose.position.x)*proportion;
            point_to_insert.pose.position.y=m_trajectory_path[i].pose.position.y+(m_trajectory_path[i+1].pose.position.y-m_trajectory_path[i].pose.position.y)*proportion;
            point_to_insert.pose.orientation=m_trajectory_path[i].pose.orientation;
            point_to_insert.header.frame_id=global_frame_;
            m_trajectory_path.insert(m_trajectory_path.begin()+i+1,point_to_insert);
            if(last_corner==false&&i>1)//reshape the last corner
            {
                m_trajectory_path.erase(m_trajectory_path.begin()+i);
                i--;
                last_corner=true;
                std::cout<<"last_corner"<<std::endl;
            }

        }
    }
    return;
}
}
