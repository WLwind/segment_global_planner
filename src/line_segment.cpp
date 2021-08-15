#include <list>
#include <tf2/utils.h>
#include <segment_global_planner/common.h>
#include <segment_global_planner/line_segment.h>

namespace segment_global_planner
{
bool LineSegment::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    unsigned int start_x, start_y, end_x, end_y;
    std::list<geometry_msgs::PoseStamped> trajectory;
    trajectory.push_back(start);
    trajectory.push_back(goal);
    if (costmap_ros_->getCostmap()->worldToMap(trajectory.begin()->pose.position.x, trajectory.begin()->pose.position.y,
                                               start_x, start_y)
       && costmap_ros_->getCostmap()->worldToMap((--trajectory.end())->pose.position.x, (--trajectory.end())->pose.position.y,
                                                 end_x, end_y)) //convert the start and goal to map coordinate and confirm that they are in the map
    {
        if (costmap_model_->lineCost(start_x, end_x, start_y, end_y) < 0.0) //lethal
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
    for (auto itr = trajectory.begin(); itr != --trajectory.end(); itr++) //no need to judge the last pose
    {
        auto itr_next = itr;
        ++itr_next;
        double dist_2_point = sq_distance(*itr, *itr_next); //distance square
        if(dist_2_point > (point_interval_ + 0.001) * (point_interval_ + 0.001)) //distance is larger than threshold, need to insert one pose
        {
            dist_2_point = std::sqrt(dist_2_point); //real distance
            double proportion = point_interval_ / dist_2_point;
            geometry_msgs::PoseStamped point_to_insert;
            double& this_x = itr->pose.position.x;
            double& this_y = itr->pose.position.y;
            double& next_x = itr_next->pose.position.x;
            double& next_y = itr_next->pose.position.y;
            point_to_insert.pose.position.x = this_x + (next_x - this_x) * proportion;
            point_to_insert.pose.position.y = this_y + (next_y - this_y) * proportion;
            auto itr_last_point = --trajectory.end(); //last point
            double insert_point_yaw = std::atan2(itr_last_point->pose.position.y - this_y, itr_last_point->pose.position.x - this_x);
            setYaw(&point_to_insert, insert_point_yaw);
            point_to_insert.header.frame_id = goal.header.frame_id;
            trajectory.insert(itr_next, point_to_insert);
        }
    }
    plan.clear();
    std::copy(trajectory.begin(), trajectory.end(), std::back_inserter(plan));
    return true;
}

void LineSegment::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle private_nh("~/" + name);
    private_nh.getParam("point_interval", point_interval_);
    costmap_ros_ = costmap_ros;
    costmap_model_ = std::make_shared<base_local_planner::CostmapModel>(*costmap_ros->getCostmap());//make a costmap model
}
}
