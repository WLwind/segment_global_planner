#pragma once

#include <nav_core/base_global_planner.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

namespace segment_global_planner
{
class LineSegment : public nav_core::BaseGlobalPlanner
{
public:
    /**
    * @brief Override nav_core::BaseGlobalPlanner::makePlan()
    * @param start The start pose
    * @param goal The goal pose
    * @param plan The plan... filled by the planner
    * @return True if a valid plan was found, false otherwise
    */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;
    /**
    * @brief Override nav_core::BaseGlobalPlanner::initialize()
    * @param name The name of this planner
    * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
    */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
private:
    double point_interval_ { 0.05 }; //distance between two points on a segment
    costmap_2d::Costmap2DROS* costmap_ros_; //points to the costmap2dros
    std::shared_ptr<base_local_planner::CostmapModel> costmap_model_; //costmap model for feasibility checking
};
}
