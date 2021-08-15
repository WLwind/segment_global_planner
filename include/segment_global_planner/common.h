#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

namespace segment_global_planner
{
/**
 * @brief distance square
 * @param p1 pose1
 * @param p2 pose2
 * @return distance square value
 */
inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return dx * dx + dy * dy;
}

/**
 * @brief Set yaw for a pose
 * @param pose Pose to be set yaw
 * @param angle yaw
 */
inline void setYaw(geometry_msgs::PoseStamped* pose, double angle)
{
    tf2::Quaternion tf2q;
    tf2q.setRPY(0.0,0.0,angle);
    pose->pose.orientation = tf2::toMsg(tf2q);
}
}
