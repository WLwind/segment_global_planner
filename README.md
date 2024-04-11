# segment_global_planner Version 3
A ROS global planner plugin for segments tracking.  
This plugin allows you to set several segments as one global plan with rviz tools "2D Nav Goal" or "Publish Point" easily. The segments are not just line segments in a narrow sense, but also segments of path that generated from other global planners(e.g. A*).
## Setup plugin
Modify your launch file of move_base. Set the value of rosparam "/move_base/base_global_planner" to **segment_global_planner/SegmentGlobalPlanner**. Make sure the name space is correct. There are two brief files (yaml and xml) for examples in param folder.
## Parameters
1. /move_base/SegmentGlobalPlanner/base_global_planner  
The real global planner implementation to ganerate path segments.You can feel free to set any general global planner plugins like **global_planner/GlobalPlanner**. If you don't set this parameter then line segement generator will be implemented and the performance is just like the previous version of segment_global_planner.
2. /move_base/SegmentGlobalPlanner/threshold_point_on_line  
Threshold that robot is considered on the tracking line within.(dynamic_reconfigure) If this value is too low, the robot may often derail and make a plan directly to your final goal.  
3. /move_base/LineSegment/point_interval  
The max interval of 2 points on a segment.(only for LineSegment)  
4. /move_base/SegmentGlobalPlanner/child_goal_threshold  
Threshold that robot is considered reaching the goal within.(dynamic_reconfigure) You should make this larger than the tolerance of local planner and set a not too low frequency for global planner (5.0Hz is adequate for a small robot like Turtlebot3). If this value or the frequency of global planner is too low, the robot may accidentally finish the navigation action at a child goal.  
## Service
* /move_base/SegmentGlobalPlanner/clear_trajectory  
A simple service to clear current trajectory of segments. e.g.  
`rosservice call /move_base/SegmentGlobalPlanner/clear_trajectory "{}"`
