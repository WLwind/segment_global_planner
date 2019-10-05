# segment_global_planner
A ROS global planner plugin for segments tracking.  
This plugin sets a line segment global plan directly to the goal. And if you put a new goal, the planner will add a new segment from the last goal to the new one. You can use this planner to set goals one by one.  
## Setup plugin
Modify your launch file of move_base.Set the value of rosparam /move_base/base_global_planner to segment_global_planner/SegmentGlobalPlanner. Make sure the name space is correct. There are two brief files (yaml and xml) for examples in param folder.  
## Parameters
1. /move_base/SegmentGlobalPlanner/threshold_point_on_line  
Threshold that robot is considered on the tracking line within.(dynamic_reconfigure)  
2. /move_base/SegmentGlobalPlanner/point_interval  
The max interval of 2 points on a segment.(dynamic_reconfigure)  
3. /move_base/SegmentGlobalPlanner/child_goal_threshold  
Threshold that robot is considered reaching the goal within.(dynamic_reconfigure)  
## Service
* /move_base/SegmentGlobalPlanner/clear_trajectory  
A simple service to clear current trajectory of segments.e.g.  
`rosservice call /move_base/SegmentGlobalPlanner/clear_trajectory "{}"`
