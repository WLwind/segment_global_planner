# segment_global_planner
A ROS global planner plugin for segments tracking.  
This plugin sets a line segment global plan directly to the goal. And if you put a new goal, the planner will add a new segment from the last goal to the new one. You can use this planner to set goals one by ont.