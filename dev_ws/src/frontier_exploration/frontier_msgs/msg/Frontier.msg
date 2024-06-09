float64 unique_id
uint32 size
float64 min_distance
geometry_msgs/Point goal_point
geometry_msgs/Quaternion best_orientation # this is not filled during search but after computation. It's left blank in case of greedy and random.