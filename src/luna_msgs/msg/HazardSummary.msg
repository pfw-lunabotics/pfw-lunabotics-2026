# HazardSummary.msg
# Published by hazard_pipeline at 8 Hz on /perception/hazard_summary
# Used by BT nodes to decide speed, clearance, and recovery actions

std_msgs/Header header

# Closest obstacle distance (XY plane, from robot origin)
# -1.0 if no obstacles detected
float32 min_distance_to_obstacle

# True if any obstacle falls within the forward travel corridor
# Corridor: x in [0.2, 2.0]m, y in [-robot_half_width, +robot_half_width]
bool path_blocked_ahead

# Quality of ground segmentation (RANSAC inlier ratio, 0.0 - 1.0)
# Low confidence = degraded terrain, BT should slow down
float32 confidence

# Point counts
int32 obstacle_count   # total unified obstacle points
int32 rock_count       # points classified as rocks (above ground)
int32 crater_count     # points classified as craters (inverted below ground)
