# Dynamic parameters published by AdaptiveTuner node
# BT and Nav2 nodes read these to adjust behavior during the run

std_msgs/Header header
float32 nav_speed_factor        # 0.4 - 1.0 (multiplier on max velocity)
float32 clearance_margin        # 0.2 - 0.6 m (inflation/buffer around obstacles)
float32 dig_duration            # seconds (bounded by ME limits)
float32 dump_shake_duration     # seconds (bounded)
bool caution_mode               # true = reduced speed, increased clearance
