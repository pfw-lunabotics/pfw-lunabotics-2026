# LocalizationQuality.msg
# Published by localization_quality_monitor at 5 Hz on /localization/quality
# Read by BT guards to decide speed, clearance, and whether to continue autonomy

std_msgs/Header header

# Overall quality status — BT reads this string directly
# Values: "NOMINAL" | "DEGRADED" | "CRITICAL"
string status

# Trace of position covariance (x + y + z diagonal) from EKF
# Higher = less certain about position
# NOMINAL < 0.05 | DEGRADED < 0.25 | CRITICAL >= 0.25
float32 covariance_magnitude

# Data-flow health flags
bool fast_lio_active    # true if /fast_lio/odom received within last 2 seconds
bool ekf_active         # true if /odom received within last 2 seconds

# Human-readable explanation (for logging and telemetry)
string message
