# Command sent from BT/autonomy to ECE excavation hardware
# Commands: DIG_START, DIG_STOP, DUMP_OPEN, DUMP_CLOSE, STOW

std_msgs/Header header
string command          # DIG_START | DIG_STOP | DUMP_OPEN | DUMP_CLOSE | STOW
float32 parameter       # Optional parameter (e.g. dig duration, dig aggression)
