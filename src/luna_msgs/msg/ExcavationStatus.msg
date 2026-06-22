# Status published by ECE excavation hardware (or sim bridge)

std_msgs/Header header
string state            # IDLE | DIGGING | DUMPING | STOWING | ERROR
float32 motor_current   # Motor current draw (amps) for stall detection + adaptive tuning
string gate_position    # OPEN | CLOSED
string error_code       # NONE | STALL | OVERCURRENT | TIMEOUT
