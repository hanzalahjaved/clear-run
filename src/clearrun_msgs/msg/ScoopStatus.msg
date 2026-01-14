# =============================================================================
# ScoopStatus.msg
# Status feedback from the scoop mechanism
# =============================================================================

std_msgs/Header header

# Current state
uint8 STATE_IDLE = 0
uint8 STATE_DEPLOYING = 1
uint8 STATE_DEPLOYED = 2
uint8 STATE_COLLECTING = 3
uint8 STATE_RETRACTING = 4
uint8 STATE_RETRACTED = 5
uint8 STATE_ERROR = 255
uint8 current_state

# Sensor feedback
bool object_detected          # Proximity sensor in scoop
bool bin_full                 # Collection bin capacity
float32 brush_current         # Motor current draw (amps)
float32 scoop_position        # Current position (0.0 = up, 1.0 = down)

# Diagnostics
bool brush_motor_ok
bool scoop_actuator_ok
string error_message
