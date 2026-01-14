# =============================================================================
# ScoopCommand.msg
# Command message for the active brush-assisted scoop mechanism
# =============================================================================

std_msgs/Header header

# Command type
uint8 CMD_IDLE = 0
uint8 CMD_DEPLOY = 1          # Lower scoop to ground
uint8 CMD_RETRACT = 2         # Raise scoop
uint8 CMD_BRUSH_START = 3     # Start inward-rotating brushes
uint8 CMD_BRUSH_STOP = 4      # Stop brushes
uint8 CMD_COLLECT = 5         # Full collection sequence
uint8 command

# Optional parameters
float32 brush_speed           # Brush rotation speed (0.0 to 1.0)
float32 scoop_angle           # Scoop tilt angle in degrees
