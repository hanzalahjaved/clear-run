# =============================================================================
# FodLocation.msg
# Custom ROS 2 message for FOD (Foreign Object Debris) location data
# =============================================================================
# This message is published by the UAV when FOD is detected and centered,
# and consumed by the UGV for navigation to the debris location.
# =============================================================================

# Header with timestamp and frame_id
std_msgs/Header header

# Unique identifier for this FOD detection
uint32 fod_id

# GPS Coordinates (WGS84)
float64 latitude                    # Decimal degrees (-90 to 90)
float64 longitude                   # Decimal degrees (-180 to 180)
float64 altitude                    # Meters above sea level (WGS84 ellipsoid)

# Local coordinates in map frame (for Nav2)
geometry_msgs/Point position        # x, y, z in meters (map frame)

# Detection metadata
string object_class                 # YOLO class label (e.g., "bolt", "rivet", "fragment")
float32 confidence                  # Detection confidence (0.0 to 1.0)
float32 estimated_size              # Estimated size in centimeters

# Image bounding box (for reference)
uint32 bbox_x                       # Top-left x pixel
uint32 bbox_y                       # Top-left y pixel  
uint32 bbox_width                   # Width in pixels
uint32 bbox_height                  # Height in pixels

# Detection source
uint8 SOURCE_RGB = 0
uint8 SOURCE_THERMAL = 1
uint8 SOURCE_FUSED = 2
uint8 detection_source              # Which sensor detected this FOD

# Status flags
bool is_centered                    # True if UAV has centered on this FOD
bool is_collected                   # True if UGV has collected this FOD
bool requires_manual                # True if automated collection failed
