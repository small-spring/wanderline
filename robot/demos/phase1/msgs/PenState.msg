# Phase1 Pen State Message
# Contains all pen-related information in synchronized format

# Position is described in "robot base frame" (= "base_link")

# Pen tip position (actual drawing point)
geometry_msgs/Point tip_position

# Pen base position (tool flange)
geometry_msgs/Point base_position

# Contact state with canvas
bool is_contact
