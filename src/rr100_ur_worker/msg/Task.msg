uint8 TYPE_REACHING = 0
uint8 TYPE_GRIPPER = 1
uint8 TYPE_PLACEMENT = 2

uint8 ACTION_GRIP = 0
uint8 ACTION_RELEASE = 1
uint8 ACTION_SET = 2

# Message definition
# Task type
uint8 type

# Target pose (ReachingTask, PlacementTask)
geometry_msgs/PoseStamped target

# Trajectory duration (ReachingTask)
float64 duration

# Gripper action (GripperTask
uint8 action
# Gripper position (GripperTask)
float64 gripper_position
