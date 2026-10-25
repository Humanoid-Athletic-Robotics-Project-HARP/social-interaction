# One person the robot can perceive, in the robot's body frame.

string id             # stable across frames while the person is tracked
float32 bearing       # radians, 0 = straight ahead, positive = to the robot's left
float32 distance      # meters
bool facing_robot
