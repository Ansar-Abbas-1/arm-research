# One detected object in one camera frame, with its position on the floor plane (the camera has no depth).
string class_name
int32 class_id
float32 confidence
float32[4] box               # x1 y1 x2 y2 in the original image pixels
bool valid                   # the ray through the box centre reaches the plane at the class's grasp height within range
geometry_msgs/Point position # the box centre on that plane, in the base frame (meaningful only when valid)
float64 plane_z              # z of the plane used (floor z + the class's grasp height) in the base frame
string frame                 # the TF frame published for it (object_<class>_<rank>), "" when not valid
