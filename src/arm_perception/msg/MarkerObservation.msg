# One marker seen in ONE camera frame (no averaging, no stillness check): what a controller needs.
int32 id
string dictionary
float64 size                 # metres, the configured (measured) size of the black square
geometry_msgs/Pose pose      # marker centre and orientation in the frame of the header (the camera optical frame)
float64 reprojection_px      # RMS reprojection error of the corners
float64 ambiguity_ratio      # IPPE best / second-best error: close to 1 = the tilt may be the mirrored solution
float64 distance             # optical centre to marker centre [m]
