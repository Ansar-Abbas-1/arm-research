Header header                # stamp and frame of the image
ObjectDetection[] detections # best first
float64 inference_ms         # preprocessing + model + decoding
float64 floor_z              # floor z in the base frame used for this frame
string floor_source          # 'marker_0' or 'param'
