# Every configured marker found in one image. header.stamp is the IMAGE time, header.frame_id the camera optical frame.
# Published for every processed frame, also with an empty list (so "nothing seen" is observable).
Header header
MarkerObservation[] markers
