# ObjectDetection.msg
string name           # e.g. "red_cylinder"
string colour          # e.g. "red"
string type           # e.g. "cylinder" or "cube"
string meaning        # e.g. "fire" "survivor" "medical_kit"
float64 x             # world position x
float64 y             # world position y
bool detected         # is it currently visible