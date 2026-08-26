int8 PATH_LINE = 1                      # straight line path
int8 PATH_BEZIER = 3                    # cubic Bezier curve path
int8 PATH_ROTATE = 4                    # in-place rotation

int8 DIRECTION_FORWARD = 1              # which means 0 degree forward 
int8 DIRECTION_BACKWARD = 2             # which means 180 degree turn
int8 DIRECTION_PATH_LEFT = 3            # which means -90 degree turn
int8 DIRECTION_PATH_RIGHT = 4           # which means +90 degree turn
int8 DIRECTION_PATH_CUSTOM = 16         # you should use this with `orientation` property

int8 type                               # PATH_LINE

float64 sx                              # starting x coordinate in map frame
float64 sy                              # starting y coordinate in map frame

float64 cx                              # control point 1 x coordinate in map frame (for PATH_BEZIER type), you should set 0 if PATH_LINE type
float64 cy                              # control point 1 y coordinate in map frame (for PATH_BEZIER type), you should set 0 if PATH_LINE type

float64 dx                              # control point 2 x coordinate in map frame (for PATH_BEZIER type), you should set 0 if PATH_LINE type
float64 dy                              # control point 2 y coordinate in map frame (for PATH_BEZIER type), you should set 0 if PATH_LINE type

float64 ex                              # ending x coordinate in map frame
float64 ey                              # ending y coordinate in map frame

float64 rotate_angle                    # in degrees, should be -360 to 360, positive for counter-clockwise, negative for clockwise rotation 

int8 direction                          # DIRECTION_FORWARD

float64 orientation                     # in degrees, should be 0 to 360, only used when direction is DIRECTION_PATH_CUSTOM and PATH_LINE

float64 limit_v                         # (m/s)   linear velocity limit
float64 limit_w                         # (rad/s) angular velocity limit
