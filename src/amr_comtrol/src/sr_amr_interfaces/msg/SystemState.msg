std_msgs/Header header

int8 LOCATION_STATE_ZERO = 0x00
int8 LOCATION_STATE_NONE = 0x01
int8 LOCATION_STATE_INITIALING = 0x02       # initializing localization    
int8 LOCATION_STATE_RUNNING = 0x03          # successfully localized
int8 LOCATION_STATE_RELOCATING = 0x04       # lost localization, trying to relocate
int8 LOCATION_STATE_ERROR = 0x05            # cannot localize, maybe env has changed a lot
int8 LOCATION_STATE_UNKNOWN = 0x20    

int8 location_state                         # e.g., LOCATION_STATE_RUNNING 
uint8 location_confidence_score             # confidence score of current localization (0 to 100)

geometry_msgs/PoseStamped current_pose      # map coordinates, (x, y, z, roll, pitch, yaw)

bool estop_active                           # true if emergency stop is engaged

bool remote_control_active                  # true if remote control is active 
bool remote_control_oba_active              # true if remote control over obstacle avoidance is active

float32 linear_velocity_x                   # (m/s)
float32 linear_velocity_y                   # (m/s)
float32 angular_velocity                    # (rad/s)

string current_map_name                     # "NO_MAP" or "" if no map is loaded, else the name of the map 
uint64 current_station_id                   # 0 if not at a station, else the ID of the station 

bool executing_movement_task                # true if currently executing a movement task
bool movement_task_manual_paused            # true if the movement task is being manual paused
bool movement_task_obstacle_paused          # true if the movement task is paused due to obstacle detection
