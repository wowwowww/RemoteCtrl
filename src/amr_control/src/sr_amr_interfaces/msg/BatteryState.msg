int8 STATE_NA = 0x0
int8 STATE_CHARING = 0x1
int8 STATE_NO_CHARING = 0x3
int8 STATE_UNKNOWN = 0x20

uint8 remaining_percentage       # battery level in percentage (0 to 100)
int32 remaining_capacity         # remaining capacity in mAh
uint32 remaining_time            # estimated remaining time in minutes

int8 state                       # e.g. STATE_NO_CHARING

uint32 voltage                   # mV 
int32 current                    # mA
int32 temperature                # °C

int32 nominal_capacity           # nominal capacity in mAh
int32 use_cycles                 # number of charge/discharge cycles completed

string[] sn                      # serial numbers of battery packs
