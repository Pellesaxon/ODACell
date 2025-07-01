# Comprehensive status message for the MG400

# From RobotMode() command (page 29)
int32 robot_mode
string robot_mode_string

# From GetErrorID() command (page 31)
int32[] controller_error_codes
string[] controller_error_strings
int32[] servo_error_codes
string[] servo_error_strings

# From Real-time Feedback (page 66-67)
bool is_in_error
bool is_enabled
bool is_in_drag

# Brake status for each joint (J1-J4)
bool[] brake_status