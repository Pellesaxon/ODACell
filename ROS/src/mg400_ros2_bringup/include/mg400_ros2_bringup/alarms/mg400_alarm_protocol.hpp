#ifndef MG400_ALARM_PROTOCOL_HPP
#define MG400_ALARM_PROTOCOL_HPP

#include "mg400_ros2_bringup/alarms/mg400_error_types.hpp"
#include <array>

namespace alarms::protocol
{

    constexpr std::array<ErrorInfo, 9> data = {{
        {0, 0, {"No error", "Command was delivered successfully.", ""}},
        {-1, 5, {"Execution Failed", "The robot received the command but failed to execute it.", "Check robot state. The robot may be busy, disabled, or in a state that prevents this command."}},
        {-10000, 5, {"Command Does Not Exist", "The sent command name is not recognized by the robot controller.", "Check for typos in the command name. Ensure you are using a compatible controller version."}},
        {-20000, 5, {"Incorrect Parameter Count", "The command was sent with the wrong number of parameters.", "Check the command's documentation for the correct number of arguments."}},
        {-30001, 5, {"Incorrect Parameter Type (1st)", "The first parameter is of the wrong data type (e.g., string instead of int).", "Check the command's documentation for the correct parameter types."}},
        {-30002, 5, {"Incorrect Parameter Type (2nd)", "The second parameter is of the wrong data type.", "Check the command's documentation for the correct parameter types."}},
        {-40001, 5, {"Incorrect Parameter Range (1st)", "The first parameter is out of the acceptable range.", "Check the command's documentation for valid ranges."}},
        {-40002, 5, {"Incorrect Parameter Range (2nd)", "The second parameter is out of the acceptable range.", "Check the command's documentation for valid ranges."}},
        {-999, 10, {"Connection Failure", "Could not connect to the robot or no response was received.", "Check network connection, IP address, and ensure the robot controller is powered on."}} // Custom own error
    }};
} // namespace alarms::protocol

#endif // MG400_ALARM_PROTOCOL_HPP