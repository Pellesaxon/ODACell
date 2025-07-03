#ifndef MG400_ALARM_MANAGER_HPP
#define MG400_ALARM_MANAGER_HPP

#include "mg400_ros2_bringup/alarms/mg400_error_types.hpp"
#include "mg400_ros2_bringup/alarms/mg400_alarm_controller.hpp"
#include "mg400_ros2_bringup/alarms/mg400_alarm_servo.hpp"
#include "mg400_ros2_bringup/alarms/mg400_alarm_protocol.hpp"

namespace alarms {

// Specify which source the error is coming from
enum class Source {
    Controller,
    Servo,
    Protocol
};

// Lookup function to find an error by ID in a given array of ErrorInfo
template<size_t N>
constexpr const ErrorInfo& findById(const std::array<ErrorInfo, N>& arr, int id) {
    // A simple loop is perfectly efficient in a constexpr context.
    for (const auto& error : arr) {
        if (error.id == id) {
            return error;
        }
    }
    return UNKNOWN_ERROR;
}

// Function to get error information based on the source and ID
inline const ErrorInfo& getErrorInfo(Source source, int id) {
    if (source == Source::Controller) {
        return findById(controller::data, id);
    }
    if (source == Source::Servo) {
        return findById(servo::data, id);
    }
    if (source == Source::Protocol) {
        return findById(protocol::data, id);
    }
    return UNKNOWN_ERROR;
}

} // namespace alarms

#endif // MG400_ALARM_MANAGER_HPP