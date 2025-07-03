#ifndef MG400_ERROR_TYPES_HPP
#define MG400_ERROR_TYPES_HPP

#pragma once

#include <string_view>

namespace alarms {

// Holds the details for an error
struct ErrorDetails {
    std::string_view description;
    std::string_view cause;
    std::string_view solution;
};

// Holds all information for a single alarm code
struct ErrorInfo {
    int id;
    int level;
    ErrorDetails en;

    // A helper to check if the error is valid (not the 'unknown' placeholder)
    constexpr bool isValid() const { return id != -1; }
};

// A constant placeholder for when a lookup fails.
constexpr ErrorInfo UNKNOWN_ERROR = {
    -1, 99,
    {"Unknown Error Code", "The provided ID does not exist in the database.", "Check the error code and the alarm source."}
};

} // namespace alarms


#endif // MG400_ERROR_TYPES_HPP