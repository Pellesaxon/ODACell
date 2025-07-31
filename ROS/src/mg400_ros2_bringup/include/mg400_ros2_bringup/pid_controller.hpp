#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <algorithm> 

struct PIDController {

    // Gains
    double kp, ki, kd;
    
    // Limits
    double min_output, max_output;
    double min_integral, max_integral;

    // State
    double integral = 0.0;
    double prev_error = 0.0;
    
    rclcpp::Clock::SharedPtr clock;
    rclcpp::Time last_time;
    bool first_run = true;

    PIDController(double p, double i, double d, 
                  double min_out, double max_out, 
                  rclcpp::Clock::SharedPtr c) 
        : kp(p), ki(i), kd(d), 
          min_output(min_out), max_output(max_out),
          clock(c) 
    {
        min_integral = min_output;
        max_integral = max_output;
    }

    double compute(double error) {
        auto now = clock->now();
        if (first_run) {
            last_time = now;
            first_run = false;
            // On the first run, there's no dt, so just return P-term clamped
            return std::clamp(kp * error, min_output, max_output);
        }
        
        double dt = (now - last_time).seconds();
        if (dt <= 1e-6) { 
             return std::clamp(kp * error, min_output, max_output);
        }

        double p_term = kp * error;

        integral += error * dt;
        integral = std::clamp(integral, min_integral, max_integral);
        double i_term = ki * integral;

        double derivative = (error - prev_error) / dt;
        double d_term = kd * derivative;
        
        prev_error = error;
        last_time = now;

        double output = p_term + i_term + d_term;
        
        return std::clamp(output, min_output, max_output);
    }

    void reset() {
        integral = 0.0;
        prev_error = 0.0;
        first_run = true;
    }
};

#endif // PID_CONTROLLER_HPP