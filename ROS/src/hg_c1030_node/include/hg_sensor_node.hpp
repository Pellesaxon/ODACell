#ifndef HG_SENSOR_NODE_HPP_
#define HG_SENSOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/range.hpp>
#include "hg_c1030_msgs/action/control_streaming.hpp"

#include <string>
#include <thread>
#include <vector>
#include <map>

class HGSensorNode : public rclcpp::Node
{
public:
  using ControlStreaming = hg_c1030_msgs::action::ControlStreaming;
  using GoalHandleControlStreaming = rclcpp_action::ServerGoalHandle<ControlStreaming>;
  /**
   * @brief Construct a new HGSensorNode object
   * @param options Node options for rclcpp::Node
   */
  explicit HGSensorNode(const rclcpp::NodeOptions &options);

  /**
   * @brief Destroy the HGSensorNode object
   */
  ~HGSensorNode();

private:
  /**
   * @brief Opens and configures the serial port to communicate with the Arduino.
   * @return true if connection is successful, false otherwise.
   */
  bool connect_to_device();

  /**
   * @brief Sends a string command to the Arduino, appending a newline.
   * @param cmd The command string to send.
   */
  void send_command(const std::string &cmd);

  /**
   * @brief Reads from the serial port until a newline character is found or a timeout occurs.
   * @param timeout_ms Timeout in milliseconds.
   * @return The line read from the serial port, or an empty string on timeout.
   */
  std::string read_line(int timeout_ms);

  /**
   * @brief The main loop for the reading thread. Continuously reads from the serial port.
   */
  void read_loop();

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const ControlStreaming::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleControlStreaming> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleControlStreaming> goal_handle);

  /**
   * @brief Executes the action goal logic in a separate thread.
   * @param goal_handle A shared pointer to the goal handle.
   */
  void execute_goal(const std::shared_ptr<GoalHandleControlStreaming> goal_handle);

  // Serial port
  int serial_fd_;
  std::string port_;
  int baud_rate_;
  std::string frame_id_;
  bool is_streaming_;
  std::thread read_thread_;
  std::string serial_buffer_;
  
  std::vector<std::string> frame_ids_;
  // Publisher and action server interfaces
  // Map of publishers by sensor ID, e.g. 0, 1, 2, etc.
  std::map<int, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> publishers_;
  rclcpp_action::Server<ControlStreaming>::SharedPtr action_server_;
};

#endif // HG_SENSOR_NODE_HPP_