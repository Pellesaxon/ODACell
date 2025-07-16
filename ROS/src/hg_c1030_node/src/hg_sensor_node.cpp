// hg_sensor_node.cpp

#include "hg_sensor_node.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>
#include <future>

// For serial port communication (Linux/macOS)
#include <errno.h>
#include <fcntl.h>
#include <limits>
#include <termios.h>
#include <unistd.h>

#define LASER_OFF_ON_INIT true

#define SENSOR_MAX_RANGE 0.035f
#define SENSOR_MIN_RANGE 0.025f

using namespace std::chrono_literals;

speed_t get_baud_rate_flag(int baud)
{
  switch (baud)
  {
  case 9600:
    return B9600;
  case 19200:
    return B19200;
  case 38400:
    return B38400;
  case 57600:
    return B57600;
  case 115200:
    return B115200;
  default:
    return B115200;
  }
}

HGSensorNode::HGSensorNode(const rclcpp::NodeOptions &options)
    : Node("hg_sensor_node", options), serial_fd_(-1), is_streaming_(false)
{

  this->declare_parameter<std::string>("port", "/dev/ttyACM0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<int>("num_sensors", 2);
  this->declare_parameter<std::string>("topic_prefix", "distance");
  this->declare_parameter<std::vector<std::string>>("frame_ids", std::vector<std::string>({"world", "world"}));

  port_ = this->get_parameter("port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  int num_sensors = this->get_parameter("num_sensors").as_int();
  std::string topic_prefix = this->get_parameter("topic_prefix").as_string();
  frame_ids_ = this->get_parameter("frame_ids").as_string_array();

  RCLCPP_INFO(this->get_logger(), "Starting HG Multi-Sensor Node...");
  RCLCPP_INFO(this->get_logger(), "Port: %s, Baud Rate: %d", port_.c_str(), baud_rate_);

  if (frame_ids_.size() != static_cast<size_t>(num_sensors))
  {
    RCLCPP_FATAL(this->get_logger(), "The number of 'frame_ids' (%zu) does not match 'num_sensors' (%d). Shutting down.", frame_ids_.size(), num_sensors);
    rclcpp::shutdown();
    return;
  }

  for (int i = 0; i < num_sensors; ++i)
  {
    std::string topic_name = topic_prefix + "/sensor_" + std::to_string(i);
    publishers_[i] = this->create_publisher<sensor_msgs::msg::Range>(topic_name, 10);
    RCLCPP_INFO(this->get_logger(), "Creating publisher for Sensor ID %d on topic '%s' with frame_id '%s'", i, topic_name.c_str(), frame_ids_[i].c_str());
  }

  if (!connect_to_device())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to device. Shutting down.");
    this->create_timer(1ms, []()
                       { rclcpp::shutdown(); });
    return;
  }

  action_server_ = rclcpp_action::create_server<ControlStreaming>(
      this,
      "control_streaming",
      std::bind(&HGSensorNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&HGSensorNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&HGSensorNode::handle_accepted, this, std::placeholders::_1));

  aux_power_client_ = this->create_client<std_srvs::srv::SetBool>("/mg400/auxiliary_power");

#if LASER_OFF_ON_INIT
  RCLCPP_INFO(this->get_logger(), "Requesting to turn off laser power on startup...");
  if (!aux_power_client_->wait_for_service(3s))
  {
    RCLCPP_WARN(this->get_logger(),
                "Auxiliary power service not available on startup. Laser may remain on.");
  }
  else
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false; // Set to FALSE to turn power OFF.

    using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
    {
      try
      {
        auto result = future.get();
        if (result->success)
        {
          RCLCPP_INFO(this->get_logger(), "Aux power successfully turned off on init: %s", result->message.c_str());
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "Aux power init call failed: %s", result->message.c_str());
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception while handling aux power init response: %s", e.what());
      }
    };

    aux_power_client_->async_send_request(request, response_received_callback);
  }
#endif

  RCLCPP_INFO(this->get_logger(), "Node initialized successfully. Ready to accept commands.");
}

HGSensorNode::~HGSensorNode()
{
  if (serial_fd_ != -1)
  {
    try
    {
      if (is_streaming_)
      {
        RCLCPP_INFO(this->get_logger(), "Sending STOP command before closing.");
        send_command("STOP\n");
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(this->get_logger(), "Could not send final STOP command: %s", e.what());
    }
    close(serial_fd_);
  }
  if (read_thread_.joinable())
  {
    read_thread_.join();
  }
}

bool HGSensorNode::connect_to_device()
{

  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ == -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Error opening serial port %s: %s", port_.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
    close(serial_fd_);
    return false;
  }

  speed_t baud_flag = get_baud_rate_flag(baud_rate_);
  if (baud_flag != static_cast<speed_t>(baud_rate_))
  {
    RCLCPP_WARN(this->get_logger(), "Unsupported baud rate %d. Defaulting to 115200.", baud_rate_);
  }
  cfsetispeed(&tty, baud_flag);
  cfsetospeed(&tty, baud_flag);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10; // 1-second timeout for each read() call
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
    close(serial_fd_);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Waiting for Arduino to reset...");
  std::this_thread::sleep_for(2s);

  RCLCPP_INFO(this->get_logger(), "Flushing serial buffer...");
  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(this->get_logger(), "Sending 'ID' command for handshake...");
  try
  {
    send_command("ID\n");
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send ID command: %s", e.what());
    close(serial_fd_);
    return false;
  }

  bool handshake_success = false;
  RCLCPP_INFO(this->get_logger(), "Waiting for handshake response...");
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 3)
  {
    std::string line = read_line(1000);
    if (!line.empty())
    {
      RCLCPP_INFO(this->get_logger(), "Handshake response received: \"%s\"", line.c_str());
      if (line.find("VERSION") != std::string::npos)
      {
        handshake_success = true;
        break;
      }
    }
  }

  if (handshake_success)
  {
    RCLCPP_INFO(this->get_logger(), "Handshake successful. Connection established.");
    read_thread_ = std::thread(&HGSensorNode::read_loop, this);
    return true;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Device on %s failed to respond correctly to 'ID' command.", port_.c_str());
    close(serial_fd_);
    return false;
  }
}

void HGSensorNode::send_command(const std::string &cmd)
{
  if (serial_fd_ == -1)
    throw std::runtime_error("Serial port not open.");

  ssize_t bytes_written = write(serial_fd_, cmd.c_str(), cmd.length());
  if (bytes_written < 0)
  {
    throw std::runtime_error("Failed to write to serial port.");
  }
  if (static_cast<size_t>(bytes_written) != cmd.length())
  {
    RCLCPP_WARN(this->get_logger(), "Could not write full command to serial port.");
  }
  RCLCPP_DEBUG(this->get_logger(), "Sent command: %s", cmd.c_str());
}

std::string HGSensorNode::read_line(int timeout_ms)
{
  size_t newline_pos = serial_buffer_.find('\n');
  if (newline_pos != std::string::npos)
  {
    std::string line = serial_buffer_.substr(0, newline_pos);
    serial_buffer_.erase(0, newline_pos + 1);
    // Trim trailing whitespace and carriage returns
    line.erase(line.find_last_not_of("\r\n \t") + 1);
    return line;
  }

  auto start_time = std::chrono::steady_clock::now();
  char read_buf[256];

  while (rclcpp::ok())
  {
    ssize_t bytes_read = read(serial_fd_, &read_buf, sizeof(read_buf));

    if (bytes_read > 0)
    {
      serial_buffer_.append(read_buf, bytes_read);
      newline_pos = serial_buffer_.find('\n');
      if (newline_pos != std::string::npos)
      {
        std::string line = serial_buffer_.substr(0, newline_pos);
        serial_buffer_.erase(0, newline_pos + 1);
        line.erase(line.find_last_not_of("\r\n \t") + 1);
        return line;
      }
    }
    else if (bytes_read < 0 && errno != EAGAIN)
    {
      RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
      return ""; // Return empty on error
    }

    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
    if (timeout_ms > 0 && elapsed_time.count() > timeout_ms)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Read timeout after %d ms", timeout_ms);
      return "";
    }
    std::this_thread::sleep_for(1ms);
  }
  return "";
}

bool HGSensorNode::set_aux_power(bool on)
{
  if (!aux_power_client_->wait_for_service(2s))
  {
    RCLCPP_ERROR(this->get_logger(), "auxiliary_power service not available.");
    return false;
  }

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = on;

  auto future = aux_power_client_->async_send_request(req);

  if (future.wait_for(2s) == std::future_status::ready)
  {
    auto res = future.get();
    if (!res->success)
    {
      RCLCPP_WARN(this->get_logger(),
                  "auxiliary_power service responded 'false': %s",
                  res->message.c_str());
    }
    return res->success;
  }
  else
  {
    // The wait timed out.
    RCLCPP_ERROR(this->get_logger(),
                 "auxiliary_power service call timed out");
    return false;
  }
}

void HGSensorNode::read_loop()
{
  RCLCPP_INFO(this->get_logger(), "Read thread started.");

  while (rclcpp::ok())
  {
    std::string line = read_line(0); // indefinite wait, relying on rclcpp::ok()

    if (!rclcpp::ok())
      break; // Exit immediately if shutdown is requested

    if (line.empty())
    {
      continue;
    }

    size_t comma_pos = line.find(',');
    if (comma_pos == std::string::npos)
    {
      RCLCPP_INFO(this->get_logger(), "Arduino Status: %s", line.c_str());
      continue;
    }

    try
    {
      std::string id_str = line.substr(0, comma_pos);
      std::string val_str = line.substr(comma_pos + 1);
      int sensor_id = std::stoi(id_str);
      float distance = std::stof(val_str); // Distance in meters

      if (publishers_.count(sensor_id))
      {
        auto msg = std::make_unique<sensor_msgs::msg::Range>();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = frame_ids_[sensor_id];
        msg->radiation_type = sensor_msgs::msg::Range::INFRARED;
        msg->min_range = SENSOR_MIN_RANGE;
        msg->max_range = SENSOR_MAX_RANGE;

        if (distance < SENSOR_MIN_RANGE || distance > SENSOR_MAX_RANGE)
        {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 5000,
                               "Sensor ID %d reported out-of-range value: %.3f m. Publishing infinity.", sensor_id, distance);
          msg->range = std::numeric_limits<float>::infinity();
        }
        else
        {
          msg->range = distance;
        }

        publishers_[sensor_id]->publish(std::move(msg));
      }
      else
      {
        RCLCPP_WARN_ONCE(this->get_logger(), "Received data for unconfigured sensor ID: %d. Ignoring.", sensor_id);
      }
    }
    catch (const std::invalid_argument &e)
    {
      RCLCPP_WARN(this->get_logger(), "Could not parse line from Arduino: '%s'. Invalid argument: %s", line.c_str(), e.what());
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_WARN(this->get_logger(), "Could not parse line from Arduino: '%s'. Out of range: %s", line.c_str(), e.what());
    }
  }
  RCLCPP_INFO(this->get_logger(), "Read thread finished.");
}

rclcpp_action::GoalResponse HGSensorNode::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const ControlStreaming::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request to %s streaming", goal->start_streaming ? "START" : "STOP");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HGSensorNode::handle_cancel(const std::shared_ptr<GoalHandleControlStreaming>)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void HGSensorNode::handle_accepted(const std::shared_ptr<GoalHandleControlStreaming> goal_handle)
{
  std::thread{std::bind(&HGSensorNode::execute_goal, this, std::placeholders::_1), goal_handle}.detach();
}

void HGSensorNode::execute_goal(const std::shared_ptr<GoalHandleControlStreaming> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<ControlStreaming::Result>();

  if (goal_handle->is_canceling())
  {
    result->success = false;
    result->message = "Goal canceled.";
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  try
  {
    if (goal->start_streaming)
    {
      if (is_streaming_)
      {
        result->message = "INFO: Streaming was already active.";
      }
      else
      {
        if (set_aux_power(true))
        {
          send_command("START\n");
          is_streaming_ = true;
          result->message = "OK: Streaming started.";
        }
        else
        {
          result->message = "ERROR: Failed to turn on auxiliary power (laser).";
          RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
          goal_handle->abort(result);
          return;
        }
      }
    }
    else
    { // stop streaming
      if (!is_streaming_)
      {
        result->message = "INFO: Streaming was already stopped.";
      }
      else
      {
        if (!set_aux_power(false))
        {
          RCLCPP_WARN(this->get_logger(), "Failed to turn off auxiliary power (laser). Continuing to stop streaming data.");
        }
        send_command("STOP\n");
        is_streaming_ = false;
        result->message = "OK: Streaming stopped.";
      }
    }
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Action Succeeded: %s", result->message.c_str());
  }
  catch (const std::exception &e)
  {
    result->success = false;
    result->message = "Failed to send command to device: " + std::string(e.what());
    goal_handle->abort(result);
    RCLCPP_ERROR(this->get_logger(), "Action Aborted: %s", result->message.c_str());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HGSensorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}