#ifndef SHARED_MEMORY_BRIDGE_HPP
#define SHARED_MEMORY_BRIDGE_HPP

/**
 * @file shared_memory_bridge.hpp
 * @brief Header file for the SharedMemoryBridge class, which manages shared memory communication.
 *
 * This class provides functionality to create, access, and manage shared memory segments
 * for inter-process communication between the high-speed ROS2 control loop and the asynchronous
 * TCP/IP communication with the MG400 robot.
 *
 * @version 1.0
 * @date 2025-06-25
 * @author LT
 *
 */

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <iostream>

struct RealTimeState
{
    // State from robot -> ros2_control
    double q_actual[6];  // Actual joint positions
    double qd_actual[6]; // Actual joint velocities

    // Commands from ros2_control -> driver
    double q_command[6];  // Target joint positions
    double qd_command[6]; // Target joint velocities
    double qdd_command[6]; // Target joint accelerations
    bool new_command_flag;
};

/**
 * Provides an interface for accessing and manipulating the shared memory segment.
 */ 
class SharedMemoryBridge
{
public:
    RealTimeState *shared_memory_ptr = nullptr; // Pointer to the shared memory segment
    int shm_fd = -1;
    std::string shm_name = "/mg400_rt_state"; // Name of the shared memory segment

    /**
     * Constructor for the SharedMemoryBridge construct
     * Ensures support for multiple shared memory spaces for multiple robots
     * @param name Robot name
     */
    SharedMemoryBridge(const std::string &name)
    {
        shm_name = "/mg400_rt_state";
        std::cout << "Shared memory initalising with name: " << shm_name << std::endl;
    }

    bool create()
    {
        shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666); // 0666 -> read/write permissions for all
        if (shm_fd < 0)
        {
            return false;
        }
        if (ftruncate(shm_fd, sizeof(RealTimeState)) < 0)
        {
            return false;
        } // Set size of shared memory segment

        shared_memory_ptr = static_cast<RealTimeState *>(mmap(nullptr, sizeof(RealTimeState), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
        if (shared_memory_ptr == MAP_FAILED)
        {
            shared_memory_ptr = nullptr;
            return false;
        }
        is_creator = true;
        return true;
    }

    /**
     * @brief Attach to an existing shared memory segment.
     * @return true if successful, false otherwise.
     */
    bool attach()
    {
        shm_fd = shm_open(shm_name.c_str(), O_RDWR, 0666); // Open existing shared memory segment
        if (shm_fd < 0)
        {
            return false;
        }

        shared_memory_ptr = static_cast<RealTimeState *>(mmap(nullptr, sizeof(RealTimeState), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
        if (shared_memory_ptr == MAP_FAILED)
        {
            shared_memory_ptr = nullptr;
            return false;
        }
        return true;
    }

    /**
     * @brief Destructor for SharedMemoryBridge.
     * Cleans up the shared memory segment.
     * @note If the destructor is called by the creator, it will also unlink the shared memory segment.
     */
    ~SharedMemoryBridge()
    {
        if (shared_memory_ptr != MAP_FAILED && shared_memory_ptr != nullptr)
        {
            munmap(shared_memory_ptr, sizeof(RealTimeState));
        }
        if (shm_fd >= 0)
        {
            close(shm_fd);
        }

        // Creator should also unlink the shared memory segment
        if (is_creator)
        {
            shm_unlink(shm_name.c_str());
        }
    }

private:
    bool is_creator = false;
};

#endif // SHARED_MEMORY_BRIDGE_HPP