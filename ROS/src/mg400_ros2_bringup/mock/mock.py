"""
Mock implementation of the mg400 robot TCP interface for testing purposes.

This script simulates the three primary TCP ports of the Dobot MG400 robot:
- Dashboard (29999): For control and status commands.
- Motion (30003): For receiving and queuing motion commands.
- Feedback (30004): For streaming real-time robot state.

A background simulation task handles motion execution, allowing the mock robot
to respond to trajectory commands from the C++ driver.
"""
import asyncio
import struct
import re
import time
import signal

# Constants from C++ and manual
HOST_IP = "localhost"
MOTION_COMMAND_PORT = 30003
REALTIME_FEEDBACK_PORT = 30004
DASHBOARD_COMMAND_PORT = 29999

FEEDBACK_PACKET_SIZE = 1440
NUM_JOINTS = 4

# RobotMode enum values from manual page 29
ROBOT_MODE_INIT = 1
ROBOT_MODE_BRAKE_OPEN = 2
ROBOT_MODE_DISABLED = 4
ROBOT_MODE_ENABLE = 5
ROBOT_MODE_RUNNING = 7
ROBOT_MODE_ERROR = 9
ROBOT_MODE_PAUSE = 10


class RobotState:
    """Encapsulates the entire state of the simulated robot."""

    def __init__(self):
        self.lock = asyncio.Lock()

        # Positional state (in degrees, as the robot protocol uses degrees)
        self.q_actual_deg = [0.1] * NUM_JOINTS
        self.qd_actual_deg = [0.0] * NUM_JOINTS

        # Control state
        self.robot_mode = ROBOT_MODE_DISABLED
        self.is_enabled = False
        self.is_in_error = False

        # Trajectory/Motion state
        self.run_queued_cmd_flag = False
        self.motion_queue = asyncio.Queue()
        self.is_moving = False

        # Other feedback flags from the manual
        self.queue_paused_flag = False
        self.is_in_drag = False
        self.brake_status_byte = 0b00111100  # Brakes on for J1-J4 when disabled

    def pack_feedback_data(self) -> bytes:
        """
        Constructs the 1440-byte feedback packet using the current robot state.
        Offsets and data types are based on the Dobot TCP/IP Interface Guide.
        """
        buffer = bytearray(FEEDBACK_PACKET_SIZE)

        def pack_at(offset, fmt, *values):
            struct.pack_into(f"<{fmt}", buffer, offset, *values)

        current_timestamp_ms = time.time_ns() // 1_000_000

        pack_at(0, "H", FEEDBACK_PACKET_SIZE)
        pack_at(24, "Q", self.robot_mode)
        
        pack_at(32, "Q", current_timestamp_ms) # TimeStamp (uint64, in ms)        
        pack_at(432, "6d", *self.q_actual_deg, 0.0, 0.0)
        pack_at(480, "6d", *self.qd_actual_deg, 0.0, 0.0)
        pack_at(1014, "b", 1 if self.run_queued_cmd_flag else 0)
        pack_at(1015, "b", 1 if self.queue_paused_flag else 0)
        pack_at(1025, "B", self.brake_status_byte)
        pack_at(1026, "b", 1 if self.is_enabled else 0)
        pack_at(1027, "b", 1 if self.is_in_drag else 0)
        pack_at(1029, "b", 1 if self.is_in_error else 0)

        return bytes(buffer)


async def robot_simulation_loop(state: RobotState):
    """The core loop that simulates the robot's physical movement."""
    start_pos = [0.0] * NUM_JOINTS
    target_pos = [0.0] * NUM_JOINTS
    move_duration = 1.0  # seconds
    move_start_time = 0

    simulation_interval = 0.008  # Corresponds to a 125Hz simulation rate, matching feedback

    while True:
        async with state.lock:
            # Check for a new movement command if we are not currently moving
            if not state.is_moving and not state.motion_queue.empty():
                current_target = await state.motion_queue.get()
                start_pos = list(state.q_actual_deg)
                target_pos = list(current_target["positions"])

                # Simple duration calculation based on max joint travel and speed
                max_travel = max(abs(t - s) for s, t in zip(start_pos, target_pos))
                speed_factor = current_target.get("speed", 100) / 100.0

                if max_travel > 0.1:  # Avoid division by zero for tiny moves
                    # Assume a 90-degree move at 100% speed takes 1.0 second
                    base_time = (max_travel / 90.0) * (1.0 / max(speed_factor, 0.01))
                    move_duration = max(base_time, 0.1)  # Minimum move time
                else:
                    move_duration = 0.1

                move_start_time = time.monotonic()
                state.is_moving = True
                state.run_queued_cmd_flag = True
                state.robot_mode = ROBOT_MODE_RUNNING
                print(f"SIM: Starting move to {target_pos} over {move_duration:.2f}s")

            # If we are moving, update the position via linear interpolation
            if state.is_moving:
                elapsed = time.monotonic() - move_start_time
                progress = min(elapsed / move_duration, 1.0)
                prev_pos = list(state.q_actual_deg)

                for i in range(NUM_JOINTS):
                    state.q_actual_deg[i] = start_pos[i] + (target_pos[i] - start_pos[i]) * progress
                    state.qd_actual_deg[i] = (state.q_actual_deg[i] - prev_pos[i]) / simulation_interval

                # When the move is complete, update state
                if progress >= 1.0:
                    print(f"SIM: Move finished at {state.q_actual_deg}")
                    state.is_moving = False
                    state.q_actual_deg = list(target_pos)  # Snap to final position
                    state.qd_actual_deg = [0.0] * NUM_JOINTS

                    if state.motion_queue.empty():
                        state.run_queued_cmd_flag = False
                        if state.is_enabled:
                            state.robot_mode = ROBOT_MODE_ENABLE

        await asyncio.sleep(simulation_interval)


async def handle_dashboard_client(reader, writer, state: RobotState):
    """Handles a single, short-lived dashboard command connection."""
    addr = writer.get_extra_info("peername")
    print(f"DASHBOARD: Connection from {addr}")

    try:
        data = await reader.read(1024)
        command_str = data.decode().strip()
        if not command_str:
            return

        print(f"DASHBOARD: Received command: {command_str}")
        match = re.match(r"(\w+)\((.*)\)", command_str)
        if not match:
            response = f"-10000,{{}},{command_str};"  # Command error
            writer.write(response.encode())
            await writer.drain()
            return

        command_name = match.group(1)
        response_str = f"0,{{}},{command_str};"  # Default success response

        async with state.lock:
            if command_name == "EnableRobot":
                state.is_enabled = True
                state.is_in_error = False
                state.robot_mode = ROBOT_MODE_ENABLE
                state.brake_status_byte = 0  # Brakes off
                print("STATE: Robot Enabled")

            elif command_name == "DisableRobot":
                state.is_enabled = False
                state.robot_mode = ROBOT_MODE_DISABLED
                state.brake_status_byte = 0b00111100
                print("STATE: Robot Disabled")

            elif command_name == "ClearError":
                state.is_in_error = False
                state.robot_mode = ROBOT_MODE_ENABLE if state.is_enabled else ROBOT_MODE_DISABLED
                print("STATE: Errors Cleared")

            elif command_name == "ResetRobot":
                while not state.motion_queue.empty():
                    await state.motion_queue.get()
                state.is_moving = False
                state.run_queued_cmd_flag = False
                state.robot_mode = ROBOT_MODE_ENABLE if state.is_enabled else ROBOT_MODE_DISABLED
                print("STATE: Robot Reset (motion queue cleared)")

            elif command_name == "GetErrorID":
                # Respond with the detailed error format expected by the C++ driver's regex
                response_str = "0,{[[],[],[],[],[],[]]},GetErrorID();"

            elif command_name == "DoExecute":
                # Acknowledge toggling digital outputs
                response_str = "0,{},DoExecute();"

            elif command_name in ["SetCollisionLevel", "SpeedFactor", "SpeedJ", "AccJ"]:
                # Acknowledge startup commands with success
                pass
            
            else:
                print(f"DASHBOARD: Unhandled command '{command_name}'")
                response_str = f"-10000,{{}},{command_str};"

        writer.write(response_str.encode())
        await writer.drain()

    except Exception as e:
        print(f"DASHBOARD: Error handling client: {e}")
    finally:
        print(f"DASHBOARD: Closing connection with {addr}")
        writer.close()
        await writer.wait_closed()


async def handle_motion_client(reader, writer, state: RobotState):
    """Handles the persistent motion command connection."""
    addr = writer.get_extra_info("peername")
    print(f"MOTION: Connection from {addr}")

    try:
        while True:
            data = await reader.read(2048) # Increased buffer size just in case
            if not data:
                print("MOTION: Client disconnected.")
                break

            command_str = data.decode().strip()
            print(f"MOTION: Received buffer with {len(command_str.split(')'))-1} potential commands.")

            # This regex is non-greedy `(.*?)` to ensure it stops at the first closing parenthesis
            pattern = re.compile(r"JointMovJ\((.*?)\)")
            matches_found = 0
            for match in pattern.finditer(command_str):
                params_str = match.group(1)
                params = params_str.split(',')
                
                try:
                    # Extract positions, speed, and acceleration
                    positions = [float(p) for p in params[:4]]
                    speed_search = re.search(r"SpeedJ=(\d+)", params_str)
                    accel_search = re.search(r"AccJ=(\d+)", params_str)

                    speed = int(speed_search.group(1)) if speed_search else 100
                    accel = int(accel_search.group(1)) if accel_search else 100

                    target = {
                        "positions": positions,
                        "speed": speed,
                        "accel": accel
                    }
                    await state.motion_queue.put(target)
                    matches_found += 1
                except (ValueError, IndexError, AttributeError) as e:
                    full_cmd = f"JointMovJ({params_str})"
                    print(f"MOTION: Error parsing JointMovJ params from '{full_cmd}': {e}")
            
            if matches_found > 0:
                print(f"MOTION: Successfully parsed and queued {matches_found} commands.")
            else:
                 print(f"MOTION: No valid JointMovJ commands found in buffer: {command_str}")


    except asyncio.CancelledError:
        print("MOTION: Connection handler cancelled.")
    except Exception as e:
        print(f"MOTION: Error: {e}")
    finally:
        print(f"MOTION: Closing connection with {addr}")
        writer.close()
        await writer.wait_closed()


async def handle_feedback_client(reader, writer, state: RobotState):
    """Handles the persistent feedback data stream connection."""
    addr = writer.get_extra_info("peername")
    print(f"FEEDBACK: Connection from {addr}")
    
    feedback_interval = 0.008

    try:
        while True:
            async with state.lock:
                feedback_packet = state.pack_feedback_data()

            writer.write(feedback_packet)
            await writer.drain()

            await asyncio.sleep(feedback_interval)

    except (ConnectionResetError, BrokenPipeError):
        print("FEEDBACK: Client disconnected.")
    except asyncio.CancelledError:
        print("FEEDBACK: Connection handler cancelled.")
    except Exception as e:
        print(f"FEEDBACK: Error: {e}")
    finally:
        print(f"FEEDBACK: Closing connection with {addr}")
        writer.close()
        await writer.wait_closed()

def signal_handler(signal, frame):
    """Handles graceful shutdown on SIGINT/SIGTERM (Ctrl+C)."""
    print("\nReceived shutdown signal, cancelling tasks...")
    for task in asyncio.all_tasks():
        task.cancel()


async def main():
    """Starts all servers and the simulation loop."""

    # Register signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    robot_state = RobotState()
    simulation_task = asyncio.create_task(robot_simulation_loop(robot_state))

    # Create a server for each port, passing the shared state object
    dashboard_server = await asyncio.start_server(
        lambda r, w: handle_dashboard_client(r, w, robot_state),
        HOST_IP, DASHBOARD_COMMAND_PORT
    )
    motion_server = await asyncio.start_server(
        lambda r, w: handle_motion_client(r, w, robot_state),
        HOST_IP, MOTION_COMMAND_PORT
    )
    feedback_server = await asyncio.start_server(
        lambda r, w: handle_feedback_client(r, w, robot_state),
        HOST_IP, REALTIME_FEEDBACK_PORT
    )

    print(f'Dashboard server listening on {HOST_IP}:{DASHBOARD_COMMAND_PORT}')
    print(f'Motion server listening on {HOST_IP}:{MOTION_COMMAND_PORT}')
    print(f'Feedback server listening on {HOST_IP}:{REALTIME_FEEDBACK_PORT}')
    print("Mock MG400 server is running. Press Ctrl+C to stop.")

    try:
        # Keep servers running indefinitely
        await asyncio.gather(
            dashboard_server.serve_forever(),
            motion_server.serve_forever(),
            feedback_server.serve_forever()
        )
    except asyncio.CancelledError:
        print("Main task cancelled, shutting down.")
    finally:
        simulation_task.cancel()
        for srv in [dashboard_server, motion_server, feedback_server]:
            srv.close()
            await srv.wait_closed()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown requested by user.")