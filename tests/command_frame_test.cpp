#include "turtlebot3/command_frame.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace
{
void require(bool condition, const char* message)
{
  if (!condition)
  {
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void writeSigned32LE(std::array<uint8_t, 10>& frame, std::size_t offset, int32_t value)
{
  const uint32_t bits = static_cast<uint32_t>(value);
  frame[offset] = static_cast<uint8_t>(bits);
  frame[offset + 1] = static_cast<uint8_t>(bits >> 8);
  frame[offset + 2] = static_cast<uint8_t>(bits >> 16);
  frame[offset + 3] = static_cast<uint8_t>(bits >> 24);
}
}  // namespace

int main()
{
  using TurtleBot3Core::CommandFrame::ACCEPTED;
  using TurtleBot3Core::CommandFrame::INVALID;
  using TurtleBot3Core::CommandFrame::apply;

  require(TurtleBot3Core::CommandFrame::ADDRESS == 182, "registered address is 182");
  require(TurtleBot3Core::CommandFrame::LENGTH == 10, "registered item length is 10 bytes");
  require(TurtleBot3Core::CommandFrame::COMMAND == 1, "COMMAND kind is 1");
  require(TurtleBot3Core::CommandFrame::STOP == 2, "STOP kind is 2");
  require(TurtleBot3Core::CommandFrame::MOTOR_CONTROL_INTERVAL_MS == 20, "control loop remains 50 Hz");
  require(TurtleBot3Core::CommandFrame::COMMAND_TIMEOUT_MS == 500, "command watchdog remains 500 ms");
  require(TurtleBot3Core::CommandFrame::HEARTBEAT_TIMEOUT_MS == 500, "heartbeat watchdog remains 500 ms");

  int32_t linear_command = 71;
  int32_t angular_command = -82;
  float linear_goal = 0.71f;
  float angular_goal = -0.82f;
  bool torque_enabled = true;
  uint8_t heartbeat = 42;
  uint32_t command_time_ms = 1234;
  uint8_t error = 0;
  std::array<uint8_t, 10> frame{};

  writeSigned32LE(frame, 0, -125);
  writeSigned32LE(frame, 4, 230);
  frame[8] = 0;
  frame[9] = TurtleBot3Core::CommandFrame::COMMAND;
  require(
    apply(frame.data(), linear_command, angular_command, linear_goal, angular_goal, torque_enabled, heartbeat,
      command_time_ms, error, -1.0f, 1.0f, -2.0f, 2.0f, 9000) == ACCEPTED,
    "valid zero-heartbeat COMMAND accepted");
  require(
    linear_command == -125 && angular_command == 230 &&
      linear_goal == -1.0f && angular_goal == 2.0f,
    "COMMAND updates both signed velocities and applies configured bounds");
  require(heartbeat == 0, "COMMAND updates heartbeat including wrap to zero");
  require(command_time_ms == 9000, "COMMAND refreshes stale-command time");
  require(torque_enabled, "COMMAND does not change explicit torque state");
  require(error == 0, "valid COMMAND does not set device error");

  const int32_t command_linear_before_stop = linear_command;
  const int32_t command_angular_before_stop = angular_command;
  const uint8_t heartbeat_before_stop = heartbeat;
  const uint32_t time_before_stop = command_time_ms;
  frame.fill(0);
  frame[9] = TurtleBot3Core::CommandFrame::STOP;
  require(
    apply(frame.data(), linear_command, angular_command, linear_goal, angular_goal, torque_enabled, heartbeat,
      command_time_ms, error, -1.0f, 1.0f, -2.0f, 2.0f, 10000) == ACCEPTED,
    "all-zero STOP accepted");
  require(
    linear_command == 0 && angular_command == 0 && linear_goal == 0.0f &&
      angular_goal == 0.0f,
    "STOP clears both commands and goals");
  require(
    heartbeat == heartbeat_before_stop && command_time_ms == time_before_stop,
    "STOP changes neither heartbeat nor command freshness");
  require(torque_enabled, "STOP does not change explicit torque state");
  require(command_linear_before_stop != 0 && command_angular_before_stop != 0,
    "STOP test begins with nonzero command state");

  linear_command = 17;
  angular_command = -19;
  linear_goal = 0.17f;
  angular_goal = -0.19f;
  heartbeat = 88;
  command_time_ms = 777;
  error = 0;
  writeSigned32LE(frame, 0, 1);
  writeSigned32LE(frame, 4, 0);
  frame[8] = 3;
  frame[9] = TurtleBot3Core::CommandFrame::STOP;
  require(
    apply(frame.data(), linear_command, angular_command, linear_goal, angular_goal, torque_enabled, heartbeat,
      command_time_ms, error, -1.0f, 1.0f, -2.0f, 2.0f, 10000) == INVALID,
    "nonzero STOP rejected");
  require(
    error == 0x04 && linear_command == 17 && angular_command == -19 &&
      linear_goal == 0.17f && angular_goal == -0.19f && heartbeat == 88 &&
      command_time_ms == 777 && torque_enabled,
    "invalid STOP sets data-range error and has no command/watchdog side effects");

  error = 0;
  frame[9] = 99;
  require(
    apply(frame.data(), linear_command, angular_command, linear_goal, angular_goal, torque_enabled, heartbeat,
      command_time_ms, error, -1.0f, 1.0f, -2.0f, 2.0f, 10000) == INVALID,
    "unknown kind rejected");
  require(
    error == 0x04 && linear_command == 17 && angular_command == -19 &&
      linear_goal == 0.17f && angular_goal == -0.19f && heartbeat == 88 &&
      command_time_ms == 777 && torque_enabled,
    "unknown kind sets data-range error without side effects");

  require(
    !TurtleBot3Core::CommandFrame::commandTimedOut(1777, 1277),
    "command remains fresh at exactly 500 ms");
  require(
    TurtleBot3Core::CommandFrame::commandTimedOut(1778, 1277),
    "command expires after 500 ms");
  require(
    TurtleBot3Core::CommandFrame::commandTimedOut(0x200U, 0xfffffff0U),
    "command watchdog handles millisecond counter wrap");
  require(
    !TurtleBot3Core::CommandFrame::heartbeatTimedOut(1776, 1277),
    "heartbeat remains live before its 500 ms loss threshold");
  require(
    TurtleBot3Core::CommandFrame::heartbeatTimedOut(1777, 1277),
    "heartbeat loss is detected at 500 ms");
  require(
    TurtleBot3Core::CommandFrame::heartbeatTimedOut(0x1ecU, 0xfffffff0U),
    "heartbeat watchdog handles millisecond counter wrap");

  std::cout << "OpenCR command frame tests passed\n";
  return EXIT_SUCCESS;
}
