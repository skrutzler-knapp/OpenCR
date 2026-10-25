#ifndef TURTLEBOT3_COMMAND_FRAME_H_
#define TURTLEBOT3_COMMAND_FRAME_H_

#include <stdint.h>

namespace TurtleBot3Core
{
namespace CommandFrame
{
static const uint16_t ADDRESS = 182;
static const uint8_t LENGTH = 10;
static const uint8_t COMMAND = 1;
static const uint8_t STOP = 2;
static const uint32_t MOTOR_CONTROL_INTERVAL_MS = 20;
static const uint32_t COMMAND_TIMEOUT_MS = 500;
static const uint32_t HEARTBEAT_TIMEOUT_MS = 500;

enum Result
{
  ACCEPTED,
  INVALID
};

inline int32_t decodeSignedInt32LE(const uint8_t* bytes)
{
  const uint32_t value = (uint32_t)bytes[0]
                       | ((uint32_t)bytes[1] << 8)
                       | ((uint32_t)bytes[2] << 16)
                       | ((uint32_t)bytes[3] << 24);
  if (value <= 0x7fffffffUL)
    return (int32_t)value;
  return (int32_t)((int64_t)value - 0x100000000LL);
}

inline Result apply(
  const uint8_t* frame,
  int32_t& linear_command,
  int32_t& angular_command,
  float& linear_goal,
  float& angular_goal,
  bool& torque_enabled,
  uint8_t& heartbeat,
  uint32_t& command_time_ms,
  uint8_t& error,
  float min_linear,
  float max_linear,
  float min_angular,
  float max_angular,
  uint32_t now_ms)
{
  // Command frames never own torque state; torque remains an explicit item.
  (void)torque_enabled;
  const int32_t linear = decodeSignedInt32LE(frame);
  const int32_t angular = decodeSignedInt32LE(frame + 4);
  const uint8_t kind = frame[9];

  if (kind == COMMAND)
  {
    linear_command = linear;
    angular_command = angular;
    const float requested_linear = static_cast<float>(linear * 0.01f);
    const float requested_angular = static_cast<float>(angular * 0.01f);
    linear_goal = requested_linear < min_linear ? min_linear :
      (requested_linear > max_linear ? max_linear : requested_linear);
    angular_goal = requested_angular < min_angular ? min_angular :
      (requested_angular > max_angular ? max_angular : requested_angular);
    command_time_ms = now_ms;
    heartbeat = frame[8];
    return ACCEPTED;
  }
  if (kind == STOP)
  {
    if (linear != 0 || angular != 0)
    {
      error = 0x04;  // DXL_ERR_DATA_RANGE
      return INVALID;
    }
    linear_command = 0;
    angular_command = 0;
    linear_goal = 0.0f;
    angular_goal = 0.0f;
    return ACCEPTED;
  }
  error = 0x04;  // DXL_ERR_DATA_RANGE
  return INVALID;
}

inline bool commandTimedOut(uint32_t now_ms, uint32_t command_time_ms)
{
  return now_ms - command_time_ms > COMMAND_TIMEOUT_MS;
}

inline bool heartbeatTimedOut(uint32_t now_ms, uint32_t heartbeat_time_ms)
{
  return now_ms - heartbeat_time_ms >= HEARTBEAT_TIMEOUT_MS;
}
}  // namespace CommandFrame
}  // namespace TurtleBot3Core

#endif  // TURTLEBOT3_COMMAND_FRAME_H_
