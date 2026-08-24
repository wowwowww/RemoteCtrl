#include "jaka_driver/pgi140_gripper.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include "jaka_driver/jkerr.h"
#include "jaka_driver/pgi140_protocol.hpp"

namespace pgi140
{
namespace
{
constexpr uint16_t kInitializationRegister = 0x0100;
constexpr uint16_t kForceRegister = 0x0101;
constexpr uint16_t kPositionRegister = 0x0103;
constexpr uint16_t kSpeedRegister = 0x0104;

bool valid_channel(int channel)
{
  return channel == 0 || channel == 1;
}
}  // namespace

Pgi140Gripper::Pgi140Gripper(JAKAZuRobot & robot, GripperConfig config)
: robot_(robot), config_(config)
{
}

bool Pgi140Gripper::fail(const std::string & message, std::string * error) const
{
  if (error != nullptr) {
    *error = message;
  }
  return false;
}

bool Pgi140Gripper::write_register(uint16_t address, uint16_t value, std::string * error)
{
  auto frame = make_write_single_register(
    static_cast<uint8_t>(config_.slave_id), address, value);
  const int ret = robot_.send_tio_rs_command(
    config_.rs485_channel, frame.data(), static_cast<int>(frame.size()));
  if (ret != ERR_SUCC) {
    return fail(
      "send_tio_rs_command failed, return code " + std::to_string(ret), error);
  }
  return true;
}

bool Pgi140Gripper::initialize(std::string * error)
{
  if (!valid_channel(config_.rs485_channel)) {
    return fail("rs485_channel must be 0 (RS485H) or 1 (RS485L)", error);
  }
  if (!is_valid_slave_id(config_.slave_id)) {
    return fail("slave_id must be in the Modbus range 1..247", error);
  }
  if (config_.baudrate <= 0 || config_.databit < 7 || config_.databit > 8 ||
    (config_.stopbit != 1 && config_.stopbit != 2) ||
    (config_.parity != 78 && config_.parity != 79 && config_.parity != 69))
  {
    return fail("invalid RS485 serial parameters", error);
  }
  if (!is_valid_force(config_.force_percent) ||
    !is_valid_speed(config_.speed_percent) ||
    !is_valid_position(config_.open_position) ||
    !is_valid_position(config_.closed_position))
  {
    return fail("invalid PGI force, speed, or position parameter", error);
  }
  if (config_.initialize_command != 0x01 && config_.initialize_command != 0xA5) {
    return fail("initialize_command must be 0x01 or 0xA5", error);
  }
  if (config_.enable_tio_power && (config_.tio_voltage < 0 || config_.tio_voltage > 1)) {
    return fail("tio_voltage must be 0 (24 V) or 1 (12 V)", error);
  }

  if (config_.enable_tio_power &&
    robot_.set_tio_vout_param(1, config_.tio_voltage) != ERR_SUCC)
  {
    return fail("set_tio_vout_param failed", error);
  }

  if (config_.configure_tio) {
    // TIO RS485H reuses DO1/DO2; RS485L reuses AIN1/AIN2.
    const int pin_ret = config_.rs485_channel == 0 ?
      robot_.set_tio_pin_mode(1, 0xFF) : robot_.set_tio_pin_mode(2, 1);
    if (pin_ret != ERR_SUCC) {
      return fail(
        "set_tio_pin_mode failed (SDK code " + std::to_string(pin_ret) +
        "); SDK channel " + std::to_string(config_.rs485_channel) +
        " maps to TIO RS485 channel " + std::to_string(config_.rs485_channel + 1) +
        "; verify TIO channel selection and controller TIO hardware/configuration, "
        "or set gripper_configure_tio=false after configuring TIO in JAKA App", error);
    }
    if (robot_.set_rs485_chn_mode(config_.rs485_channel, 0) != ERR_SUCC) {
      return fail("set_rs485_chn_mode(Modbus RTU) failed", error);
    }

    ModRtuComm comm{};
    comm.chn_id = config_.rs485_channel;
    comm.slaveId = config_.slave_id;
    comm.baudrate = config_.baudrate;
    comm.databit = config_.databit;
    comm.stopbit = config_.stopbit;
    comm.parity = config_.parity;
    if (robot_.set_rs485_chn_comm(comm) != ERR_SUCC) {
      return fail("set_rs485_chn_comm failed", error);
    }
  }

  configured_ = true;
  if (config_.initialize) {
    if (!write_register(kInitializationRegister,
      static_cast<uint16_t>(config_.initialize_command), error))
    {
      configured_ = false;
      return false;
    }
    // PGI documentation specifies an initialization time of about 0.5..3 s.
    if (config_.initialize_delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(config_.initialize_delay_ms));
    }
  }

  if (!set_force(config_.force_percent, error) ||
    !set_speed(config_.speed_percent, error) ||
    !set_position(config_.open_position, error))
  {
    configured_ = false;
    return false;
  }
  closed_ = false;
  return true;
}

bool Pgi140Gripper::set_position(int position, std::string * error)
{
  if (!configured_) {
    return fail("gripper is not initialized", error);
  }
  if (!is_valid_position(position)) {
    return fail("position must be in the PGI range 0..1000", error);
  }
  return write_register(kPositionRegister, static_cast<uint16_t>(position), error);
}

bool Pgi140Gripper::set_force(int force_percent, std::string * error)
{
  if (!configured_) {
    return fail("gripper is not initialized", error);
  }
  if (!is_valid_force(force_percent)) {
    return fail("force must be in the PGI range 20..100", error);
  }
  return write_register(kForceRegister, static_cast<uint16_t>(force_percent), error);
}

bool Pgi140Gripper::set_speed(int speed_percent, std::string * error)
{
  if (!configured_) {
    return fail("gripper is not initialized", error);
  }
  if (!is_valid_speed(speed_percent)) {
    return fail("speed must be in the PGI range 1..100", error);
  }
  return write_register(kSpeedRegister, static_cast<uint16_t>(speed_percent), error);
}

bool Pgi140Gripper::set_closed(bool closed, std::string * error)
{
  if (!set_position(closed ? config_.closed_position : config_.open_position, error)) {
    return false;
  }
  closed_ = closed;
  return true;
}

}  // namespace pgi140
