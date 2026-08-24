#ifndef JAKA_DRIVER_PGI140_GRIPPER_HPP_
#define JAKA_DRIVER_PGI140_GRIPPER_HPP_

#include <cstdint>
#include <string>

#include "jaka_driver/JAKAZuRobot.h"

namespace pgi140
{

struct GripperConfig
{
  int rs485_channel = 1;       // 0: TIO RS485H, 1: TIO RS485L (TIO channel 2)
  int slave_id = 1;
  int baudrate = 115200;
  int databit = 8;
  int stopbit = 1;
  int parity = 78;             // JAKA SDK: 78 means no parity
  int force_percent = 50;      // PGI range: 20..100
  int speed_percent = 50;      // PGI range: 1..100
  int open_position = 0;       // PGI range: 0..1000 per mille
  int closed_position = 1000;  // PGI range: 0..1000 per mille
  bool initialize = true;
  int initialize_command = 0x01;
  int initialize_delay_ms = 2000;
  bool configure_tio = true;   // false: use the TIO configuration saved in JAKA App
  bool enable_tio_power = true;
  int tio_voltage = 0;         // JAKA TIO_VOUT_24V = 0, TIO_VOUT_12V = 1
};

class Pgi140Gripper
{
public:
  Pgi140Gripper(JAKAZuRobot & robot, GripperConfig config);

  // Configure the selected TIO RS485 channel and optionally initialize the gripper.
  bool initialize(std::string * error = nullptr);

  // Send a target position. The value is validated before any frame is sent.
  bool set_position(int position, std::string * error = nullptr);
  bool set_force(int force_percent, std::string * error = nullptr);
  bool set_speed(int speed_percent, std::string * error = nullptr);
  bool set_closed(bool closed, std::string * error = nullptr);

  bool closed() const { return closed_; }
  const GripperConfig & config() const { return config_; }

private:
  bool write_register(uint16_t address, uint16_t value, std::string * error);
  bool fail(const std::string & message, std::string * error) const;

  JAKAZuRobot & robot_;
  GripperConfig config_;
  bool configured_ = false;
  bool closed_ = false;
};

}  // namespace pgi140

#endif  // JAKA_DRIVER_PGI140_GRIPPER_HPP_
