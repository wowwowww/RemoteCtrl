#include "jaka_driver/pgi140_protocol.hpp"

namespace pgi140
{

uint16_t crc16_modbus(const uint8_t *data, std::size_t size)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = static_cast<uint16_t>((crc >> 1U) ^ 0xA001U);
      } else {
        crc = static_cast<uint16_t>(crc >> 1U);
      }
    }
  }
  return crc;
}

std::vector<uint8_t> append_crc(std::vector<uint8_t> frame)
{
  const uint16_t crc = crc16_modbus(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
  frame.push_back(static_cast<uint8_t>((crc >> 8U) & 0xFFU));
  return frame;
}

std::vector<uint8_t> make_write_single_register(
  uint8_t slave_id, uint16_t register_address, uint16_t value)
{
  return append_crc({
    slave_id,
    0x06U,
    static_cast<uint8_t>((register_address >> 8U) & 0xFFU),
    static_cast<uint8_t>(register_address & 0xFFU),
    static_cast<uint8_t>((value >> 8U) & 0xFFU),
    static_cast<uint8_t>(value & 0xFFU),
  });
}

std::vector<uint8_t> make_read_holding_registers(
  uint8_t slave_id, uint16_t register_address, uint16_t count)
{
  return append_crc({
    slave_id,
    0x03U,
    static_cast<uint8_t>((register_address >> 8U) & 0xFFU),
    static_cast<uint8_t>(register_address & 0xFFU),
    static_cast<uint8_t>((count >> 8U) & 0xFFU),
    static_cast<uint8_t>(count & 0xFFU),
  });
}

bool is_valid_slave_id(int value)
{
  return value >= 1 && value <= 247;
}

bool is_valid_force(int value)
{
  return value >= 20 && value <= 100;
}

bool is_valid_position(int value)
{
  return value >= 0 && value <= 1000;
}

bool is_valid_speed(int value)
{
  return value >= 1 && value <= 100;
}

}  // namespace pgi140
