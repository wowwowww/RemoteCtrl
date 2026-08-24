#ifndef JAKA_DRIVER_PGI140_PROTOCOL_HPP_
#define JAKA_DRIVER_PGI140_PROTOCOL_HPP_

#include <cstdint>
#include <cstddef>
#include <vector>

namespace pgi140
{

// PGI-140 uses standard Modbus-RTU CRC16, transmitted low byte first.
uint16_t crc16_modbus(const uint8_t *data, std::size_t size);

std::vector<uint8_t> make_write_single_register(
  uint8_t slave_id, uint16_t register_address, uint16_t value);

std::vector<uint8_t> make_read_holding_registers(
  uint8_t slave_id, uint16_t register_address, uint16_t count = 1);

bool is_valid_slave_id(int value);
bool is_valid_force(int value);
bool is_valid_position(int value);
bool is_valid_speed(int value);

}  // namespace pgi140

#endif  // JAKA_DRIVER_PGI140_PROTOCOL_HPP_
