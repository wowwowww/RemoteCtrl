#include <cassert>
#include <cstdint>
#include <vector>

#include "jaka_driver/pgi140_protocol.hpp"

using pgi140::make_read_holding_registers;
using pgi140::make_write_single_register;

void expect_frame(const std::vector<uint8_t> & actual, std::initializer_list<uint8_t> expected)
{
  assert(actual == std::vector<uint8_t>(expected));
}

int main()
{
  // Frames copied from the PGI-140-80 V3.1 manual.
  expect_frame(make_write_single_register(1, 0x0100, 1),
               {0x01, 0x06, 0x01, 0x00, 0x00, 0x01, 0x49, 0xF6});
  expect_frame(make_write_single_register(1, 0x0101, 30),
               {0x01, 0x06, 0x01, 0x01, 0x00, 0x1E, 0x59, 0xFE});
  expect_frame(make_write_single_register(1, 0x0103, 500),
               {0x01, 0x06, 0x01, 0x03, 0x01, 0xF4, 0x78, 0x21});
  expect_frame(make_write_single_register(1, 0x0104, 50),
               {0x01, 0x06, 0x01, 0x04, 0x00, 0x32, 0x48, 0x22});
  expect_frame(make_read_holding_registers(1, 0x0201),
               {0x01, 0x03, 0x02, 0x01, 0x00, 0x01, 0xD4, 0x72});

  assert(pgi140::is_valid_slave_id(1));
  assert(!pgi140::is_valid_slave_id(248));
  assert(pgi140::is_valid_force(20));
  assert(!pgi140::is_valid_force(19));
  assert(pgi140::is_valid_position(1000));
  assert(!pgi140::is_valid_position(1001));
  assert(pgi140::is_valid_speed(1));
  assert(!pgi140::is_valid_speed(0));
  return 0;
}
