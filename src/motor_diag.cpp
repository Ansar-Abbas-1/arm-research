// READ-ONLY motor status report. Writes nothing to any motor: it pings each ID
// and reads status registers (Protocol 2.0, X-series addresses), so it cannot
// move a motor or change torque, limits or gains.
//
// It needs the serial port to itself: stop the robot_interface stack first
// (two processes sharing /dev/ttyUSB0 corrupt each other's packets).
//
// Usage (run by the user, stack stopped):
//   rosrun arm_interface motor_diag [--device /dev/ttyUSB0] [--baud 1000000] [--ids 1,2,3,4,5,6]
//
// What it answers: did a motor shut itself down (Torque Enable 0 with a
// Hardware Error Status bit set, e.g. overload), is it hot or under-voltage,
// how much load/current it carries now, and what the Shutdown register says it
// is configured to do on each error.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <dynamixel_sdk/dynamixel_sdk.h>

#include "arm_interface/motor_status.h"

namespace {

// X-series control table (Protocol 2.0).
constexpr uint16_t kAddrOperatingMode = 11;
constexpr uint16_t kAddrTemperatureLimit = 31;
constexpr uint16_t kAddrShutdown = 63;
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrHardwareError = 70;
constexpr uint16_t kAddrProfileAcceleration = 108;
constexpr uint16_t kAddrProfileVelocity = 112;
constexpr uint16_t kAddrMoving = 122;
constexpr uint16_t kAddrPresentLoadOrCurrent = 126;  // current (X430/X540) or load (MX-2.0), 2 bytes signed
constexpr uint16_t kAddrPresentPosition = 132;
constexpr uint16_t kAddrPresentVoltage = 144;
constexpr uint16_t kAddrPresentTemperature = 146;

struct Reader {
  dynamixel::PortHandler* port;
  dynamixel::PacketHandler* packet;
  uint8_t id;

  bool Read1(uint16_t addr, uint8_t* v) const {
    uint8_t err = 0;
    return packet->read1ByteTxRx(port, id, addr, v, &err) == COMM_SUCCESS;
  }
  bool Read2(uint16_t addr, uint16_t* v) const {
    uint8_t err = 0;
    return packet->read2ByteTxRx(port, id, addr, v, &err) == COMM_SUCCESS;
  }
  bool Read4(uint16_t addr, uint32_t* v) const {
    uint8_t err = 0;
    return packet->read4ByteTxRx(port, id, addr, v, &err) == COMM_SUCCESS;
  }
};

std::vector<int> ParseIds(const std::string& s) {
  std::vector<int> ids;
  size_t start = 0;
  while (start < s.size()) {
    size_t comma = s.find(',', start);
    ids.push_back(std::atoi(s.substr(start, comma == std::string::npos ? std::string::npos : comma - start).c_str()));
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return ids;
}

}  // namespace

int main(int argc, char** argv) {
  std::string device = "/dev/ttyUSB0";
  int baud = 1000000;
  std::vector<int> ids = {1, 2, 3, 4, 5, 6};
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--device")) device = argv[i + 1];
    else if (!std::strcmp(argv[i], "--baud")) baud = std::atoi(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--ids")) ids = ParseIds(argv[i + 1]);
  }

  dynamixel::PortHandler* port = dynamixel::PortHandler::getPortHandler(device.c_str());
  dynamixel::PacketHandler* packet = dynamixel::PacketHandler::getPacketHandler(2.0);
  if (!port->openPort() || !port->setBaudRate(baud)) {
    std::printf("Cannot open %s at %d baud (is the robot_interface stack still running?).\n", device.c_str(), baud);
    return 1;
  }
  std::printf("READ-ONLY report, %s @ %d baud. Nothing is written to any motor.\n\n", device.c_str(), baud);

  for (int id : ids) {
    Reader r{port, packet, static_cast<uint8_t>(id)};
    uint16_t model = 0;
    uint8_t ping_err = 0;
    if (packet->ping(port, r.id, &model, &ping_err) != COMM_SUCCESS) {
      std::printf("ID %d: NO REPLY to ping (unpowered, disconnected, or bus fault)\n\n", id);
      continue;
    }
    uint8_t mode = 0, temp_limit = 0, shutdown = 0, torque = 0, hw_err = 0, moving = 0, temp = 0;
    uint16_t load = 0, volt = 0;
    uint32_t prof_acc = 0, prof_vel = 0, pos = 0;
    bool ok = r.Read1(kAddrOperatingMode, &mode) & r.Read1(kAddrTemperatureLimit, &temp_limit) &
              r.Read1(kAddrShutdown, &shutdown) & r.Read1(kAddrTorqueEnable, &torque) &
              r.Read1(kAddrHardwareError, &hw_err) & r.Read4(kAddrProfileAcceleration, &prof_acc) &
              r.Read4(kAddrProfileVelocity, &prof_vel) & r.Read1(kAddrMoving, &moving) &
              r.Read2(kAddrPresentLoadOrCurrent, &load) & r.Read4(kAddrPresentPosition, &pos) &
              r.Read2(kAddrPresentVoltage, &volt) & r.Read1(kAddrPresentTemperature, &temp);
    std::printf("ID %d  model %u%s\n", id, model, ok ? "" : "  (some registers did not read)");
    std::printf("  operating mode        : %s\n", arm_interface::DecodeOperatingMode(mode).c_str());
    std::printf("  torque enable         : %u%s\n", torque, torque ? "" : "   <-- torque is OFF in the motor");
    std::printf("  hardware error status : 0x%02X  (%s)%s\n", hw_err,
                arm_interface::JoinNames(arm_interface::DecodeHardwareError(hw_err)).c_str(),
                hw_err ? "   <-- ERROR LATCHED" : "");
    std::printf("  shutdown on           : %s  (0x%02X)\n",
                arm_interface::JoinNames(arm_interface::DecodeShutdownMask(shutdown)).c_str(), shutdown);
    std::printf("  temperature           : %u C  (limit %u C)\n", temp, temp_limit);
    std::printf("  input voltage         : %.1f V\n", volt * 0.1);
    std::printf("  present load/current  : %d raw (signed, 2 bytes)\n", static_cast<int16_t>(load));
    std::printf("  present position      : %d ticks\n", static_cast<int32_t>(pos));
    std::printf("  moving                : %u\n", moving);
    std::printf("  profile accel / vel   : %u / %u  (0 = unlimited)\n\n", prof_acc, prof_vel);
  }
  port->closePort();
  return 0;
}
