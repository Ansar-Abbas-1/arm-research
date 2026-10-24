#pragma once

namespace arm_interface {

// True once `elapsed_sec` since the last accepted command exceeds
// `timeout_sec` — trigger for disabling torque on the real-hardware node.
inline bool IsCommandStale(double elapsed_sec, double timeout_sec) {
  return elapsed_sec > timeout_sec;
}

// Rejects non-positive timeouts, which would either never trigger the
// watchdog (<=0 read as "no timeout") or trigger it immediately.
inline bool IsValidCommandTimeout(double timeout_sec) {
  return timeout_sec > 0.0;
}

}  // namespace arm_interface
