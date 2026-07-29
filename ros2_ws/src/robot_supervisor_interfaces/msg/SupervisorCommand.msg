# High-level synthetic supervisor command. Sequence ownership belongs to the
# command publisher; reset is deliberately excluded and uses ResetFault.
builtin_interfaces/Time stamp
uint64 sequence
bool arm_request
bool run_request
bool shutdown_request
float64 requested_motor_voltage_v
