# Synthetic sampled safety inputs. software_estop_active is not a physical
# emergency-stop function and must never be represented as one.
builtin_interfaces/Time stamp
uint64 sequence
float64 supply_voltage_v
bool software_estop_active
bool watchdog_heartbeat
bool relay_feedback_enabled
bool relay_feedback_healthy
