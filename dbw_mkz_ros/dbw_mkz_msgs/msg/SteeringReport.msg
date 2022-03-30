Header header

# Steering Wheel
float32 steering_wheel_angle      # rad
float32 steering_wheel_cmd        # rad or Nm
float32 steering_wheel_torque     # Nm
uint8 steering_wheel_cmd_type

# Command types
uint8 CMD_ANGLE=0
uint8 CMD_TORQUE=1

# Vehicle Speed
float32 speed                     # m/s

# Status
bool enabled  # Enabled
bool override # Driver override
bool timeout  # Command timeout

# Watchdog Counter
bool fault_wdc

# Faults
bool fault_bus1
bool fault_bus2
bool fault_calibration
bool fault_power
