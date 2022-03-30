# Brake pedal
# Options defined below
float32 pedal_cmd
uint8 pedal_cmd_type

# Enable
bool enable

# Clear driver overrides
bool clear

# Ignore driver overrides
bool ignore

# Watchdog counter (optional)
uint8 count

uint8 CMD_NONE=0
uint8 CMD_PEDAL=1     # Unitless, range 0.15 to 0.50
uint8 CMD_PERCENT=2   # Percent of maximum torque, range 0 to 1
uint8 CMD_TORQUE=3    # Nm, range 0 to 3412, open-loop
uint8 CMD_TORQUE_RQ=4 # Nm, range 0 to 3412, closed-loop
uint8 CMD_DECEL=6     # m/s^2, range 0 to 10

float32 TORQUE_BOO=520  # Nm, brake lights threshold
float32 TORQUE_MAX=3412 # Nm, maximum torque
