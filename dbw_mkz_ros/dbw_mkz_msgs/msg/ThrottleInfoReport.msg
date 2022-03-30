Header header

# Throttle Pedal
float32 throttle_pc   # Throttle pedal percent, range 0 to 1
float32 throttle_rate # Throttle pedal change per second (1/s)
QualityFactor throttle_pedal_qf # Non-zero is limp-home

# Engine
float32 engine_rpm    # Engine Speed (rpm)

# Gear Num
GearNum gear_num      # Gear Number

# Ignition status
Ignition ignition     # Ignition status (enum)

# Battery Current
float32 batt_curr     # Battery Current (Amps)
