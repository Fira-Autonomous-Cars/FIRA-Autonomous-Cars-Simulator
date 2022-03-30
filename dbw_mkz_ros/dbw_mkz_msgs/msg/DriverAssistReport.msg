Header header

# Vehicle Decceleration (m/s^2)
float32 decel

# Deceleration source
uint8 decel_src
uint8 DECEL_NONE=0
uint8 DECEL_AEB=1
uint8 DECEL_ACC=2

# FCW Status
bool fcw_enabled
bool fcw_active

# AEB Status
bool aeb_enabled
bool aeb_precharge
bool aeb_braking

# ACC Status
bool acc_enabled
bool acc_braking
