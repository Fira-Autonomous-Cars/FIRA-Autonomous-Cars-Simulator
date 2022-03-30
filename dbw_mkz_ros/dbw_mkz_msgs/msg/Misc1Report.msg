Header header

# Turn Signal enumeration
TurnSignal turn_signal

# High beams
bool high_beam_headlights

# Front Windshield Wipers enumeration
Wiper wiper

# Ambient Light Sensor enumeration
AmbientLight ambient_light

# Outside air temperature
float32 outside_temperature # Degrees Celsius

# Buttons
bool btn_cc_on        # Cruise Control On
bool btn_cc_off       # Cruise Control Off
bool btn_cc_on_off    # Cruise Control On/Off Toggle
bool btn_cc_res       # Cruise Control Resume
bool btn_cc_cncl      # Cruise Control Cancel
bool btn_cc_res_cncl  # Cruise Control Resume/Cancel
bool btn_cc_res_inc   # Cruise Control Resume+
bool btn_cc_res_dec   # Cruise Control Resume-
bool btn_cc_set_inc   # Cruise Control Set+
bool btn_cc_set_dec   # Cruise Control Set-
bool btn_cc_gap_inc   # Cruise Control Gap+
bool btn_cc_gap_dec   # Cruise Control Gap-
bool btn_la_on_off    # Lane Assist On/Off Toggle
bool btn_ld_ok        # Left D-Pad OK
bool btn_ld_up        # Left D-Pad Up
bool btn_ld_down      # Left D-Pad Down
bool btn_ld_left      # Left D-Pad Left
bool btn_ld_right     # Left D-Pad Right
bool btn_rd_ok        # Right D-Pad OK
bool btn_rd_up        # Right D-Pad Up
bool btn_rd_down      # Right D-Pad Down
bool btn_rd_left      # Right D-Pad Left
bool btn_rd_right     # Right D-Pad Right
bool btn_vol_inc      # Volume increment
bool btn_vol_dec      # Volume decrement
bool btn_mute         # Volume mute
bool btn_media        # Media (source)
bool btn_prev         # Media previous
bool btn_next         # Media next
bool btn_speak        # Speak
bool btn_call_start   # Call start/answer
bool btn_call_end     # Call end/disconnect

# Faults
bool fault_bus

# Doors
bool door_driver
bool door_passenger
bool door_rear_left
bool door_rear_right
bool door_hood
bool door_trunk

# Passenger seat
bool passenger_detect
bool passenger_airbag

# Seat Belts
bool buckle_driver
bool buckle_passenger
