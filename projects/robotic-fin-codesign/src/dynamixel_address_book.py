# Control table address
# config EEPROM
ADDR_OPERATING_MODE         = 11
ADDR_CURRENT_LIMIT          = 38
ADDR_MAX_POSITION_LIMIT     = 48
ADDR_MIN_POSITION_LIMIT     = 52

# Config RAM
ADDR_TORQUE_ENABLE          = 64         
ADDR_LED                    = 65

# Reading
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_VELOCITY       = 128
# NOTE: on XM-series motors this address is genuinely "Present Current" (mA).
# On XL-series motors (including the XL430-W250 used in this project), the
# X-series control table instead calls address 126 "Present Load" -- a
# signed percentage (-100.0% ~ 100.0%, unit 0.1%/LSB) of the motor's rated
# torque/current, not raw milliamps. read_current() still works (same
# register, same read mechanism), just interpret its return value as
# raw_value * 0.1 = load percent for this motor, not mA.
ADDR_PRESENT_CURRENT        = 126
ADDR_PRESENT_PWM            = 124
ADDR_PRESENT_INPUT_VOLTAGE  = 144  # RAM, read-only, unit 0.1 [V] (all X-series motors)

# Writing
ADDR_GOAL_POSITION          = 116
ADDR_GOAL_VELOCITY          = 104
ADDR_GOAL_CURRENT           = 102
ADDR_GOAL_PWM               = 100
ADDR_PROFILE_VELOCITY       = 112
ADDR_PROFILE_ACCELERATION   = 108

# Status
ADDR_MOVING                 = 122
ADDR_MOVING_STATUS          = 123


operating_modes_xm = {
                    "current": 0,
                    "velocity": 1,
                    "position": 3,
                    "extended position": 4,
                    "current-based position": 5,
                    "pwm": 16
                }

operating_modes_xl = {
                    "velocity": 1,
                    "position": 3,
                    "extended position": 4,
                    "pwm": 16
                }


# 2s complement conversion
max_register_value = {
                        "current": 65536,
                        "pwm": 65536,
                        "position": 4294967296,
                        "velocity": 4294967296,
                        "1 byte": 256,
                        "2 bytes": 65536,
                        "4 bytes": 4294967296
                    }