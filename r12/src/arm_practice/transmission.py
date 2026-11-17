"""XM430 output shaft <-> R12 wheel conversion; external 4x speed increase.

Internal DYNAMIXEL reduction is already included in published output-shaft RPM.
Efficiency must be supplied for torque conversion; no efficiency is certified.
"""
import math
import numpy as np

DRIVE_SPEED_RATIO = 4.0
STEER_SPEED_RATIO = 1.0
WHEEL_RADIUS_M = .03068719889482
NOMINAL_OUTPUT_RPM = 70.0  # XM430-W210 at 11.1V, no load
VELOCITY_UNIT_RPM = .229

def motor_to_wheel(value):
    return np.asarray(value)*DRIVE_SPEED_RATIO

def wheel_to_motor(value):
    return np.asarray(value)/DRIVE_SPEED_RATIO

def motor_torque_to_wheel(torque, efficiency):
    if not math.isfinite(efficiency) or not 0<efficiency<=1:
        raise ValueError('Efficiency must be in (0,1]')
    return np.asarray(torque)*efficiency/DRIVE_SPEED_RATIO

def wheel_velocity_to_register(wheel_rad_s):
    value=np.asarray(wheel_rad_s,dtype=float)
    if not np.isfinite(value).all():raise ValueError('Non-finite velocity')
    motor_rpm=wheel_to_motor(value)*60/(2*math.pi)
    # Truncate toward zero so quantization never raises requested shaft speed.
    limit=math.floor(NOMINAL_OUTPUT_RPM/VELOCITY_UNIT_RPM)
    return np.trunc(np.clip(motor_rpm/VELOCITY_UNIT_RPM,-limit,limit)).astype(np.int32)

def register_to_wheel_velocity(raw):
    return motor_to_wheel(np.asarray(raw)*VELOCITY_UNIT_RPM*2*math.pi/60)
