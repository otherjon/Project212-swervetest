"""
This file defines constants related to our robot.  These constants include:

 * Physical constants (exterior dimensions, wheel base)

 * Mechanical constants (gear reduction ratios)

 * Electrical constants (current limits, CAN bus IDs, roboRIO slot numbers)

 * Operation constants (desired max velocity, max turning speed)

 * Software constants (USB ID for driver joystick)
"""

import math
from collections import namedtuple
import rev
from swervepy import u

# Physical constants
phys_data = {
    "track_width": (14.75 * u.inch).m_as(u.m),
    "wheel_base": (14.75 * u.inch).m_as(u.m),
    "wheel_circumference": 4 * math.pi * u.inch,
}
PHYS = namedtuple("Data", phys_data.keys())(**phys_data)

# Mechanical constants
mech_data = {
    "swerve_module_propulsion_gearing_ratio": 6.75,   # SDS Mk4i L2
    "swerve_module_steering_gearing_ratio": 150 / 7,  # SDS Mk4i

    "propulsion_motor_inverted": False,
    "steering_motor_inverted": False,
}
MECH = namedtuple("Data", mech_data.keys())(**mech_data)

# Electrical constants
elec_data = {
    # These current limit parameters are per-motor in the swerve modules
    "continuous_current_limit": 40,
    "peak_current_limit": 60,

    # Talon FX motor controllers can set peak_current_duration.
    # SparkMAX motor controllers can't.
    #"peak_current_duration": 0.01,

    # time in seconds for propulsion motors to ramp up to full speed
    # reference: https://codedocs.revrobotics.com/java/com/revrobotics/cansparkmax
    "open_loop_ramp_rate": 0.5,
    "closed_loop_ramp_rate": 0.5,

    "LF_drive_CAN_ID": 11,
    "LF_steer_CAN_ID": 12,
    "LF_encoder_DIO": 0,
    "RF_drive_CAN_ID": 13,
    "RF_steer_CAN_ID": 14,
    "RF_encoder_DIO": 1,
    "RB_drive_CAN_ID": 15,
    "RB_steer_CAN_ID": 16,
    "RB_encoder_DIO": 2,
    "LB_drive_CAN_ID": 17,
    "LB_steer_CAN_ID": 18,
    "LB_encoder_DIO": 3,
}
ELEC = namedtuple("Data", elec_data.keys())(**elec_data)

# Operation constants
op_data = {
    # These maximum parameters reflect the maximum physically possible, not the
    # desired maximum limit.
    "max_speed": 4.5 * (u.m / u.s),
    "max_angular_velocity": 11.5 * (u.rad / u.s),

    "propulsion_neutral": rev.CANSparkMax.IdleMode.kCoast,
    "steering_neutral": rev.CANSparkMax.IdleMode.kBrake,

    # Values to pass to stick.getRawAxis()
    "translation_joystick_axis": 1,
    "strafe_joystick_axis": 0,
    "rotation_joystick_axis": 4,
}
OP = namedtuple("Data", op_data.keys())(**op_data)

# Software constants
sw_data = {
    # field_relative: True if "forward" means "down the field"; False if
    # "forward" means "in the direction the robot is facing"
    "field_relative": False,

    # open_loop: True if we're not using any PID control or other forms of
    # feedback
    "open_loop": True,

    # Constants for PID control of the propulsion AND steering motors
    # (set all to 0 for open-loop control)
    "kP": 0,
    "kI": 0,
    "kD": 0,

    # Constants for feed-forward of propulsion motors
    "kS": 0,
    "kV": 0,
    "kA": 0,
}
SW = namedtuple("Data", sw_data.keys())(**sw_data)
