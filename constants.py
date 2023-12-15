"""
This file defines constants related to our robot.  These constants include:

 * Physical constants (exterior dimensions, wheel base)

 * Mechanical constants (gear reduction ratios)

 * Electrical constants (current limits, CAN bus IDs, roboRIO slot numbers)

 * Operation constants (desired max velocity, max turning speed)

 * Software constants (USB ID for driver joystick)
"""

import math
import ctre

# Physical constants
PHYS = {
    track_width: (21.73 * u.inch).m_as(u.m),
    wheel_base: (21.73 * u.inch).m_as(u.m),
    wheel_circumference: 4 * math.pi * u.inch,
}

# Mechanical constants
MECH = {
    swerve_module_propulsion_gearing_ratio: 6.75,   # SDS Mk4i L2
    swerve_module_steering_gearing_ratio: 150 / 7,  # SDS Mk4i

    propulsion_motor_inverted: False,
    steering_motor_inverted: False,
}

# Electrical constants
ELEC = {
    # These current limit parameters are per-motor in the swerve modules
    continuous_current_limit: 40,
    peak_current_limit: 60,
    peak_current_duration: 0.01,

    # time in seconds for propulsion motors to ramp up to full speed
    open_loop_ramp_rate: 0,
    closed_loop_ramp_rate: 0,
  
}

# Operation constants
OP = {
    max_speed: 4.5 * (u.m / u.s),
    max_angular_velocity: 11.5 * (u.rad / u.s),

    propulsion_neutral: ctre.NeutralMode.Coast,
    steering_neutral: ctre.NeutralMode.Brake,
}

# Software constants
SW = {
    # field_relative: True if "forward" means "down the field"; False if
    # "forward" means "in the direction the robot is facing"
    field_relative: False,

    # open_loop: True if we're not using any PID control or other forms of
    # feedback
    open_loop: True,

    # Constants for PID control of the propulsion AND steering motors
    # (set all to 0 for open-loop control)
    kP: 0,
    kI: 0,
    kD: 0,

    # Constants for feed-forward of propulsion motors
    kS: 0,
    kV: 0,
    kA: 0,

}
