"""
This file defines the components of our swerve drive.  This is mostly just
defining which components (and thus which classes) we're using, calling the
appropriate constructors, and passing in variables.  Since this many lines of
code takes up space, it can obscure what's going on around it, so we'll do
all that here instead of in the container.py module.
"""

from wpilib import DutyCycleEncoder
from constants import PHYS, MECH, ELEC, OP, SW
from swervepy.impl import (
    DummyGyro,
    CoaxialSwerveModule,
    NEOCoaxialDriveComponent,
    NEOCoaxialAzimuthComponent,
)

# Define which components we're using (which motors we're using)
#
drive_component_class = NEOCoaxialDriveComponent
azimuth_component_class = NEOCoaxialAzimuthComponent
gyro_component_class = DummyGyro
absolute_encoder_class = DutyCycleEncoder

# For the drive/azimuth classes selected, set the parameters which DO NOT vary
# between the modules.  (Look at the Parameters data class for the
# DriveComponent and AzimuthComponent classes you've selected.  Each Parameters
# data class has different options, based on the motors it uses etc.)
#
drive_param_values = {
    # For NEOCoaxialDriveComponent:
    wheel_circumference: PHYS.wheel_circumference,
    gear_ratio: MECH.swerve_module_propulsion_gearing_ratio,
    max_speed: OP.max_speed,
    open_loop_ramp_rate: ELEC.open_loop_ramp_rate,
    closed_loop_ramp_rate: ELEC.closed_loop_ramp_rate,
    continuous_current_limit: ELEC.continuous_current_limit,
    peak_current_limit: ELEC.peak_current_limit,
    peak_current_duration: ELEC.peak_current_duration,
    neutral_mode: OP.propulsion_neutral,
    kP: SW.kP,
    kI: SW.kI,
    kD: SW.kD,
    kS: SW.kS,
    kV: SW.kV,
    kA: SW.kA,
    invert_motor: MECH.propulsion_motor_inverted,
}
azimuth_param_values = {
    # For NEOCoaxialAzimuthComponent:
    gear_ratio: MECH.swerve_module_propulsion_gearing_ratio,
    max_angular_velocity: OP.max_angular_velocity,
    ramp_rate: 0,
    continuous_current_limit: ELEC.continuous_current_limit,
    peak_current_limit: ELEC.peak_current_limit,
    peak_current_duration: ELEC.peak_current_duration,
    neutral_mode: OP.steering_neutral,
    kP: SW.kP,
    kI: SW.kI,
    kD: SW.kD,
    invert_motor: MECH.steering_motor_inverted,
}

gyro_param_values = {
    # For DummyGyro:
}

drive_component_params_class = drive_component_class.Parameters
azimuth_component_params_class = azimuth_component_class.Parameters

drive_params = drive_component_params_class(**drive_param_values)
azimuth_params = azimuth_component_params_class(**azimuth_param_values)

module_locations = {
    # When defining module positions for kinematics, +x values represent
    # moving toward the front of the robot, and +y values represent
    # moving toward the left of the robot.
    #
    'LF': (PHYS.wheel_base / 2, PHYS.track_width / 2),
    'RF': (PHYS.wheel_base / 2, -PHYS.track_width / 2),
    'LB': (-PHYS.wheel_base / 2, PHYS.track_width / 2),
    'RB': (-PHYS.wheel_base / 2, -PHYS.track_width / 2),
}
