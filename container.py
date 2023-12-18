import math

import wpilib
import wpimath.trajectory
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from swervepy import u, SwerveDrive, TrajectoryFollowerParameters
from swervepy.impl import CoaxialSwerveModule

from constants import PHYS, MECH, ELEC, OP, SW
import components

class RobotContainer:
    def __init__(self):
        gyro = components.gyro_component_class(**components.gyro_param_values)

        # The Azimuth component included the absolute encoder because it needs
        # to be able to reset to absolute position.
        #
        abs_enc = components.absolute_encoder_class
        modules = (
            # Left Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LF_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=azimuth_params,
                    absolute_encoder=abs_enc(ELEC.LF_encoder_DIO)),
                placement=Translation2d(*components.module_locations['LF']),
            ),
            # Right Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RF_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=azimuth_params,
                    absolute_encoder=abs_enc(ELEC.RF_encoder_DIO)),
                placement=Translation2d(*components.module_locations['RF']),
            ),
            # Left Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LB_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=azimuth_params,
                    absolute_encoder=abs_enc(ELEC.LB_encoder_DIO)),
                placement=Translation2d(*components.module_locations['LB']),
            ),
            # Right Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RB_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(0),
                    parameters=azimuth_params,
                    absolute_encoder=abs_enc(ELEC.RB_encoder_DIO)),
                placement=Translation2d(*components.module_locations['RB']),
            ),
        )

        self.stick = wpilib.Joystick(0)

        # Define a swerve drive subsystem by passing in a list of SwerveModules
        # and some options
        #
        self.swerve = SwerveDrive(
            modules, gyro, OP.max_speed, OP.max_angular_velocity)

        # Set the swerve subsystem's default command to teleoperate using
        # the controller joysticks
        #
        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                translation=self.get_translation_input,
                strafe=self.get_strafe_input,
                rotation=self.get_rotation_input,
                field_relative=SW.field_relative,
                open_loop=SW.open_loop,
            )
        )

    @staticmethod
    def deadband(value, band):
        return value if abs(value) > band else 0

    def get_translation_input(self):
        raw_stick_val = self.stick.getRawAxis(OP.translation_joystick_axis)
        return deadband(-raw_stick_val, 0.05)

    def get_strafe_input(self):
        raw_stick_val = self.stick.getRawAxis(OP.strafe_joystick_axis)
        return deadband(-raw_stick_val, 0.05)

    def get_rotation_input(self):
        raw_stick_val = self.stick.getRawAxis(OP.rotation_joystick_axis)
        return deadband(-raw_stick_val, 0.1)

    def get_autonomous_command(self):
        follower_params = TrajectoryFollowerParameters(
            target_angular_velocity=math.pi * (u.rad / u.s),
            target_angular_acceleration=math.pi * (u.rad / (u.s * u.s)),
            theta_kP=1,
            x_kP=1,
            y_kP=1,
        )

        trajectory_config = wpimath.trajectory.TrajectoryConfig(maxVelocity=4.5, maxAcceleration=1)

        trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(
            [
                Pose2d(0, 0, 0),  # Start at (0, 0)
                Pose2d(1, 0, 0),  # Move 1m forward
            ],
            trajectory_config,
        )

        first_path = True  # reset robot pose to initial pose in trajectory
        return self.swerve.follow_trajectory_command(
            trajectory, follower_params, first_path)
