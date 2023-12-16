import math

import wpilib
import wpimath.trajectory
from wpimath.geometry import Translation2d, Rotation2d, Transform3d, Pose2d

from swervepy import u, vision, SwerveDrive, TrajectoryFollowerParameters
from swervepy.impl import (
    PigeonGyro,
    CoaxialSwerveModule,
    Falcon500CoaxialDriveComponent,
    Falcon500CoaxialAzimuthComponent,
    AbsoluteCANCoder,
)

from constants import PHYS, MECH, ELEC, OP, SW

class RobotContainer:
    def __init__(self):

        # Each component defines a Parameters dataclass with any options
        # applicable to all instances of that component.  For example, wheel
        # circumference is the same for all four modules on a SDS Mk4 drive
        # base, so that is included in the Parameters class.  Motor IDs are
        # different for each instance of a module, so those are not included.
        #
        drive_params = NEOCoaxialDriveComponent.Parameters(
            wheel_circumference=PHYS.wheel_circumference,
            gear_ratio=MECH.swerve_module_propulsion_gearing_ratio,
            max_speed=OP.max_speed,
            open_loop_ramp_rate=ELEC.open_loop_ramp_rate,
            closed_loop_ramp_rate=ELEC.closed_loop_ramp_rate,
            continuous_current_limit=ELEC.continuous_current_limit,
            peak_current_limit=ELEC.peak_current_limit,
            peak_current_duration=ELEC.peak_current_duration,
            neutral_mode=OP.propulsion_neutral,
            kP=SW.kP,
            kI=SW.kI,
            kD=SW.kD,
            kS=SW.kS,
            kV=SW.kV,
            kA=SW.kA,
            invert_motor=MECH.propulsion_motor_inverted,
        )
        azimuth_params = NEOCoaxialAzimuthComponent.Parameters(
            gear_ratio=MECH.swerve_module_propulsion_gearing_ratio,
            max_angular_velocity=OP.max_angular_velocity,
            ramp_rate=0,
            continuous_current_limit=ELEC.continuous_current_limit,
            peak_current_limit=ELEC.peak_current_limit,
            peak_current_duration=ELEC.peak_current_duration,
            neutral_mode=OP.steering_neutral,
            kP=SW.kP,
            kI=SW.kI,
            kD=SW.kD,
            invert_motor=MECH.steering_motor_inverted,
        )

        # TODO: Create an actual gyro object, once one is connected
        #gyro = PigeonGyro(0, True)
        gyro = None

        # When defining module positions for kinematics, +x values represent
        # moving toward the front of the robot, and +y values represent
        # moving toward the left of the robot.
        #
        modules = (
            # Swerve module implementations are as general as possible (coaxial, differential) but take specific components
            # like Falcon or NEO drive motors as arguments.
            CoaxialSwerveModule(
                # Pass in general Parameters and module-specific options
                NEOCoaxialDriveComponent(ELEC.LF_drive_CAN_ID, drive_params),
                # The Azimuth component included the CANCOder (absolute encoder) because it needs to be able to
                # reset to absolute position
                NEOCoaxialAzimuthComponent(ELEC.LF_steer_CAN_ID, Rotation2d.fromDegrees(0), azimuth_params, DutyCycleEncoder(ELEC.LF_encoder_DIO),
                Translation2d(PHYS.wheel_base / 2, PHYS.track_width / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(ELEC.RF_drive_CAN_ID, drive_params),
                NEOCoaxialAzimuthComponent(ELEC.RF_steer_CAN_ID, Rotation2d.fromDegrees(0), azimuth_params, DutyCycleEncoder(ELEC.RF_encoder_DIO),
                Translation2d(PHYS.wheel_base / 2, -PHYS.track_width / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(ELEC.RB_drive_CAN_ID, drive_params),
                NEOCoaxialAzimuthComponent(ELEC.RB_steer_CAN_ID, Rotation2d.fromDegrees(0), azimuth_params, DutyCycleEncoder(ELEC.RB_encoder_DIO),
                Translation2d(-PHYS.wheel_base / 2, PHYS.track_width / 2),
            ),
            CoaxialSwerveModule(
                NEOCoaxialDriveComponent(ELEC.LB_drive_CAN_ID, drive_params),
                NEOCoaxialAzimuthComponent(ELEC.LB_steer_CAN_ID, Rotation2d.fromDegrees(0), azimuth_params, DutyCycleEncoder(ELEC.LB_encoder_DIO),
                Translation2d(-PHYS.wheel_base / 2, -PHYS.track_width / 2),
            ),
        )

        self.stick = wpilib.Joystick(0)

        # Define a swerve drive subsystem by passing in a list of SwerveModules and some options
        self.swerve = SwerveDrive(modules, gyro, OP.max_speed, OP.max_angular_velocity)

        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                lambda: deadband(-self.stick.getRawAxis(1), 0.05),
                lambda: deadband(-self.stick.getRawAxis(0), 0.05),
                lambda: deadband(-self.stick.getRawAxis(4), 0.1),  # Invert for CCW+
                SW.field_relative,
                SW.open_loop,
            )
        )

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

        return self.swerve.follow_trajectory_command(trajectory, follower_params, True)


def deadband(value, band):
    return value if abs(value) > band else 0
