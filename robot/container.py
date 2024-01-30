import math
import logging
logger = logging.getLogger("your.robot")

import wpilib
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
import commands2

from swervepy import u, SwerveDrive, TrajectoryFollowerParameters
from swervepy.impl import CoaxialSwerveModule

from constants import PHYS, MECH, ELEC, OP, SW
import components

from commands.wheels_straight import WheelsStraight

class RobotContainer:
    def __init__(self):
        swerve_gyro = components.gyro_component_class(
            **components.gyro_param_values)  # NavXGyro wrapper class
        self.gyro = swerve_gyro.navx  # underlying navx.AHRS object
        self.gyro.zeroYaw()

        # The Azimuth component included the absolute encoder because it needs
        # to be able to reset to absolute position.
        #
        self.lf_enc = components.absolute_encoder_class(
            ELEC.LF_encoder_DIO, SW.lf_enc_zeropos)
        self.lb_enc = components.absolute_encoder_class(
            ELEC.LB_encoder_DIO, SW.lb_enc_zeropos)
        self.rf_enc = components.absolute_encoder_class(
            ELEC.RF_encoder_DIO, SW.rf_enc_zeropos)
        self.rb_enc = components.absolute_encoder_class(
            ELEC.RB_encoder_DIO, SW.rb_enc_zeropos)

        # Determine what the current reading of the 4 encoders should be, given
        # that SW.XX_enc_zeropos says where the wheels face front
        #
        lf_enc_pos = self.lf_enc.absolute_position_degrees - SW.lf_enc_zeropos
        rf_enc_pos = self.rf_enc.absolute_position_degrees - SW.rf_enc_zeropos
        lb_enc_pos = self.lb_enc.absolute_position_degrees - SW.lb_enc_zeropos
        rb_enc_pos = self.rb_enc.absolute_position_degrees - SW.rb_enc_zeropos
        logger.info(f"Encoder startup positions: {lf_enc_pos}, {lb_enc_pos}, {rf_enc_pos}, {rb_enc_pos}")

        modules = (
            # Left Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LF_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(lf_enc_pos),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.lf_enc),
                placement=Translation2d(*components.module_locations['LF']),
            ),
            # Right Front module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RF_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RF_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(rf_enc_pos),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.rf_enc),
                placement=Translation2d(*components.module_locations['RF']),
            ),
            # Left Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.LB_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.LB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(lb_enc_pos),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.lb_enc),
                placement=Translation2d(*components.module_locations['LB']),
            ),
            # Right Back module
            CoaxialSwerveModule(
                drive=components.drive_component_class(
                    id_=ELEC.RB_drive_CAN_ID,
                    parameters=components.drive_params),
                azimuth=components.azimuth_component_class(
                    id_=ELEC.RB_steer_CAN_ID,
                    azimuth_offset=Rotation2d.fromDegrees(rb_enc_pos),
                    parameters=components.azimuth_params,
                    absolute_encoder=self.rb_enc),
                placement=Translation2d(*components.module_locations['RB']),
            ),
        )

        self.stick = commands2.button.CommandXboxController(0)
        def wheelsStraight():
            logger.info("Straightening wheels")
            for m in modules:
                m._azimuth.follow_angle(Rotation2d.fromDegrees(0))
        wheels_straight_cmd = commands2.cmd.run(wheelsStraight)
        self.stick.a().onTrue(wheels_straight_cmd)
        self.stick.a().onFalse(
            commands2.cmd.runOnce(wheels_straight_cmd.cancel))

        self.speed_limit_ratio = 1.0
        if OP.speed_limit:
            if OP.speed_limit > OP.max_speed:
                wpilib.reportWarning("Speed limit is greater than max_speed and won't be used")
            else:
                self.speed_limit_ratio = OP.speed_limit / OP.max_speed

        self.angular_velocity_limit_ratio = 1.0
        if OP.angular_velocity_limit:
            if OP.angular_velocity_limit > OP.max_angular_velocity:
                wpilib.reportWarning("Angular velocity limit is greater than max_angular_velocity and won't be used")
            else:
                self.angular_velocity_limit_ratio = (
                    OP.angular_velocity_limit / OP.max_angular_velocity)

        # Define a swerve drive subsystem by passing in a list of SwerveModules
        # and some options
        #
        self.swerve = SwerveDrive(
            modules, swerve_gyro, OP.max_speed, OP.max_angular_velocity)

        def set_speed_limits(transl, rot):
            logger.info(f"Setting limits to {transl}, {rot}")
            self.speed_limit_ratio = transl
            self.angular_velocity_limit_ratio = rot
        # X button: "low gear", set the speed limits as requested in constants.py
        self.stick.x().onTrue(commands2.InstantCommand(
            lambda: set_speed_limits(
                OP.speed_limit / OP.max_speed,
                OP.angular_velocity_limit / OP.max_angular_velocity)))
        # B button: "high gear", set the speed limits to "as fast as possible"
        self.stick.b().onTrue(commands2.InstantCommand(
            lambda: set_speed_limits(1.0, 1.0)))
        # Y button: reset the "down the field" orientation (joystick up)
        self.stick.y().onTrue(commands2.InstantCommand(
            lambda: logger.info("Zeroing yaw") and self.gyro.zeroYaw()))

        # START button: tell the encoders that the wheels are straight
        # (Probably not needed anymore)
        # TODO: investigate whether we can get rid of this!
        self.stick.start().onTrue(
            commands2.cmd.runOnce(self.setEncodersToStraight))

        def showOffsets():
            offsets = {}
            for i, name in enumerate("LF RF LB RB".split()):
                m = self.swerve._modules[i]._azimuth
                logger.info(f"{name}: abs={m._absolute_encoder.absolute_position.degrees():6.1f} - offset={m._offset.degrees():6.1f} => enc={m._encoder.getPosition():6.1f}")
        # RIGHT BUMPER: display the azimuth encoder offset values
        self.stick.rightBumper().onTrue(commands2.cmd.runOnce(showOffsets))

        # Set the swerve subsystem's default command to teleoperate using
        # the controller joysticks
        #
        self.swerve.setDefaultCommand(
            self.swerve.teleop_command(
                translation=self.get_translation_input,
                strafe=self.get_strafe_input,
                rotation=self.get_rotation_input,
                field_relative=SW.field_relative,
                drive_open_loop=SW.open_loop,
            )
        )

    def setEncodersToStraight(self):
        logger.info("Resetting encoders assuming wheels have been manually straightened")
        for module in self.swerve._modules:
            az = module._azimuth
            az._offset = az._absolute_encoder.absolute_position
            logger.info(f" * offset = {az._offset.degrees()}")
            az.reset()

    def set_ntval(self, topicname, value, cls='Double'):
        method_map = {
            'Double': wpilib.SmartDashboard.putNumber,
            'Int': wpilib.SmartDashboard.putNumber,
            'String': wpilib.SmartDashboard.putString,
            'Boolean': wpilib.SmartDashboard.putBoolean,
            'Data': wpilib.SmartDashboard.putData,
        }
        method = method_map[cls]
        method(f".P212/{topicname}", value)

    def log_data(self):
        for pos in ("LF", "RF", "LB", "RB"):
            encoder = getattr(self, f"{pos.lower()}_enc")
            self.set_ntval(
                f"AbsEnc Pos/{pos}", encoder.absolute_position_degrees)
        self.set_ntval(
            "NavX gyro fused heading", self.gyro.getFusedHeading())
        self.set_ntval(
            "NavX gyro yaw", self.gyro.getYaw())
        for i, loc in enumerate(('LF', 'RF', 'LB', 'RB')):
            module = self.swerve._modules[i]
            self.set_ntval(
                f"AZ/AbsEnc Offset {loc}", module._azimuth._offset.degrees())
            self.set_ntval(
                f"AZ/AbsEnc Pos {loc}", module._azimuth._absolute_encoder.absolute_position_degrees)
            self.set_ntval(
                f"AZ/RelEnc Pos {loc}", module._azimuth._encoder.getPosition())

    @staticmethod
    def deadband(value, band):
        return value if abs(value) > band else 0

    def process_joystick_input(self, val, deadband=0.03, exponent=1,
                               limit_ratio=1.0, invert=False):
        """
        Given a raw joystick reading, return the processed value after adjusting
        for real-world UX considerations:
          * apply a deadband to ignore jitter around zero
          * apply an exponent for greater low-velocity control
        """
        deadbanded_input = self.deadband(val, deadband)
        input_sign = +1 if val > 0 else -1   # this works for val=0 also
        invert_sign = -1 if invert else +1
        # abs required for fractional exponents
        scaled_input = limit_ratio * abs(deadbanded_input) ** exponent
        return invert_sign * input_sign * scaled_input

    def get_translation_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.translation_joystick_axis)
        return self.process_joystick_input(raw_stick_val, invert=invert,
                                           limit_ratio=self.speed_limit_ratio)

    def get_strafe_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.strafe_joystick_axis)
        return self.process_joystick_input(raw_stick_val, invert=invert,
                                           limit_ratio=self.speed_limit_ratio)

    def get_rotation_input(self, invert=True):
        raw_stick_val = self.stick.getRawAxis(OP.rotation_joystick_axis)
        return self.process_joystick_input(
            raw_stick_val, invert=invert,
            limit_ratio=self.angular_velocity_limit_ratio)

    def get_autonomous_command(self):
        follower_params = TrajectoryFollowerParameters(
            max_drive_velocity=4.5 * (u.m / u.s),
            theta_kP=1,
            xy_kP=1,
        )

        bezier_points = PathPlannerPath.bezierFromPoses([
            Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)),
        ])
        path = PathPlannerPath(
            bezier_points,
            PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi),
            GoalEndState(0.0, Rotation2d.fromDegrees(-90)),     # Zero velocity and facing 90 degrees clockwise
        )

        first_path = True  # reset robot pose to initial pose in trajectory
        open_loop = True   # don't use built-in motor feedback for velocity
        return self.swerve.follow_trajectory_command(path, follower_params, first_path, open_loop)
