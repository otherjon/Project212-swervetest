import sys
from typing import Optional
import commands2
import wpilib
from container import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """
    A subclass of commands2.TimedCommandRobot which represents *our* robot.

    Our robot defines most of its functionality in the RobotContainer class,
    so this class just delegates most things to the RobotContainer and tells
    the framework where to find its default things to do.
    """
    def robotInit(self) -> None:
        """
        When the robot software is initialized:
         * make a RobotContainer (that's where our robot's behavior is defined)
         * grab a copy of the CommandScheduler in case we need it later
         * create a placeholder for a future autonomous command
        """
        self.container = RobotContainer()
        self.scheduler = commands2.CommandScheduler.getInstance()
        self.autonomous_command: Optional[commands2.Command] = None
        self.container.swerve.reset_modules()

    def robotPeriodic(self) -> None:
        if hasattr(self.container, "log_data"):
            self.container.log_data()

    def autonomousInit(self) -> None:
        """
        When we enter autonomous mode:
         * get the autonomous command that our container says to use
         * if the autonomous command isn't None, schedule it
        """
        self.autonomous_command = self.container.get_autonomous_command()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self) -> None:
        """
        When we enter teleoperated mode:
         * if we were running an autonomous command, cancel it
        """
        if self.autonomous_command:
            self.autonomous_command.cancel()
            self.autonomous_command = None

# The magical line below tells Python that if we're running the robot for real,
# it should use the wpilib module's run() function to make our robot go.  (On
# the other hand, if we're just importing this file into an interactive python
# session, don't make the robot go.)
#
if __name__ == "__main__":
    wpilib.run(Robot)
