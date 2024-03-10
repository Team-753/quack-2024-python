from subsystems.driveTrain import DriveTrainSubsystem
from commands2 import button
from commands.defaultDriveCommand import DefaultDriveCommand

import RobotConfig

class RobotContainer:
    
    def __init__(self) -> None:
        
        self.joystick = button.CommandJoystick(RobotConfig.DriveConstants.Joystick.USB_ID)
        
        self.driveTrain = DriveTrainSubsystem(self.joystick)
        
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))
    
    def autonomousInit(self):
        pass
    
    def getAutonomousCommand(self):
        pass