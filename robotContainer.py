import RobotConfig
import commands2
import math
import wpilib
import pathplannerlib
import os
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder

from wpimath import geometry, kinematics, estimator
from commands2 import button, cmd
from subsystems.driveTrain import DriveTrainSubsystem
from commands.defaultDriveCommand import DefaultDriveCommand
#from commands.ArmCommands import grabberEvents

from wpilib.cameraserver import CameraServer
#from commands.ArmCommands import empty
#from commands.ArmCommands import armEvents
from commands.HardAuto import simpleAutoDrive, ModificationDrive


class RobotContainer:
    
    def __init__(self) -> None:
        
        self.joystick = button.CommandJoystick(RobotConfig.DriveConstants.Joystick.USB_ID)
        
        self.driveTrain = DriveTrainSubsystem(self.joystick)
        
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))
        
    #--------------------------------------------------------------------------------
    #Configure Auto Settings
        self.autonomousChooser = wpilib.SendableChooser()
        self.autonomousChooser.setDefaultOption("OnlyForward", "OnlyForward")
        self.autonomousChooser.addOption("Experimental", "Experimental")
        for pathName in self.autoList:
            self.autonomousChooser.addOption(pathName, pathName)
        wpilib.SmartDashboard.putData("Autonomous Chooser", self.autonomousChooser)
    #--------------------------------------------------------------------------------  
    
    def autonomousInit(self):
        pass
    
    def getAutonomousCommand(self):
        #return commands2.SequentialCommandGroup(commands2.WaitCommand(12), simpleAutoDrive(self.driveTrain))
        path = PathPlannerPath.fromPathFile('Example Path')
        return AutoBuilder.followPath(path)
        