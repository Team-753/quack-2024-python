from subsystems.driveTrain import DriveTrainSubsystem
from commands2 import button
from commands.defaultDriveCommand import DefaultDriveCommand
import os
import wpilib
from wpilib import SmartDashboard, SendableChooser
import RobotConfig
import pathplannerlib
from pathplannerlib import auto
from pathplannerlib.auto import PathPlannerAuto



class RobotContainer:

    #--------------------------------------------------------------------------------
    #Autonomous Auto Select
    folderPath = os.path.dirname(os.path.abspath(__file__))
    tempAutoList = os.listdir(os.path.join(folderPath, 'deploy/pathplanner/autos'))
    autoList = []
    for pathName in tempAutoList:
            autoList.append(pathName.removesuffix(".auto"))
    #--------------------------------------------------------------------------------
    
    def __init__(self) -> None:
        
        self.joystick = button.CommandJoystick(RobotConfig.DriveConstants.Joystick.USB_ID)
        
        self.driveTrain = DriveTrainSubsystem(self.joystick)
        
        self.driveTrain.setDefaultCommand(DefaultDriveCommand(self.driveTrain))

    #--------------------------------------------------------------------------------
    #Configure Auto Settings
        self.autonomousChooser = wpilib.SendableChooser()
        self.autonomousChooser.setDefaultOption("OnlyForward", "OnlyForward")
        #self.autonomousChooser.addOption("Only Taxi", "Only Taxi")
        for pathName in self.autoList:
            self.autonomousChooser.addOption(pathName, pathName)
        wpilib.SmartDashboard.putData("Autonomous Chooser", self.autonomousChooser)
    #--------------------------------------------------------------------------------    

    #-----------------------------------------------------------------------------------------------   
    #Autonomous Start Protocol
    def getAutonomousCommand(self):
        
        """ Logic for what will run in autonomous mode. Returning anything but a command will result in nothing happening in autonomous. """
        pathName = self.autonomousChooser.getSelected()
        if pathName == "OnlyForward": 
            #return commands2.SequentialCommandGroup(ArmConfirmUp, AutoShootSpeaker)
            #return commands2.SequentialCommandGroup(commands2.WaitCommand(12), simpleAutoDrive(self.driveTrain))
            pass
        else:
            return PathPlannerAuto(pathName)
            
    #-----------------------------------------------------------------------------------------------   
    
    def autonomousInit(self):
        pass
    
    def getAutonomousCommand(self):
        pass
