#from networktables import NetworkTables
import commands2
from commands2 import button
from subsystems.swerveModule import SwerveModule
import RobotConfig
import wpilib 
import math
import navx
from wpimath import geometry, kinematics, estimator, controller
import wpimath
import wpiutil
from wpilib import DriverStation

class DriveTrainSubsystem(commands2.Subsystem):

    stateStdDevs = 0.1, 0.1, 0.1
    visionMeasurementStdDevs = 0.9, 0.9, 0.9 * float(math.pi)
    def __init__(self, joystick: button.CommandJoystick) -> None:
        super().__init__()
        
        self.joystick = joystick
        '''NetworkTables.initialize() # you use networktables to access limelight data
        self.LimelightTable = NetworkTables.getTable('limelight')''' # giving us access to the limelight's data as a variable

        self.navx = navx.AHRS.create_spi(update_rate_hz=100)
        
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)

        self.kMaxSpeed = RobotConfig.DriveConstants.RobotSpeeds.maxSpeed
        self.kMaxAngularVelocity = RobotConfig.DriveConstants.RobotSpeeds.maxSpeed / math.hypot(RobotConfig.RobotDimensions.trackWidth / 2, RobotConfig.RobotDimensions.wheelBase / 2)
        self.wheelBase = RobotConfig.RobotDimensions.wheelBase
        self.trackWidth = RobotConfig.RobotDimensions.trackWidth

        self.frontLeft = SwerveModule(RobotConfig.frontLeft)
        self.frontRight = SwerveModule(RobotConfig.frontRight)
        self.rearLeft = SwerveModule(RobotConfig.rearLeft)
        self.rearRight = SwerveModule(RobotConfig.rearRight)

        self.alliance = wpilib.DriverStation.getAlliance()

        self.KINEMATICS = kinematics.SwerveDrive4Kinematics(geometry.Translation2d(float(self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(self.trackWidth / 2), float(-self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(self.wheelBase / 2)), geometry.Translation2d(float(-self.trackWidth / 2), float(-self.wheelBase / 2)))
        self.poseEstimator = estimator.SwerveDrive4PoseEstimator(self.KINEMATICS, 
                                                                self.getNAVXRotation2d(), 
                                                                self.getSwerveModulePositions(), 
                                                                geometry.Pose2d(0, 0, geometry.Rotation2d(RobotConfig.AutonomousConfig.startingRobotDirectionRadians)), 
                                                                self.stateStdDevs,
                                                                self.visionMeasurementStdDevs)
    
    def getNAVXRotation2d(self) -> geometry.Rotation2d:
        ''' Returns the NAVX estimated angle as a rotation2d object. '''
        return self.navx.getRotation2d()
    
    def resetPose(self, poseToset: geometry.Pose2d) -> None:
        ''' Resets the robot's pose to whatever is passed in, this will also offset the NAVX, useful for pathplanner autos. '''
        self.poseEstimator.resetPosition(self.getNAVXRotation2d(), self.getSwerveModulePositions(), poseToset)
    
    def shouldFlipPath() -> bool:
        ''' Boolean supplier that controls when the path will be mirrored for the red alliance '''
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    
    def getJoystickInput(self) -> tuple[float]:
        """ Returns all 3 axes on a scale from -1 to 1, if the robot driving 
        is inverted, make all these values positive instead of negative. """
        constants = RobotConfig.DriveConstants.Joystick
        return (
            -wpimath.applyDeadband(self.joystick.getY(), constants.yDeadband),
            -wpimath.applyDeadband(self.joystick.getX(), constants.xDeadband),
            -wpimath.applyDeadband(self.joystick.getZ(), constants.thetaDeadband)
        )
    
    def autoDrive(self, chassisSpeeds: kinematics.ChassisSpeeds):
        ''' Use this function for inputting pathplanner chassis speeds '''
        swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(chassisSpeeds)
        self.frontLeft.setState(swerveModuleStates[0])
        self.frontRight.setState(swerveModuleStates[1])
        self.rearLeft.setState(swerveModuleStates[2])
        self.rearRight.setState(swerveModuleStates[3])

    def joystickDrive(self, inputs: tuple[float]) -> None:
        xSpeed, ySpeed, zSpeed = (inputs[0] * self.kMaxSpeed, 
                                  inputs[1] * self.kMaxSpeed, 
                                  inputs[2] * self.kMaxAngularVelocity * RobotConfig.DriveConstants.RobotSpeeds.manualRotationSpeedFactor)
        self.setSwerveStates(xSpeed, ySpeed, zSpeed)

    def setSwerveStates(self, xSpeed: float, ySpeed: float, zSpeed: float, fieldOrient = True):
        if fieldOrient:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed), self.poseEstimator.getEstimatedPosition().rotation()))
        else:
            swerveModuleStates = self.KINEMATICS.toSwerveModuleStates(kinematics.ChassisSpeeds(xSpeed, ySpeed, zSpeed))
        
        #self.KINEMATICS.desaturateWheelSpeeds(swerveModuleStates, self.kMaxSpeed)
        self.frontLeft.setState(swerveModuleStates[0])
        self.frontRight.setState(swerveModuleStates[1])
        self.rearLeft.setState(swerveModuleStates[2])
        self.rearRight.setState(swerveModuleStates[3])

    def stationary(self):
        self.frontLeft.Brake()
        self.frontRight.Brake()
        self.rearLeft.Brake()
        self.rearRight.Brake()
    
    def coast(self):
        self.frontLeft.Coast()
        self.frontRight.Coast()
        self.rearLeft.Coast()
        self.rearRight.Coast()

    def getSwerveModulePositions(self) -> tuple[kinematics.SwerveModulePosition]:
        return self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition()
    
    def getRobotRelativeChassisSpeeds(self) -> kinematics.ChassisSpeeds:
        ''' Robot relative chassis speeds useful for pathplanner autobuilder and diagnostics. '''
        states = (self.frontLeft.getState(), self.frontRight.getState(), self.rearLeft.getState(), self.rearRight.getState())
        return self.KINEMATICS.toChassisSpeeds(states)
    
    def getPose(self) -> geometry.Pose2d:
        ''' Returns the current estimated robot position and rotation as a pose2d object. '''
        return self.poseEstimator.getEstimatedPosition()

    def periodic(self) -> None:
        ''' Runs every drivetrain subsystem loop '''
        '''if self.LimelightTable.getNumber('getpipe', 0) == 0: # 0 being our apriltag pipeline
            if self.LimelightTable.getNumber('tv', 0) == 1: # are there any valid targets
                if self.alliance == wpilib.DriverStation.Alliance.kBlue:
                    botPoseData = self.LimelightTable.getNumberArray('botpose_wpiblue', [0,0,0,0,0,0,0])
                else:
                    botPoseData = self.LimelightTable.getNumberArray('botpose_wpired', [0,0,0,0,0,0,0])
                botPose2D = geometry.Pose2d(geometry.Translation2d(botPoseData[0], botPoseData[1]), geometry.Rotation2d(botPoseData[5]))
                latency = botPoseData[6]
                self.poseEstimator.addVisionMeasurement(botPose2D, latency)'''
        currentPose = self.poseEstimator.update(
            self.getNAVXRotation2d(),
            self.getSwerveModulePositions())
        self.field.setRobotPose(currentPose)
    
    def setAlliance(self, allianceColor: wpilib.DriverStation.Alliance):
        self.alliance = allianceColor