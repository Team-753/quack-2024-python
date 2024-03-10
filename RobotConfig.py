from wpimath import geometry

class RobotDimensions:
    trackWidth = 0.4954524
    wheelBase = 0.5969
    wheeDiameter = 0.1016
    
class AutonomousConfig:
    startingRobotDirectionRadians = 0

class SwerveModuleConfig:
    turningGearRatio = 12.8
    drivingGearRatio = 8.14
    driveMotorID = 0
    turnMotorID = 0
    absoluteID = 0
    absoluteOffset = 0

class frontLeft(SwerveModuleConfig):
    driveMotorID = 1
    turnMotorID = 2
    absoluteID = 0
    absoluteOffset = 77.05

class frontRight(SwerveModuleConfig):
    driveMotorID = 3
    turnMotorID = 4
    absoluteID = 1
    absoluteOffset = 309.7
    
class rearRight(SwerveModuleConfig):
    driveMotorID = 5
    turnMotorID = 6
    absoluteID = 2
    absoluteOffset = 227
    
class rearLeft(SwerveModuleConfig):
    driveMotorID = 7
    turnMotorID = 8
    absoluteID = 3
    absoluteOffset = 24.1

class Arm:
    leftMotorCanID = 9
    rightMotorCanID = 10
    limitSwitch1RIO = 0
    limitSwitch2RIO = 1

class DriveConstants:
    class Joystick:
        USB_ID = 0
        xDeadband = 0.1
        yDeadband = 0.1
        thetaDeadband = 0.15
    class RobotSpeeds:
        maxSpeed = 4.8
        maxAcceleration = 4
        manualRotationSpeedFactor = 1
        
    class PoseConstants:
        class translationPIDConstants:
            kP = 5.0
            kI = 0.0
            kD = 0.0
            period = 0.025
            
        class rotationPIDConstants:
            kP = 1.0
            kI = 0.0
            kD = 0.0
            period = 0.025
            
        xPoseToleranceMeters = 0.05
        yPoseToleranceMeters = 0.05
        thetaPoseToleranceRadians = 0.01745
        teleopVelLimit = 4.25
        teleopAccelLimit = 3
