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
    def __init__(self) -> None:
        self.driveMotorID = 0
        self.turnMotorID = 0
        self.absoluteID = 0
        self.absoluteOffset = 0

class frontLeft(SwerveModuleConfig):
    def __init__(self) -> None:
        super().__init__()
        self.driveMotorID = 1
        self.turnMotorID = 2
        self.absoluteID = 0
        self.absoluteOffset = 77.05

class frontRight(SwerveModuleConfig):
    def __init__(self) -> None:
        super().__init__()
        self.driveMotorID = 3
        self.turnMotorID = 4
        self.absoluteID = 1
        self.absoluteOffset = 309.7
    
class rearRight(SwerveModuleConfig):
    def __init__(self) -> None:
        super().__init__()
        self.driveMotorID = 5
        self.turnMotorID = 6
        self.absoluteID = 2
        self.absoluteOffset = 227
    
class rearLeft(SwerveModuleConfig):
    def __init__(self) -> None:
        super().__init__()
        self.driveMotorID = 7
        self.turnMotorID = 8
        self.absoluteID = 3
        self.absoluteOffset = 24.1

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
