import RobotConfig
import phoenix6
import wpilib
from wpimath import kinematics, geometry
from math import pi

class SwerveModule:
    def __init__(self, swerveModuleConfig: RobotConfig.SwerveModuleConfig) -> None:
        self.m_request_velocity = phoenix6.controls.VelocityVoltage(0).with_slot(0)
        self.m_request_position = phoenix6.controls.PositionVoltage(0).with_slot(0)
        self.m_coast = phoenix6.controls.CoastOut()
        self.m_brake = phoenix6.controls.StaticBrake()
        
        self.driveMotor = phoenix6.hardware.TalonFX(swerveModuleConfig.driveMotorID)
        self.turnMotor = phoenix6.hardware.TalonFX(swerveModuleConfig.turnMotorID)
        self.analogEncoder = wpilib.AnalogEncoder(swerveModuleConfig.absoluteID)
        self.absoluteOffset = swerveModuleConfig.absoluteOffset
        
        turnMotorConfig = phoenix6.configs.TalonFXConfiguration()
        turnMotorConfig.slot0.k_p = 1
        turnMotorConfig.slot0.k_i = 0.01
        turnMotorConfig.slot0.k_d = 0.0001
        turnMotorConfig.slot0.k_v = 0
        turnMotorConfig.current_limits.supply_current_limit = 35
        turnMotorConfig.closed_loop_general.continuous_wrap = True
        self.turnMotor.configurator.apply(turnMotorConfig, 0.05)
        self.turnMotor.set_position(self.__getAbsolutePosition() * RobotConfig.SwerveModuleConfig.turningGearRatio, 0.05)
        
        driveMotorConfig = phoenix6.configs.TalonFXConfiguration()
        driveMotorConfig.slot0.k_p = 0.0005
        driveMotorConfig.slot0.k_i = 0.0005
        driveMotorConfig.slot0.k_d = 0
        driveMotorConfig.slot0.k_v = 1
        driveMotorConfig.slot0.k_s = 0.0875
        driveMotorConfig.current_limits.supply_current_limit = 60
        self.driveMotor.configurator.apply(driveMotorConfig, 0.05)
        self.driveMotor.set_position(0, 0.05)
        
        
    def __getAbsolutePosition(self) -> float:
        return ((self.analogEncoder.getAbsolutePosition() * 360) - self.absoluteOffset) / 360
    
    def getZeroingAbsolutePosition(self) -> float:
        return self.analogEncoder.getAbsolutePosition() * 360
    
    def getPosition(self) -> kinematics.SwerveModulePosition:
        return kinematics.SwerveModulePosition(self.driveMotor.get_position().value_as_double * RobotConfig.RobotDimensions.wheeDiameter * pi / RobotConfig.SwerveModuleConfig.drivingGearRatio,
                                               geometry.Rotation2d(2 * pi * self.turnMotor.get_position().value_as_double / RobotConfig.SwerveModuleConfig.turningGearRatio))
    
    def getState(self) -> kinematics.SwerveModuleState:
        return kinematics.SwerveModuleState(self.driveMotor.get_velocity().value_as_double * RobotConfig.RobotDimensions.wheeDiameter * pi / RobotConfig.SwerveModuleConfig.drivingGearRatio, 
                                            geometry.Rotation2d(2 * pi * self.turnMotor.get_position().value_as_double / RobotConfig.SwerveModuleConfig.turningGearRatio))
        
    def setState(self, desiredState: kinematics.SwerveModuleState) -> None:
        optimizedDesiredState = kinematics.SwerveModuleState.optimize(desiredState, 
                                                                      geometry.Rotation2d(2 * pi * self.turnMotor.get_position().value_as_double / RobotConfig.SwerveModuleConfig.turningGearRatio))
        self.driveMotor.set_control(self.m_request_velocity.with_velocity(optimizedDesiredState.speed * RobotConfig.SwerveModuleConfig.drivingGearRatio / (RobotConfig.RobotDimensions.wheeDiameter * pi)))
        self.turnMotor.set_control(self.m_request_position.with_position(optimizedDesiredState.angle.radians() * RobotConfig.SwerveModuleConfig.turningGearRatio / (2 * pi)))
        
    def Brake(self):
        self.driveMotor.set_control(self.m_brake)
        self.turnMotor.set_control(self.m_brake)
        
    def Coast(self):
        self.driveMotor.set_control(self.m_coast)
        self.driveMotor.set_control(self.m_coast)