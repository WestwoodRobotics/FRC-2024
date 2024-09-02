package frc.robot.subsystems.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

/**
 * Represents a swerve module with driving and turning capabilities.
 * This class encapsulates the functionality for controlling a single swerve module,
 * including motor control, encoder feedback, and state management.
 */
public class SwerveModule {
  private final CANSparkFlex drivingMotorController;
  private final CANSparkMax turningMotorController;

  private final RelativeEncoder drivingMotorEncoder;
  private final AbsoluteEncoder turningMotorEncoder;

  private final SparkPIDController drivingMotorPIDController;
  private final SparkPIDController turningMotorPIDController;

  private double moduleChassisAngularOffset = 0;
  private SwerveModuleState moduleDesiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a SwerveModule with specified CAN IDs for driving and turning motors.
   * 
   * @param drivingCANId The CAN ID for the driving motor.
   * @param turningCANId The CAN ID for the turning motor.
   * @param chassisAngularOffset The angular offset of the module relative to the robot chassis.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    drivingMotorController = new CANSparkFlex(drivingCANId, CANSparkFlex.MotorType.kBrushless);
    turningMotorController = new CANSparkMax(turningCANId, CANSparkMax.MotorType.kBrushless);

    drivingMotorController.restoreFactoryDefaults();
    turningMotorController.restoreFactoryDefaults();

    drivingMotorEncoder = drivingMotorController.getEncoder();
    turningMotorEncoder = turningMotorController.getAbsoluteEncoder(Type.kDutyCycle);
    
    drivingMotorPIDController = drivingMotorController.getPIDController();
    turningMotorPIDController = turningMotorController.getPIDController();
    drivingMotorPIDController.setFeedbackDevice(drivingMotorEncoder);
    turningMotorPIDController.setFeedbackDevice(turningMotorEncoder);

    drivingMotorEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    drivingMotorEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    turningMotorEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turningMotorEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    turningMotorEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    turningMotorPIDController.setPositionPIDWrappingEnabled(true);
    turningMotorPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningMotorPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    drivingMotorPIDController.setP(ModuleConstants.kDrivingP);
    drivingMotorPIDController.setI(ModuleConstants.kDrivingI);
    drivingMotorPIDController.setD(ModuleConstants.kDrivingD);
    drivingMotorPIDController.setFF(ModuleConstants.kDrivingFF);
    drivingMotorPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    turningMotorPIDController.setP(ModuleConstants.kTurningP);
    turningMotorPIDController.setI(ModuleConstants.kTurningI);
    turningMotorPIDController.setD(ModuleConstants.kTurningD);
    turningMotorPIDController.setFF(ModuleConstants.kTurningFF);
    turningMotorPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    drivingMotorController.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    turningMotorController.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    drivingMotorController.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningMotorController.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    drivingMotorController.burnFlash();
    turningMotorController.burnFlash();

    moduleChassisAngularOffset = chassisAngularOffset;
    moduleDesiredState.angle = new Rotation2d(turningMotorEncoder.getPosition());
    drivingMotorEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(drivingMotorEncoder.getVelocity(),
        new Rotation2d(turningMotorEncoder.getPosition() - moduleChassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        drivingMotorEncoder.getPosition(),
        new Rotation2d(turningMotorEncoder.getPosition() - moduleChassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(moduleChassisAngularOffset));

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningMotorEncoder.getPosition()));

    drivingMotorPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningMotorPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    moduleDesiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingMotorEncoder.setPosition(0);
  }

  /**
   * Moves the module to what it thinks is the position (0, 0).
   */
  public void moveToZero() {
    SwerveModuleState zeroState = new SwerveModuleState(0.0, new Rotation2d());

    setDesiredState(zeroState);
  }

  public RelativeEncoder getDriveEncoder(){
    return drivingMotorController.getEncoder();
  }
  public double getDriveEncoderPosition() {
    return drivingMotorEncoder.getPosition();
  }
  
  public double getTurningEncoderPosition() {
    return turningMotorEncoder.getPosition();
  }
}
