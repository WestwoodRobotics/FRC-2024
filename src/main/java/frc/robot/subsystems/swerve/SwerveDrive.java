// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

//import com.kauailabs.navx.frc.AHRS;
//import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.subsystems.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
//Import auto builder





public class SwerveDrive extends SubsystemBase {

  //private final PIDController m_headingController = new PIDController(
   // DriveConstants.kSwerveP, DriveConstants.kSwerveI, DriveConstants.kSwerveD
 // );

  private Gyro gyro;
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      PortConstants.kFrontLeftDrivingCanId,
      PortConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      PortConstants.kFrontRightDrivingCanId,
      PortConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      PortConstants.kRearLeftDrivingCanId,
      PortConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      PortConstants.kRearRightDrivingCanId,
      PortConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightChassisAngularOffset);

  // The gyro sensor

  // public static final AHRS gyro = new AHRS(SPI.Port.kMXP); //NAVXMP v1 Initialization

  //Instangiate a CTRE Pigeon Gyro


  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  // Slew rate limiters for controlling lateral acceleration
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);

  // Slew rate limiter for controlling rotational acceleration
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  private double m_prevTime = WPIUtilJNI.now() * 1e-6; //convert from microseconds to seconds

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      this.getHeadingObject(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  Field2d m_field;

 /** Creates a new SwerveDrive */
  public SwerveDrive() {
    try {
      gyro = new Gyro();
    } catch (NullPointerException e) {
      System.out.println("Warning: Gyro not responding. Skipping gyro initialization.");
      gyro = null;
    }      

    //reset the gyro if it isn't null
    if (gyro != null) {
      gyro.reset();
    }

    
    m_field = new Field2d();
    SmartDashboard.putData("Field", m_field);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(ModuleConstants.kDrivingP+3, ModuleConstants.kDrivingI+1, ModuleConstants.kDrivingD), // Translation PID constants
              new PIDConstants(ModuleConstants.kTurningP+6, ModuleConstants.kTurningI+0.001, ModuleConstants.kTurningD), // Rotation PID constants
              AutoConstants.kMaxModuleSpeedMetersPerSecond, // Max module speed, in m/s
              AutoConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return (alliance.get() == DriverStation.Alliance.Red);
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(gyro.getZAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }
    );


    SmartDashboard.putNumber("Z Gyro Angle", gyro.getZAngle());
    SmartDashboard.putNumber("X Gyro Angle", gyro.getXAngle());
    SmartDashboard.putNumber("Y Gyro Angle", gyro.getYAngle());
    m_field.setRobotPose(getPose());
    SmartDashboard.putNumber("mod stuff", m_frontLeft.getDriveEncoder().getVelocity());

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetGyro(){
    if (gyro != null) {
      gyro.reset();
      System.out.println("RESET GYRO!");
    } else {
      System.out.println("Warning: Gyro not responding. Skipping gyro reset.");
    }
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        this.getHeadingObject(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getZAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveChassisSpeeds(ChassisSpeeds s){
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(s);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  public void TriggerDrive(double triggerValue, double leftJoystickX, double leftJoystickY, double rightJoystickRotation, boolean fieldRelative, boolean rateLimit) {
    // Normalize the trigger value to be between 0 and 1
    double normalizedTriggerValue = MathUtil.clamp(triggerValue, 0, 1);

    // Use the left joystick X and Y values for the speed (xSpeed and ySpeed)
    double xSpeed = normalizedTriggerValue * leftJoystickX;
    double ySpeed = normalizedTriggerValue * leftJoystickY;

    // Use the right joystick for rotation
    double rot = rightJoystickRotation;

    // Call the existing drive method with these values
    drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
 }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */


  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    if (gyro != null) {
      return (Rotation2d.fromDegrees(gyro.getZAngle()).getDegrees());
    } else {
      System.out.println("Warning: Gyro not responding. Returning default heading.");
      return 0.0; // Return a default value
    }
  }

  public void setGyroYawOffset(double offset){
    this.gyro.setGyroYawOffset(offset);
  } 

  public Rotation2d getHeadingObject() {
    if (gyro != null) {
      return (gyro.getRawRot2dYaw());
    } else {
      System.out.println("Warning: Gyro not responding. Returning default heading.");
      return Rotation2d.fromDegrees(0); // Return a default value
    }

  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    if (gyro != null) {
      return gyro.getZRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    } else {
      System.out.println("Warning: Gyro not responding. Returning default turn rate.");
      return 0.0; // Return a default value
    }

  }

  //make a method that will get robot relative speeds. Returns the current robot-relative ChassisSpeeds. This can be calculated using one of WPILib's drive kinematics classes. No parms as input
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
      }
    );
  }
  
  public void recalibrateGyro() {
    if (gyro != null) {
      gyro.reset();
    } else {
      System.out.println("Warning: Gyro not responding. Skipping gyro recalibration.");
    }
  }

  public void resetPose(Pose2d Pose){
    m_odometry.resetPosition(
      this.getHeadingObject(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      Pose);
  }





  
}