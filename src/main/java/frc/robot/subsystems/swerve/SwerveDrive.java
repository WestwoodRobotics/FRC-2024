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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.subsystems.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The SwerveDrive class encapsulates the functionality for the robot's swerve drive system.
 * It handles the drive calculations and integrates with the gyro for orientation control.
 */
public class SwerveDrive extends SubsystemBase {

  private Gyro gyroSubsystem; // Renamed for clarity

  // Create MAXSwerveModules
  private final SwerveModule frontLeftModule = new SwerveModule(
      PortConstants.kFrontLeftDrivingCanId,
      PortConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule frontRightModule = new SwerveModule(
      PortConstants.kFrontRightDrivingCanId,
      PortConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule rearLeftModule = new SwerveModule(
      PortConstants.kRearLeftDrivingCanId,
      PortConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftChassisAngularOffset);

  private final SwerveModule rearRightModule = new SwerveModule(
      PortConstants.kRearRightDrivingCanId,
      PortConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightChassisAngularOffset);

  private double currentRotationRate = 0.0;
  private double currentTranslationDirection = 0.0;
  private double currentTranslationMagnitude = 0.0;

  // Slew rate limiters for controlling lateral acceleration
  private SlewRateLimiter translationMagnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);

  // Slew rate limiter for controlling rotational acceleration
  private SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  private double previousTime = WPIUtilJNI.now() * 1e-6; //convert from microseconds to seconds

  // Odometry class for tracking robot pose
  SwerveDriveOdometry swerveDriveOdometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      this.getGyroHeadingAsRotation2d(),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      });

  Field2d fieldVisualization;

 /** Creates a new SwerveDrive */
  public SwerveDrive() {
    try {
      gyroSubsystem = new Gyro();
    } catch (NullPointerException e) {
      System.out.println("Warning: Gyro not responding. Skipping gyro initialization.");
      gyroSubsystem = null;
    }      

    //reset the gyro if it isn't null
    if (gyroSubsystem != null) {
      gyroSubsystem.reset();
    }

    
    fieldVisualization = new Field2d();
    SmartDashboard.putData("Field", fieldVisualization);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetRobotPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(ModuleConstants.kDrivingP+6, ModuleConstants.kDrivingI+1, ModuleConstants.kDrivingD), // Translation PID constants
              new PIDConstants(ModuleConstants.kTurningP+20, ModuleConstants.kTurningI+0.003, ModuleConstants.kTurningD), // Rotation PID constants
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
    swerveDriveOdometry.update(
        Rotation2d.fromDegrees(gyroSubsystem.getZAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        }
    );


    SmartDashboard.putNumber("Z Gyro Angle", gyroSubsystem.getZAngle());
    SmartDashboard.putNumber("X Gyro Angle", gyroSubsystem.getXAngle());
    SmartDashboard.putNumber("Y Gyro Angle", gyroSubsystem.getYAngle());
    fieldVisualization.setRobotPose(getPose());
    SmartDashboard.putNumber("mod stuff", frontLeftModule.getDriveEncoder().getVelocity());

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  public void resetGyroHeading(){
    if (gyroSubsystem != null) {
      gyroSubsystem.reset();
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
    swerveDriveOdometry.resetPosition(
        this.getGyroHeadingAsRotation2d(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
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
  public void driveRobot(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMagnitude != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMagnitude);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - previousTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDirection);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDirection = SwerveUtils.StepTowardsCircular(currentTranslationDirection, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = translationMagnitudeLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMagnitude > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMagnitude = translationMagnitudeLimiter.calculate(0.0);
        }
        else {
          currentTranslationDirection = SwerveUtils.WrapAngle(currentTranslationDirection + Math.PI);
          currentTranslationMagnitude = translationMagnitudeLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDirection = SwerveUtils.StepTowardsCircular(currentTranslationDirection, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMagnitude = translationMagnitudeLimiter.calculate(0.0);
      }
      previousTime = currentTime;
      
      xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
      ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
      currentRotationRate = rotationRateLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotationRate = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotationRate * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyroSubsystem.getZAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
  }

  public void driveChassisSpeeds(ChassisSpeeds s){
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(s);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    rearLeftModule.setDesiredState(swerveModuleStates[2]);
    rearRightModule.setDesiredState(swerveModuleStates[3]);
  }


  public void driveWithTriggers(double triggerValue, double leftJoystickX, double leftJoystickY, double rightJoystickRotation, boolean fieldRelative, boolean rateLimit) {
    // Normalize the trigger value to be between 0 and 1
    double normalizedTriggerValue = MathUtil.clamp(triggerValue, 0, 1);

    // Use the left joystick X and Y values for the speed (xSpeed and ySpeed)
    double xSpeed = normalizedTriggerValue * leftJoystickX;
    double ySpeed = normalizedTriggerValue * leftJoystickY;

    // Use the right joystick for rotation
    double rot = rightJoystickRotation;

    // Call the existing drive method with these values
    driveRobot(xSpeed, ySpeed, rot, fieldRelative, rateLimit);
 }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void lockWheelsInXFormation() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetSwerveModuleEncoders() {
    frontLeftModule.resetEncoders();
    rearLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getRobotHeading() {
    if (gyroSubsystem != null) {
      return (Rotation2d.fromDegrees(gyroSubsystem.getZAngle()).getDegrees());
    } else {
      System.out.println("Warning: Gyro not responding. Returning default heading.");
      return 0.0; // Return a default value
    }
  }

  public void adjustGyroYawOffset(double offset){
    this.gyroSubsystem.setGyroYawOffset(offset);
  } 

  public Rotation2d getGyroHeadingAsRotation2d() {
    if (gyroSubsystem != null) {
      return (gyroSubsystem.getRawYaw());
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
  public double getRobotTurnRate() {
    if (gyroSubsystem != null) {
      return gyroSubsystem.getZRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    } else {
      System.out.println("Warning: Gyro not responding. Returning default turn rate.");
      return 0.0; // Return a default value
    }

  }

  //make a method that will get robot relative speeds. Returns the current robot-relative ChassisSpeeds. This can be calculated using one of WPILib's drive kinematics classes. No parms as input
  public ChassisSpeeds getCurrentRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        rearLeftModule.getState(),
        rearRightModule.getState()
      }
    );
  }
  
  public void recalibrateGyroSensor() {
    if (gyroSubsystem != null) {
      gyroSubsystem.reset();
    } else {
      System.out.println("Warning: Gyro not responding. Skipping gyro recalibration.");
    }
  }

  public void resetRobotPose(Pose2d Pose){
    swerveDriveOdometry.resetPosition(
      this.getGyroHeadingAsRotation2d(),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      },
      Pose);
  }
