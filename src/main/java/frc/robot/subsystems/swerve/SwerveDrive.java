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
 * Represents the swerve drive system. This subsystem includes methods to drive the robot using joystick inputs,
 * control individual swerve modules, and update the robot's odometry based on the swerve module states.
 */
public class SwerveDrive extends SubsystemBase {

  private Gyro gyroSubsystem;

  // Swerve modules
  private final SwerveModule frontLeftSwerveModule = new SwerveModule(
      PortConstants.kFrontLeftDrivingCanId,
      PortConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule frontRightSwerveModule = new SwerveModule(
      PortConstants.kFrontRightDrivingCanId,
      PortConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule rearLeftSwerveModule = new SwerveModule(
      PortConstants.kRearLeftDrivingCanId,
      PortConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftChassisAngularOffset);

  private final SwerveModule rearRightSwerveModule = new SwerveModule(
      PortConstants.kRearRightDrivingCanId,
      PortConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightChassisAngularOffset);

  // Slew rate limiters to control acceleration
  private SlewRateLimiter translationRateLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry swerveDriveOdometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      this.getHeadingObject(),
      new SwerveModulePosition[] {
          frontLeftSwerveModule.getPosition(),
          frontRightSwerveModule.getPosition(),
          rearLeftSwerveModule.getPosition(),
          rearRightSwerveModule.getPosition()
      });

  Field2d fieldVisualization;

  /**
   * Initializes a new instance of the SwerveDrive class.
   */
  public SwerveDrive() {
    try {
      gyroSubsystem = new Gyro();
    } catch (NullPointerException e) {
      System.out.println("Warning: Gyro not responding. Skipping gyro initialization.");
      gyroSubsystem = null;
    }      

    if (gyroSubsystem != null) {
      gyroSubsystem.reset();
    }

    fieldVisualization = new Field2d();
    SmartDashboard.putData("Field", fieldVisualization);

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      this::driveChassisSpeeds,
      new HolonomicPathFollowerConfig(
              new PIDConstants(ModuleConstants.kDrivingP+6, ModuleConstants.kDrivingI+1, ModuleConstants.kDrivingD),
              new PIDConstants(ModuleConstants.kTurningP+20, ModuleConstants.kTurningI+0.003, ModuleConstants.kTurningD),
              AutoConstants.kMaxModuleSpeedMetersPerSecond,
              AutoConstants.kDriveBaseRadius,
              new ReplanningConfig()
      ),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return (alliance.get() == DriverStation.Alliance.Red);
        }
        return false;
      },
      this
    );
  }

  /**
   * Updates the odometry of the swerve drive system.
   */
  @Override
  public void periodic() {
    swerveDriveOdometry.update(
        Rotation2d.fromDegrees(gyroSubsystem.getRawGyroObject().getZAngle()),
        new SwerveModulePosition[] {
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            rearLeftSwerveModule.getPosition(),
            rearRightSwerveModule.getPosition()
        }
    );

    SmartDashboard.putNumber("Z Gyro Angle", gyroSubsystem.getRawGyroObject().getZAngle());
    SmartDashboard.putNumber("X Gyro Angle", gyroSubsystem.getRawGyroObject().getXAngle());
    SmartDashboard.putNumber("Y Gyro Angle", gyroSubsystem.getRawGyroObject().getYAngle());
    fieldVisualization.setRobotPose(getPose());
    SmartDashboard.putNumber("Module Velocity", frontLeftSwerveModule.getDriveEncoder().getVelocity());
  }

  /**
   * Gets the current pose of the robot.
   * 
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  /**
   * Resets the gyro sensor to zero.
   */
  public void resetGyro(){
    if (gyroSubsystem != null) {
      gyroSubsystem.reset();
    } else {
      System.out.println("Warning: Gyro not responding. Skipping gyro reset.");
    }
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose to reset the odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    swerveDriveOdometry.resetPosition(
        this.getHeadingObject(),
        new SwerveModulePosition[] {
            frontLeftSwerveModule.getPosition(),
            frontRightSwerveModule.getPosition(),
            rearLeftSwerveModule.getPosition(),
            rearRightSwerveModule.getPosition()
        },
        pose);
  }

  /**
   * Drives the robot at given speeds.
   * 
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the speeds are relative to the field.
   * @param rateLimit Whether to limit the rate of change of speed.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedCommanded = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotCommanded = rot * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedCommanded, ySpeedCommanded, rotCommanded, Rotation2d.fromDegrees(gyroSubsystem.getRawGyroObject().getZAngle()))
            : new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, rotCommanded));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Drives the robot using chassis speeds.
   * 
   * @param s The chassis speeds to drive the robot.
   */
  public void driveChassisSpeeds(ChassisSpeeds s){
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(s);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
    frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
    rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
    rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the modules to an X configuration.
   */
  public void setX() {
    frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeftSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRightSwerveModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the desired state for each swerve module.
   * 
   * @param desiredStates The desired states for the swerve modules.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    frontRightSwerveModule.setDesiredState(desiredStates[1]);
    rearLeftSwerveModule.setDesiredState(desiredStates[2]);
    rearRightSwerveModule.setDesiredState(desiredStates[3]);
  }

  /**
   * Resets the encoders of all swerve modules.
   */
  public void resetEncoders() {
    frontLeftSwerveModule.resetEncoders();
    rearLeftSwerveModule.resetEncoders();
    frontRightSwerveModule.resetEncoders();
    rearRightSwerveModule.resetEncoders();
  }

  /**
   * Gets the heading of the robot.
   * 
   * @return The heading of the robot in degrees.
   */
  public double getHeading() {
    return gyroSubsystem != null ? Rotation2d.fromDegrees(gyroSubsystem.getRawGyroObject().getZAngle()).getDegrees() : 0.0;
  }

  /**
   * Sets the yaw offset of the gyro.
   * 
   * @param offset The offset to set.
   */
  public void setGyroYawOffset(double offset){
    if (gyroSubsystem != null) {
      gyroSubsystem.setGyroYawOffset(offset);
    }
  } 

  /**
   * Gets the heading of the robot as a Rotation2d object.
   * 
   * @return The heading of the robot.
   */
  public Rotation2d getHeadingObject() {
    return gyroSubsystem != null ? gyroSubsystem.getRawRot2dYaw() : Rotation2d.fromDegrees(0);
  }

  /**
   * Gets the turn rate of the robot.
   * 
   * @return The turn rate of the robot in degrees per second.
   */
  public double getTurnRate() {
    return gyroSubsystem != null ? gyroSubsystem.getZRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0) : 0.0;
  }

  /**
   * Gets the chassis speeds relative to the robot.
   * 
   * @return The chassis speeds relative to the robot.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        frontLeftSwerveModule.getState(),
        frontRightSwerveModule.getState(),
        rearLeftSwerveModule.getState(),
        rearRightSwerveModule.getState()
      }
    );
  }
  
  /**
   * Recalibrates the gyro sensor.
   */
  public void recalibrateGyro() {
    if (gyroSubsystem != null) {
      gyroSubsystem.reset();
    } else {
      System.out.println("Warning: Gyro not responding. Skipping gyro recalibration.");
    }
  }

  /**
   * Resets the pose of the robot.
   * 
   * @param Pose The new pose of the robot.
   */
  public void resetPose(Pose2d Pose){
    swerveDriveOdometry.resetPosition(
      this.getHeadingObject(),
      new SwerveModulePosition[] {
          frontLeftSwerveModule.getPosition(),
          frontRightSwerveModule.getPosition(),
          rearLeftSwerveModule.getPosition(),
          rearRightSwerveModule.getPosition()
      },
      Pose);
  }
}
