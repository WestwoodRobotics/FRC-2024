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
 * The SwerveDrive subsystem incorporates the control system for a swerve drive chassis.
 * It includes methods to drive the robot using joystick inputs, control individual swerve modules,
 * and update the robot's odometry based on the swerve module states.
 */
public class SwerveDrive extends SubsystemBase {

  private Gyro gyro;

  // Swerve modules
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

  // Slew rate limiters to control acceleration
  private SlewRateLimiter translationMagnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      this.getHeadingObject(),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      });

  Field2d field;

  public SwerveDrive() {
    try {
      gyro = new Gyro();
    } catch (NullPointerException e) {
      System.out.println("Warning: Gyro not responding. Skipping gyro initialization.");
      gyro = null;
    }      

    if (gyro != null) {
      gyro.reset();
    }

    field = new Field2d();
    SmartDashboard.putData("Field", field);

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

  @Override
  public void periodic() {
    odometry.update(
        Rotation2d.fromDegrees(gyro.getZAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        }
    );

    SmartDashboard.putNumber("Z Gyro Angle", gyro.getZAngle());
    SmartDashboard.putNumber("X Gyro Angle", gyro.getXAngle());
    SmartDashboard.putNumber("Y Gyro Angle", gyro.getYAngle());
    field.setRobotPose(getPose());
    SmartDashboard.putNumber("Module Velocity", frontLeftModule.getDriveEncoder().getVelocity());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetGyro(){
    if (gyro != null) {
      gyro.reset();
    } else {
      System.out.println("Warning: Gyro not responding. Skipping gyro reset.");
    }
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        this.getHeadingObject(),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    double xSpeedCommanded = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedCommanded = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotCommanded = rot * DriveConstants.kMaxAngularSpeed;

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedCommanded, ySpeedCommanded, rotCommanded, Rotation2d.fromDegrees(gyro.getZAngle()))
            : new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, rotCommanded));
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

  public void setX() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    rearLeftModule.setDesiredState(desiredStates[2]);
    rearRightModule.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    rearLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    rearRightModule.resetEncoders();
  }

  public double getHeading() {
    return gyro != null ? Rotation2d.fromDegrees(gyro.getZAngle()).getDegrees() : 0.0;
  }

  public void setGyroYawOffset(double offset){
    if (gyro != null) {
      gyro.setGyroYawOffset(offset);
    }
  } 

  public Rotation2d getHeadingObject() {
    return gyro != null ? gyro.getRawRot2dYaw() : Rotation2d.fromDegrees(0);
  }

  public double getTurnRate() {
    return gyro != null ? gyro.getZRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0) : 0.0;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        rearLeftModule.getState(),
        rearRightModule.getState()
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
    odometry.resetPosition(
      this.getHeadingObject(),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          rearLeftModule.getPosition(),
          rearRightModule.getPosition()
      },
      Pose);
  }
}
