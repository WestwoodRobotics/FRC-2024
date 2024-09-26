// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkBase.IdleMode;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 6.77;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = Math.PI/3; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    // These values are the angle offset of the wheels when the robot is facing forwards (Absolute Encoders)
    //DO NOT CHANGE THESE VALUES UNLESS YOU KNOW WHAT YOU'RE DOING!!
    public static final double kFrontLeftChassisAngularOffset = 2*Math.PI/2;
    public static final double kFrontRightChassisAngularOffset = 0; 
    public static final double kRearLeftChassisAngularOffset = Math.PI;
    public static final double kRearRightChassisAngularOffset = 0;



    public static final boolean kGyroReversed = false;

    public static final double slowModeMultiplier = 0.25;
  }

  public static final class PortConstants{
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 17;
    public static final int kFrontRightDrivingCanId = 13;
    public static final int kRearRightDrivingCanId = 15;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;




    public static final int testCANId = 0; //TODO: Change this to the CAN ID of the motor you want to test



    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kProgrammerControllerPort = 2;
  }

  public static final class TransportConstants{

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians


    // These PID Gains have been tested
    public static final double kDrivingP = 0.2;
    public static final double kDrivingI = 0; 
    public static final double kDrivingD = 0.002; 
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps; 

    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0.2;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

  }

  public static final class ControllerConstants {
    public static final double kDriveDeadband = 0.02;
  }

  public static final class AutoConstants {
    //These constants need to tuned when setting up Auton Paths
    public static final double kMaxModuleSpeedMetersPerSecond = 6.5;
     //Distance from the center of the robot to the farthest SwerveModule
    public static final double kDriveBaseRadius = 0.4; //meters


    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;


    //These control x, y, and theta corrections when doing path following, 
    //basically like a joystick input to correct for misalignment, 
    //Units are m/s per meter of offset or rad/s per radian of offset

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2;
    
    public static final double kIXController = 0;
    public static final double kIYController = 0;
    public static final double kIThetaController = 0;

    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kDThetaController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class ElevatorConstants {
    public static final int kElevatorMotor1Port = 30;
    public static final int kElevatorMotor2Port = 31;
    public static final int kRollerMotorPort = 32;
    public static final int kElevatorPivotMotorPort = 33;

    public static double kElevatorP = 0.1; //TODO: Change this value
    public static double kElevatorI = 0.02; //TODO: Change this value
    public static double kElevatorD = 0; //TODO: Change this value
    public static double kElevatorFF = 0; //TODO: Change this value

    public static double kElevatorPivotP = 0.1; //TODO: Change this value
    public static double kElevatorPivotI = 0; //TODO: Change this value
    public static double kElevatorPivotD = 0; //TODO: Change this value
    public static double kElevatorPivotFF = 0; //TODO: Change this value

    public static double kElevatorPodiumPosition = 0; //TODO: Change this value
    public static double kElevatorSourcePosition = 14.73221125;//12.595;
    public static double kElevatorAmpPosition = 15; //TODO: Change this value
    public static double kElevatorAutoShootPosition = 0;
    public static double kElevatorHomePosition = 0; //TODO: Change this value
    public static double kElevatorHandoffPosition = 0; //TODO: Change this value


    public static double kElevatorPodiumPivotPosition = 0; //TODO: Change this value
    //public static double kElevatorSourcePivotPosition = 27.5; //TODO: Change this value
    public static double kElevatorSourcePivotPosition = -17.857100; //TODO: Change this value
    public static double kElevatorAutoShootPivotPosition = -40;
    public static double kElevatorAmpPivotPosition = -28; //TODO: Change this value
    public static double kElevatorHomePivotPosition = 0; //TODO: Change this value
    public static double kElevatorHandoffPivotPosition = -7; //TODO: Change this value



    
  }
  public static final class IntakeShooterConstants {
    public static final int kUpperMotorPort = 21;
    public static final int kLowerMotorPort = 22;
    public static final int kStowMotorPort = 20;
    public static final int kPivotMotorPort = 23;


    public static double kUpperRollerP = 2; //TODO: Change this value
    public static double kUpperRollerI = 0; //TODO: Change this value
    public static double kUpperRollerD = 0; //TODO: Change this value
    public static double kUpperRollerFF = 100; //TODO: Change this value

    public static double kLowerRollerP = 0.00045; //TODO: Change this value
    public static double kLowerRollerI = 0; //TODO: Change this value
    public static double kLowerRollerD = 0; //TODO: Change this value
    public static double kLowerRollerFF = 200; //TODO: Change this value

    public static double kPivotP = 0.065; //TODO: Change this value
    public static double kPivotI = 0; //TODO: Change this value
    public static double kPivotD = 0; //TODO: Change this value
    public static double kPivotFF = 8; //TODO: Change this value



    public static double kHomePivotPosition = 4 ; //TODO: Change this value
    public static double kIntakePivotPosition = 30.0; //106.597160; //TODO: Change this value
    public static double kAutoIntakePivotPosition = 30.0; //TODO: Change this value
    public static double kShootNearSpeakerPivotPosition = 3.81;
    public static double kShootNearSpeakerFacingForwardsPivotPosition = 13.357189178466797;
    public static double kShootPodiumShot = 13.642882347106934;

    public static double kShootFarSpeakerPivotPosition = 5.435001818181818;
    public static double kShootNearSpeakerAutonPivotPosition = 4.09090909;
    public static double kShootPivotAmp = 6.1557844;
  } 
}
