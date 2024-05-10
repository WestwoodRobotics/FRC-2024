// Adapted from team Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Gyro.java

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.NeoADIS16470;

/**
 * The Gyro subsystem integrates with the ADIS16740 IMU sensor to provide gyroscopic functionality.
 * It allows for obtaining the robot's orientation and angular rate.
 */
public class Gyro extends SubsystemBase {

  private NeoADIS16470 gyro;
  private Rotation2d yawOffset = new Rotation2d(0);

  /**
   * Constructs a Gyro subsystem, initializing the gyro sensor and setting the initial yaw offset.
   */
  public Gyro() {
    gyro = new NeoADIS16470();
    zeroGyro();
  }

  /**
   * Zeros the gyro, effectively setting the current orientation as the reference point.
   */
  public void zeroGyro() {
    setGyroYawOffset(0);
  }

  /**
   * Sets the gyro's yaw offset to a specific degree.
   * 
   * @param degrees The degree to set as the yaw offset.
   */
  public void setGyroYawOffset(double degrees) {
    yawOffset = getRawRot2dYaw().minus(Rotation2d.fromDegrees(degrees));
  }

  /**
   * Gets the processed yaw value as a Rotation2d, accounting for the yaw offset.
   * 
   * @return The processed yaw value.
   */
  public Rotation2d getProcessedRot2dYaw() {
    return getRawRot2dYaw().minus(yawOffset).times(-1);
  }

  /**
   * Gets the raw yaw value directly from the gyro as a Rotation2d.
   * 
   * @return The raw yaw value.
   */
  public Rotation2d getRawRot2dYaw() {
    return Rotation2d.fromDegrees(gyro.getXAngle());
  }

  /**
   * Gets the X angle from the gyro.
   * 
   * @return The X angle in degrees.
   */
  public double getXAngle(){
    return gyro.getXAngle();
  }

  /**
   * Gets the Y angle from the gyro.
   * 
   * @return The Y angle in degrees.
   */
  public double getYAngle(){
    return gyro.getYAngle();
  }

  /**
   * Gets the Z angle from the gyro.
   * 
   * @return The Z angle in degrees.
   */
  public double getZAngle(){
    return gyro.getZAngle();
  }

  /**
   * Resets the gyro's yaw value to zero.
   */
  public void resetYaw(){
    gyro.reset();
  }

  /**
   * Gets the Z angular rate from the gyro.
   * 
   * @return The Z angular rate in degrees per second.
   */
  public double getZRate(){
    return gyro.getZAngularRate();
  }

  /**
   * Gets the X angular rate from the gyro.
   * 
   * @return The X angular rate in degrees per second.
   */
  public double getXRate(){
    return gyro.getXAngularRate();
  }

  /**
   * Gets the Y angular rate from the gyro.
   * 
   * @return The Y angular rate in degrees per second.
   */
  public double getYRate(){
    return gyro.getYAngularRate();
  }

  /**
   * Resets the gyro, setting all angles and rates back to zero.
   */
  public void reset(){
    this.resetYaw();
  }
}
