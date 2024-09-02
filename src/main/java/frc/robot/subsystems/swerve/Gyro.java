// Adapted from team Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Gyro.java

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.NeoADIS16470;

/**
 * The Gyro class is a subsystem that encapsulates the functionality of the ADIS16470 IMU sensor.
 * It provides methods to access the sensor's yaw, pitch, and roll angles, as well as angular rates.
 * Additionally, it supports setting offsets for each angle to calibrate the sensor's zero position.
 */
public class Gyro extends SubsystemBase {

  public NeoADIS16470 gyroSensor; // Renamed from 'gyroscope' to 'gyroSensor' for clarity

  public Rotation2d gyroYawOffset = new Rotation2d(0); // Renamed from 'yawOffset' to 'gyroYawOffset' for clarity
  public Rotation2d gyroPitchOffset = new Rotation2d(0); // Renamed from 'pitchOffset' to 'gyroPitchOffset' for clarity
  public Rotation2d gyroRollOffset = new Rotation2d(0); // Renamed from 'rollOffset' to 'gyroRollOffset' for clarity

  /**
   * Constructs a Gyro object, initializing the ADIS16470 IMU sensor and zeroing its angles.
   */
  public Gyro() {
    gyroSensor = new NeoADIS16470();
    zeroGyro();
  }

  /**
   * Zeros the gyro's yaw angle by setting the current yaw angle as the offset.
   */
  public void zeroGyro() {
    setGyroYawOffset(0);
  }

  /**
   * Sets the yaw offset to a specific angle.
   * @param degrees The angle in degrees to set as the yaw offset.
   */
  public void setGyroYawOffset(double degrees) {
    gyroYawOffset = getRawRot2dYaw().minus(Rotation2d.fromDegrees(degrees));
  }

  /**
   * Sets the pitch offset to a specific angle.
   * @param degrees The angle in degrees to set as the pitch offset.
   */
  public void setGyroPitchOffset(double degrees) {
    gyroPitchOffset = getRawRot2dPitch().minus(Rotation2d.fromDegrees(degrees));
  }

  /**
   * Sets the roll offset to a specific angle.
   * @param degrees The angle in degrees to set as the roll offset.
   */
  public void setGyroRollOffset(double degrees) {
    gyroRollOffset = getRawRot2dRoll().minus(Rotation2d.fromDegrees(degrees));
  }

  /**
   * Gets the processed yaw angle, accounting for the offset.
   * @return The processed yaw angle as a Rotation2d object.
   */
  public Rotation2d getProcessedRot2dYaw() {
    return getRawRot2dYaw().minus(gyroYawOffset);
  }

  /**
   * Gets the processed pitch angle, accounting for the offset.
   * @return The processed pitch angle as a Rotation2d object.
   */
  public Rotation2d getProcessedRot2dPitch() {
    return getRawRot2dPitch().minus(gyroPitchOffset);
  }

  /**
   * Gets the processed roll angle, accounting for the offset.
   * @return The processed roll angle as a Rotation2d object.
   */
  public Rotation2d getProcessedRot2dRoll() {
    return getRawRot2dRoll().minus(gyroRollOffset);
  }

  /**
   * Gets the raw yaw angle directly from the sensor.
   * @return The raw yaw angle as a Rotation2d object.
   */
  public Rotation2d getRawRot2dYaw() {
    return Rotation2d.fromDegrees(gyroSensor.getXAngle());
  }

  /**
   * Gets the raw pitch angle directly from the sensor.
   * @return The raw pitch angle as a Rotation2d object.
   */
  public Rotation2d getRawRot2dPitch() {
    return Rotation2d.fromDegrees(gyroSensor.getYAngle());
  }

  /**
   * Gets the raw roll angle directly from the sensor.
   * @return The raw roll angle as a Rotation2d object.
   */
  public Rotation2d getRawRot2dRoll() {
    return Rotation2d.fromDegrees(gyroSensor.getZAngle());
  }

  /**
   * Resets the yaw angle to zero.
   */
  public void resetYaw(){
    gyroSensor.reset();
  }

  /**
   * Gets the angular rate around the Z-axis.
   * @return The Z-axis angular rate in degrees per second.
   */
  public double getZRate(){
    return gyroSensor.getZAngularRate();
  }

  /**
   * Gets the angular rate around the X-axis.
   * @return The X-axis angular rate in degrees per second.
   */
  public double getXRate(){
    return gyroSensor.getXAngularRate();
  }

  /**
   * Gets the angular rate around the Y-axis.
   * @return The Y-axis angular rate in degrees per second.
   */
  public double getYRate(){
    return gyroSensor.getYAngularRate();
  }

  /**
   * Provides direct access to the raw gyroscope object.
   * @return The raw NeoADIS16470 gyroscope object.
   */
  public NeoADIS16470 getRawGyroObject() {
    return gyroSensor;
  }
  /**
   * Resets all gyro angles and offsets to zero.
   */
  public void reset(){
    this.resetYaw();
    this.gyroYawOffset = new Rotation2d(0);
    this.gyroRollOffset = new Rotation2d(0);
    this.gyroPitchOffset = new Rotation2d(0);
  }
}
