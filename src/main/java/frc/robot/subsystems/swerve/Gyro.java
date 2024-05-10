// Adapted from team Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Gyro.java

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.NeoADIS16470;

/**
 * The Gyro subsystem integrates with the ADIS16740 IMU sensor to provide gyroscopic functionality.
 * It allows for tracking and adjusting the robot's yaw, pitch, and roll angles with offset capabilities.
 */
public class Gyro extends SubsystemBase {

  public NeoADIS16470 gyroSensor; // Renamed for clarity

  public Rotation2d yawOffsetAngle = new Rotation2d(0); // Renamed for clarity
  public Rotation2d pitchOffsetAngle = new Rotation2d(0); // Renamed for clarity
  public Rotation2d rollOffsetAngle = new Rotation2d(0); // Renamed for clarity

  /**
   * Creates a new Gyro, which is a wrapper for the ADIS16740 IMU and stores an offset so we don't
   * have to directly zero the gyro
   */
  public Gyro() {
    gyroSensor = new NeoADIS16470(); // Renamed for clarity
    zeroGyro();
  }

  /**
   * Zero the gyro
   */
  public void zeroGyro() {
    setGyroYawOffset(0);
  }

  /**
   * Sets the yaw offset for the gyro.
   * @param degrees The offset in degrees.
   */
  public void setGyroYawOffset(double degrees) {
    yawOffsetAngle = getRawYaw().minus(Rotation2d.fromDegrees(degrees)); // Renamed for clarity
  }

  /**
   * Sets the pitch offset for the gyro.
   * @param degrees The offset in degrees.
   */
  public void setGyroPitchOffset(double degrees) {
    pitchOffsetAngle = getRawPitch().minus(Rotation2d.fromDegrees(degrees)); // Renamed for clarity
  }

  /**
   * Sets the roll offset for the gyro.
   * @param degrees The offset in degrees.
   */
  public void setGyroRollOffset(double degrees) {
    rollOffsetAngle = getRawRoll().minus(Rotation2d.fromDegrees(degrees)); // Renamed for clarity
  }

  /**
   * Gets the adjusted yaw angle considering the offset.
   * @return The adjusted yaw angle as Rotation2d.
   */
  public Rotation2d getAdjustedYaw() { // Renamed for clarity
    return getRawYaw().minus(yawOffsetAngle); // Renamed for clarity
  }

  /**
   * Gets the adjusted pitch angle considering the offset.
   * @return The adjusted pitch angle as Rotation2d.
   */
  public Rotation2d getAdjustedPitch() { // Renamed for clarity
    return getRawPitch().minus(pitchOffsetAngle); // Renamed for clarity
  }

  /**
   * Gets the adjusted roll angle considering the offset.
   * @return The adjusted roll angle as Rotation2d.
   */
  public Rotation2d getAdjustedRoll() { // Renamed for clarity
    return getRawRoll().minus(rollOffsetAngle); // Renamed for clarity
  }

  /**
   * Gets the raw yaw angle from the gyro.
   * @return The raw yaw angle as Rotation2d.
   */
  public Rotation2d getRawYaw() { // Renamed for clarity
    return Rotation2d.fromDegrees(gyroSensor.getXAngle()); // Renamed for clarity
  }

  /**
   * Gets the raw pitch angle from the gyro.
   * @return The raw pitch angle as Rotation2d.
   */
  public Rotation2d getRawPitch() { // Renamed for clarity
    return Rotation2d.fromDegrees(gyroSensor.getYAngle()); // Renamed for clarity
  }

  /**
   * Gets the raw roll angle from the gyro.
   * @return The raw roll angle as Rotation2d.
   */
  public Rotation2d getRawRoll() { // Renamed for clarity
    return Rotation2d.fromDegrees(gyroSensor.getZAngle()); // Renamed for clarity
  }

  public double getXAngle(){
    return gyroSensor.getXAngle(); // Renamed for clarity
  }

  public double getYAngle(){
    return gyroSensor.getYAngle(); // Renamed for clarity
  }

  public double getZAngle(){
    return gyroSensor.getZAngle(); // Renamed for clarity
  }

  /**
   * Resets the gyro's yaw angle to zero.
   */
  public void resetGyroYaw(){ // Renamed for clarity
    gyroSensor.reset(); // Renamed for clarity
  }
  

  public double getZRate(){
    return gyroSensor.getZAngularRate(); // Renamed for clarity
  }

  public double getXRate(){
    return gyroSensor.getXAngularRate(); // Renamed for clarity
  }

  public double getYRate(){
    return gyroSensor.getYAngularRate(); // Renamed for clarity
  }

  /**
   * Resets the gyro's angles and offsets to zero.
   */
  public void reset(){
    this.resetGyroYaw(); // Renamed for clarity
    this.yawOffsetAngle = new Rotation2d(0); // Renamed for clarity
    this.rollOffsetAngle = new Rotation2d(0); // Renamed for clarity
    this.pitchOffsetAngle = new Rotation2d(0); // Renamed for clarity
  }
    
}
