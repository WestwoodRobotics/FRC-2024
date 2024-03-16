// Adapted from team Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Gyro.java

package frc.robot.subsystems.swerve;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.NeoADIS16470;


public class Gyro extends SubsystemBase {

  public NeoADIS16470 gyro;

  public Rotation2d yawOffset = new Rotation2d(0);
  public Rotation2d pitchOffset = new Rotation2d(0);
  public Rotation2d rollOffset = new Rotation2d(0);



  /**
   * Creates a new Gyro, which is a wrapper for the ADIS16740 IMU and stores an offset so we don't
   * have to directly zero the gyro
   */
  public Gyro() {
    gyro = new NeoADIS16470();
    zeroGyro();
  }

  /**
   * Zero the gyro
   */
  public void zeroGyro() {
    setGyroYawOffset(0);
  }

  public void setGyroYawOffset(double degrees) {
    yawOffset = getRawRot2dYaw().minus(Rotation2d.fromDegrees(degrees));
  }

  public void setGyroPitchOffset(double degrees) {
    pitchOffset = getRawRot2dPitch().minus(Rotation2d.fromDegrees(degrees));
  }

  public void setGyroRollOffset(double degrees) {
    rollOffset = getRawRot2dRoll().minus(Rotation2d.fromDegrees(degrees));
  }


  public Rotation2d getProcessedRot2dYaw() {
    return getRawRot2dYaw().minus(yawOffset);
  }

  public Rotation2d getProcessedRot2dPitch() {
    return getRawRot2dPitch().minus(pitchOffset);
  }

  public Rotation2d getProcessedRot2dRoll() {
    return getRawRot2dRoll().minus(rollOffset);
  }



  public Rotation2d getRawRot2dYaw() {
    return Rotation2d.fromDegrees(gyro.getXAngle());
  }

  public Rotation2d getRawRot2dPitch() {
    return Rotation2d.fromDegrees(gyro.getYAngle());
  }

  public Rotation2d getRawRot2dRoll() {
    return Rotation2d.fromDegrees(gyro.getZAngle());
  }

  public double getXAngle(){
    return gyro.getXAngle();
  }

  public double getYAngle(){
    return gyro.getYAngle();
  }

  public double getZAngle(){
    return gyro.getZAngle();
  }

  public void resetYaw(){
    gyro.reset();
  }
  

  public double getZRate(){
    return gyro.getZAngularRate();
  }

  public double getXRate(){
    return gyro.getXAngularRate();
  }

  public double getYRate(){
    return gyro.getYAngularRate();
  }


  public void reset(){
    this.resetYaw();
  }
    
}