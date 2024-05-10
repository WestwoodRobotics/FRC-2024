package frc.robot.subsystems.swerve;

/**
 * The DriveSpeed class is responsible for calculating the drive speeds based on joystick inputs.
 * It computes the speed and direction for the robot's movement, including a braking mechanism.
 */
public class DriveSpeed {
  public double xControlInput; // Renamed from xSpeed for clarity
  public double yControlInput; // Renamed from ySpeed for clarity
  private double brakeSpeed;
  private double lastDistance;
  private double lastAngle;

  /**
   * Constructor for DriveSpeed.
   * @param brakeSpeed The speed at which the robot should brake.
   */
  public DriveSpeed(double brakeSpeed) {
    this.brakeSpeed = brakeSpeed;
  }

  /**
   * Computes the drive speeds based on joystick inputs.
   * @param xControlInput The x-axis input from the joystick.
   * @param yControlInput The y-axis input from the joystick.
   * @return An array containing the computed x and y speeds.
   */
  public double[] compute(double xControlInput, double yControlInput) {
    double distance = Math.sqrt(Math.pow(xControlInput, 2) + Math.pow(yControlInput, 2)); //Pythagorean Theorem
    double angle = Math.atan2(yControlInput, xControlInput);

    // if distance is 0, start decreasing the speed by the brake speed
    if (distance == 0) {
      lastDistance = Math.max(lastDistance - brakeSpeed, 0);

      updateSpeeds(lastDistance, lastAngle);
    } else {
      lastDistance = distance;
      lastAngle = angle;

      updateSpeeds(distance, angle);
    }

    return new double[]{xControlInput, yControlInput};
  }

  /**
   * Updates the speeds based on the computed distance and angle.
   * @param distance The computed distance.
   * @param lastAngle The computed angle.
   */
  private void updateSpeeds(double distance, double lastAngle) {
    xControlInput = distance * Math.cos(lastAngle);
    yControlInput = distance * Math.sin(lastAngle);
  }
}
