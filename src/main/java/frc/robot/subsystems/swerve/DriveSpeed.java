package frc.robot.subsystems.swerve;

/**
 * This class is responsible for calculating the drive speeds based on control inputs.
 * It computes the x and y speeds for the robot, taking into account a braking mechanism.
 */
public class DriveSpeed {
  public double xSpeed; // Speed of the robot in the x direction
  public double ySpeed; // Speed of the robot in the y direction
  private double brakeSpeed; // Speed reduction factor for braking
  private double lastDistance; // Last computed distance from the origin
  private double lastAngle; // Last computed angle of movement

  /**
   * Constructor for DriveSpeed.
   * 
   * @param brakeSpeed The speed reduction factor for braking.
   */
  public DriveSpeed(double brakeSpeed) {
    this.brakeSpeed = brakeSpeed;
  }

  /**
   * Computes the drive speeds based on control inputs.
   * 
   * @param controlX The x control input.
   * @param controlY The y control input.
   * @return An array containing the computed x and y speeds.
   */
  public double[] compute(double controlX, double controlY) {
    double distance = Math.sqrt(Math.pow(controlX, 2) + Math.pow(controlY, 2)); //Pythagorean Theorem
    double angle = Math.atan2(controlY, controlX);

    // if distance is 0, start decreasing the speed by the brake speed
    if (distance == 0) {
      lastDistance = Math.max(lastDistance - brakeSpeed, 0);

      updateSpeeds(lastDistance, lastAngle);
    } else {
      lastDistance = distance;
      lastAngle = angle;

      updateSpeeds(distance, angle);
    }

    return new double[]{xSpeed, ySpeed};
  }

  /**
   * Updates the x and y speeds based on the given distance and angle.
   * 
   * @param distance The distance from the origin.
   * @param lastAngle The angle of movement.
   */
  private void updateSpeeds(double distance, double lastAngle) {
    xSpeed = distance * Math.cos(lastAngle);
    ySpeed = distance * Math.sin(lastAngle);
  }
}
