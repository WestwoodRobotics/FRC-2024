package frc.robot.subsystems.swerve;

public class DriveSpeed {
  public double xSpeed;
  public double ySpeed;
  private double brakeSpeed;
  private double lastDistance;
  private double lastAngle;

  public DriveSpeed(double brakeSpeed) {
    this.brakeSpeed = brakeSpeed;
  }

  public double[] compute(double controlX, double controlY) {
    double distance = Math.sqrt(Math.pow(controlX, 2) + Math.pow(controlY, 2)); //Pyhthagorean Theorem
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

  private void updateSpeeds(double distance, double lastAngle) {
    xSpeed = distance * Math.cos(lastAngle);
    ySpeed = distance * Math.sin(lastAngle);
  }
}