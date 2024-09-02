// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Main class serves as the entry point for the Java program. It is responsible for initializing
 * the robot application. This class strictly adheres to the WPILib's framework for robot applications
 * and should not contain any robot logic or initialization code outside of the startRobot call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. This method is called when the application starts and is responsible
   * for initializing the robot application. The method uses the startRobot method from the RobotBase
   * class to start the robot application.
   *
   * @param arguments The command line arguments passed to the application. This parameter is not used
   *                  in this application but is required for compatibility with the RobotBase.startRobot method.
   */
  public static void main(String... arguments) {
    RobotBase.startRobot(Robot::new);
  }
}
