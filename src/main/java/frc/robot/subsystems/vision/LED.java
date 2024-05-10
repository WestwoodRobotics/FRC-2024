package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The LED subsystem controls the LED lights on the robot.
 * It uses a Spark controller to manage the LED patterns.
 */
public class LED extends SubsystemBase {
    
    // Controller for the LED lights
    Spark ledController;

    /**
     * Constructs the LED subsystem.
     * 
     * @param channel The PWM channel the LED controller is connected to.
     */
    public LED(int channel) {
        ledController = new Spark(channel);
    }

    /**
     * Sets the LED lights to a specific pattern or color.
     * 
     * @param value The value corresponding to a specific pattern or color.
     */
    public void setLights(double value) {
        ledController.set(value);
    }
}
