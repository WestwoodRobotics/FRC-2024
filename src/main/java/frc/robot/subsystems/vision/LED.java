package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The LED subsystem controls the LED lights on the robot.
 * It uses a Spark controller to manage the LED patterns.
 * 
 * This subsystem allows for setting different LED patterns to indicate robot states or events.
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
     *              This value can range from -1.0 to 1.0, where specific values correspond
     *              to predefined patterns or colors as per the LED controller documentation.
     */
    public void setLights(double value) {
        ledController.set(value);
    }
}
