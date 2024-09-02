package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The LimitSwitch class represents a digital limit switch sensor subsystem.
 * It uses a DigitalInput to detect whether the limit switch is pressed or not.
 * 
 * This class extends GenericDigitalPinObject, providing methods to check the beam break status.
 */
public class LimitSwitch extends GenericDigitalPinObject
{
    // Digital input channel the limit switch is connected to
    private double limitSwitchChannel;

    /**
     * Constructs a LimitSwitch object with a specified channel.
     * 
     * @param channel The digital input channel the limit switch is connected to.
     */
    public LimitSwitch(int channel){
       super(channel);
       this.limitSwitchChannel = channel;
    }

    /**
     * Periodically updates the SmartDashboard with the status of the limit switch.
     * This method is called automatically to update sensor status on the dashboard.
     */
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Limit Switch " + limitSwitchChannel, this.getStatus());
    }

    /**
     * Overrides the getStatus method to invert the status of the limit switch.
     * 
     * @return True if the limit switch is not pressed, false if it is pressed.
     */
    @Override
    public boolean getStatus(){
        return !super.getStatus();
    }

}
