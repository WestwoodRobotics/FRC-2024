package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The GenericDigitalPinObject class serves as a base class for subsystems that utilize a digital input pin.
 * It provides a common structure for managing a digital input, including reading its status.
 */
public abstract class GenericDigitalPinObject extends SubsystemBase{
    private DigitalInput input;

    /**
     * Constructs a GenericDigitalPinObject with a specified digital input channel.
     * 
     * @param channel The digital input channel the object is connected to.
     */
    public GenericDigitalPinObject(int channel){
       input = new DigitalInput(channel);
    }

    /**
     * Returns the status of the digital input.
     * 
     * @return True if the input is high, false if it is low.
     */
    public boolean getStatus(){
        return (input.get());
    }

    @Override
    public void periodic(){
    }
}
