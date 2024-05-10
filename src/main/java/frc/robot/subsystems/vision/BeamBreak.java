package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The BeamBreak subsystem represents a digital beam break sensor.
 * It is used to detect the presence or absence of an object interrupting a beam of light.
 */
public class BeamBreak extends SubsystemBase{
    private DigitalInput input;

    /**
     * Constructs a BeamBreak subsystem with a specified digital input channel.
     * 
     * @param channel The digital input channel the sensor is connected to.
     */
    public BeamBreak(int channel){
       input = new DigitalInput(channel);
    }

    /**
     * Returns the status of the beam break sensor.
     * 
     * @return true if the beam is interrupted (object detected), false otherwise.
     */
    public boolean getStatus(){
        return (input.get());
    }

    @Override
    public void periodic(){
        // This method will be called once per scheduler run
        // No specific periodic action defined for BeamBreak
    }
}
