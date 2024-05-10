package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The BeamBreak class represents a digital beam break sensor subsystem.
 * It uses a DigitalInput to detect whether the beam is broken or not.
 */
public class BeamBreak extends GenericDigitalPinObject{

    private double beamBreakChannel; // Renamed from 'channel' for descriptiveness

    /**
     * Constructs a BeamBreak object with a specified channel.
     * 
     * @param beamBreakChannel The digital input channel the beam break sensor is connected to.
     */
    public BeamBreak(int beamBreakChannel){
       super(beamBreakChannel);
       this.beamBreakChannel = beamBreakChannel;
    }

    /**
     * Periodically updates the SmartDashboard with the status of the beam break sensor.
     */
    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Beam Break " + beamBreakChannel, this.getStatus());
    }
}
