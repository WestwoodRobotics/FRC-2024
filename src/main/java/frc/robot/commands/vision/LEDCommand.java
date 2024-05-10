package frc.robot.commands.vision;

import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to control the LED subsystem based on the status of the BeamBreak sensor.
 */
public class LEDCommand extends Command {

    private LED ledSubsystem; // Renamed from 'led' for clarity
    private BeamBreak beamBreakSensor; // Renamed from 'b' for clarity

    /**
     * Constructs an LEDCommand.
     * 
     * @param ledSubsystem The LED subsystem this command will control.
     * @param beamBreakSensor The BeamBreak sensor used to determine LED behavior.
     */
    public LEDCommand(LED ledSubsystem, BeamBreak beamBreakSensor){
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
        this.beamBreakSensor = beamBreakSensor;
    }

    /**
     * Executes the LEDCommand.
     * Changes LED color based on the BeamBreak sensor status.
     */
    @Override
    public void execute(){        
        if(!beamBreakSensor.getStatus()) {
            ledSubsystem.setLights(0.77); // Set LED color to green
        } else {
            ledSubsystem.setLights(0.61); // Set LED color to red
        }
        SmartDashboard.putBoolean("Beam Break Status", beamBreakSensor.getStatus());
    }
}
