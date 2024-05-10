package frc.robot.commands.vision;

import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to control the LED subsystem based on the status of beam break sensors.
 * It sets different LED patterns based on whether the beam break sensors at the intake pivot and elevator pivot are triggered.
 */
public class LEDCommand extends Command {

    private LED ledSubsystem; // Renamed for clarity
    private BeamBreak intakePivotBeamBreakSensor, elevatorBeamBreakSensor; // Renamed for clarity

    /**
     * Constructs an LEDCommand.
     * @param ledSubsystem The LED subsystem to control
     * @param intakePivotBeamBreakSensor The beam break sensor at the intake pivot
     * @param elevatorBeamBreakSensor The beam break sensor at the elevator pivot
     */
    public LEDCommand(LED ledSubsystem, BeamBreak intakePivotBeamBreakSensor, BeamBreak elevatorBeamBreakSensor){
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
        this.intakePivotBeamBreakSensor = intakePivotBeamBreakSensor;
        this.elevatorBeamBreakSensor = elevatorBeamBreakSensor;
    }

    @Override
    public void execute(){        
        SmartDashboard.putBoolean("elevator beam break", elevatorBeamBreakSensor.getStatus());
        SmartDashboard.putBoolean("intake shooter beam break", intakePivotBeamBreakSensor.getStatus());

        if(!intakePivotBeamBreakSensor.getStatus() || !elevatorBeamBreakSensor.getStatus()){
            ledSubsystem.setLights(0.77);
        } else {
            ledSubsystem.setLights(0.61);
        }
    }   
}
