package frc.robot.commands.vision;

import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command to control the LED subsystem based on the status of beam breaks.
 * This command sets the LED lights to different patterns based on whether the beam breaks are triggered.
 */
public class LEDCommand extends CommandBase {

    private LED led;
    private BeamBreak beamBreakIntakePivot, beamBreakElevatorPivot;

    /**
     * Constructs an LEDCommand.
     * 
     * @param i The LED subsystem.
     * @param IntakeShooterBeamBreak The beam break sensor for the intake shooter.
     * @param ElevatorBeamBreak The beam break sensor for the elevator.
     */
    public LEDCommand(LED i, BeamBreak IntakeShooterBeamBreak, BeamBreak ElevatorBeamBreak){
        led = i;
        addRequirements(i);
        beamBreakIntakePivot = IntakeShooterBeamBreak;
        beamBreakElevatorPivot = ElevatorBeamBreak;
    }

    /**
     * Executes the LEDCommand.
     * Sets the LED lights based on the status of the beam break sensors.
     */
    @Override
    public void execute(){        
        SmartDashboard.putBoolean("elevator beam break", beamBreakElevatorPivot.getStatus());
        SmartDashboard.putBoolean("intake shooter beam break", beamBreakIntakePivot.getStatus());

        if(!beamBreakIntakePivot.getStatus() || !beamBreakElevatorPivot.getStatus()){
            led.setLights(0.77);
        } else {
            led.setLights(0.61);
        }
    }
}
