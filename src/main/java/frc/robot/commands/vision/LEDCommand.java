package frc.robot.commands.vision;

import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Command to control the LED subsystem based on the status of beam breaks.
 * This command sets the LED lights to different patterns based on whether the beam breaks are triggered.
 */
public class LEDCommand extends Command {

    private LED ledSubsystem;
    private BeamBreak intakePivotBeamBreakSensor, elevatorPivotBeamBreakSensor;

    /**
     * Constructs an LEDCommand.
     * 
     * @param i The LED subsystem.
     * @param IntakeShooterBeamBreak The beam break sensor for the intake shooter.
     * @param ElevatorBeamBreak The beam break sensor for the elevator.
     */
    public LEDCommand(LED i, BeamBreak IntakeShooterBeamBreak, BeamBreak ElevatorBeamBreak){
        ledSubsystem = i;
        addRequirements(i);
        intakePivotBeamBreakSensor = IntakeShooterBeamBreak;
        elevatorPivotBeamBreakSensor = ElevatorBeamBreak;
    }

    /**
     * Executes the LEDCommand.
     * Sets the LED lights based on the status of the beam break sensors.
     */
    @Override
    public void execute(){        
        SmartDashboard.putBoolean("elevator beam break", elevatorPivotBeamBreakSensor.getStatus());
        SmartDashboard.putBoolean("intake shooter beam break", intakePivotBeamBreakSensor.getStatus());

        if(!intakePivotBeamBreakSensor.getStatus() || !elevatorPivotBeamBreakSensor.getStatus()){
            ledSubsystem.setLights(0.77);
        } else {
            ledSubsystem.setLights(0.61);
        }
    }
}
