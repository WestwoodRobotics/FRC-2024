package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.vision.BeamBreak;

/**
 * Command to activate the intake shooter when the beam break sensor is triggered.
 * This command activates the intake shooter subsystem based on the status of a beam break sensor.
 */
public class IntakeBeamBreakCommand extends Command{
    Timer t = new Timer();
    private IntakeShooter intakeShooterSubsystem; // Renamed for clarity
    private BeamBreak beamBreakSensor; // Renamed for clarity

    private boolean isFinished;

    /**
     * Constructs an IntakeBeamBreakCommand.
     * @param intakeShooterSubsystem The intake shooter subsystem to be controlled
     * @param beamBreakSensor The beam break sensor to monitor
     */
    public IntakeBeamBreakCommand(IntakeShooter intakeShooterSubsystem, BeamBreak beamBreakSensor){
        this.intakeShooterSubsystem = intakeShooterSubsystem;
        this.beamBreakSensor = beamBreakSensor;
    }

    /**
     * Initializes the command with a timer reset.
     */
    @Override
    public void initialize(){
        t.reset(); 
        t.start();
    }

    /**
     * Executes the command, activating the intake shooter when the beam break sensor is triggered.
     */
    @Override
    public void execute(){
        intakeShooterSubsystem.setRollerPower(1);
        intakeShooterSubsystem.setStowPower(-1);
        isFinished = false;
    }

    /**
     * Determines if the command has finished.
     * @return true if the beam break sensor is not triggered, false otherwise.
     */
    @Override
    public boolean isFinished(){
        return !beamBreakSensor.getStatus();
    }

    /**
     * Ends the command, stopping the intake shooter.
     * @param interrupted Whether the command was interrupted or not
     */
    @Override
    public void end(boolean interrupted){
        intakeShooterSubsystem.setRollerPower(0);
        intakeShooterSubsystem.setStowPower(0);
    }
}
