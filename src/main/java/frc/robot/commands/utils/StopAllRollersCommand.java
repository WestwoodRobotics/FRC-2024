package frc.robot.commands.utils;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeRollers;

/**
 * Command to stop all rollers in the intake and elevator subsystems.
 * This command sets the power of all rollers to zero.
 */
public class StopAllRollersCommand extends Command {
     private IntakeRollers intakeRollersSubsystem;
     private Elevator elevatorSubsystem;

    /**
     * Constructs a new StopAllRollersCommand.
     * 
     * @param intakeShooter The intake shooter subsystem.
     * @param elevator The elevator subsystem.
     */
    public StopAllRollersCommand(IntakeRollers intakeShooter, Elevator elevator){
        intakeRollersSubsystem = intakeShooter;
        elevatorSubsystem = elevator;
        addRequirements(intakeShooter, elevator);
    }
    
    /**
     * Sets the power of all rollers to zero when the command is executed.
     */
    @Override
    public void execute(){
        intakeRollersSubsystem.setRollerPower(0);
        intakeRollersSubsystem.setStowPower(0);
        elevatorSubsystem.setRollerPower(0);
    }

    /**
     * Determines if the command is finished.
     * 
     * @return true to indicate the command should finish.
     */
    @Override
    public boolean isFinished(){
        return true;
    }

    /**
     * Called when the command ends or is interrupted.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override 
    public void end(boolean interrupted){
        intakeRollersSubsystem.setRollerPower(0);
        intakeRollersSubsystem.setStowPower(0);
        elevatorSubsystem.setRollerPower(0);
    }
}
