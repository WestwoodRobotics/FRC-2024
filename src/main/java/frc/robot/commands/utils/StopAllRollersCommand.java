package frc.robot.commands.utils;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeRollers;

/**
 * Command to stop all rollers in the robot.
 * This command stops the rollers in both the intake and elevator subsystems.
 */
public class StopAllRollersCommand extends CommandBase {
    private IntakeRollers intakeRollersSubsystem; // Renamed for clarity
    private Elevator elevatorSubsystem; // Renamed for clarity

    /**
     * Constructs a new StopAllRollersCommand.
     * @param intakeRollersSubsystem The intake rollers subsystem
     * @param elevatorSubsystem The elevator subsystem
     */
    public StopAllRollersCommand(IntakeRollers intakeRollersSubsystem, Elevator elevatorSubsystem){
        this.intakeRollersSubsystem = intakeRollersSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(intakeRollersSubsystem, elevatorSubsystem);
    }
    
    /**
     * Stops the rollers in both the intake and elevator subsystems.
     */
    @Override
    public void execute(){
        intakeRollersSubsystem.setRollerPower(0);
        intakeRollersSubsystem.setStowPower(0);
        elevatorSubsystem.setRollerPower(0);
    }

    /**
     * Determines if the command has finished.
     * @return true to indicate the command has finished.
     */
    @Override
    public boolean isFinished(){
        return true;
    }

    /**
     * Called after the command ends or is interrupted.
     * @param interrupted indicates if the command was interrupted.
     */
    @Override 
    public void end(boolean interrupted){
        intakeRollersSubsystem.setRollerPower(0);
        intakeRollersSubsystem.setStowPower(0);
        elevatorSubsystem.setRollerPower(0);
    }
}
