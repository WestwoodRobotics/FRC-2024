package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeRollers;

/**
 * Command to control the power of intake rollers and stow motor.
 * This command sets the power for the intake rollers and stow motor in the IntakeRollers subsystem.
 */
public class IntakeRollersCommand extends Command{
    Timer t = new Timer();
    private IntakeRollers intakeRollersSubsystem; // Renamed for clarity
    private double desiredRollerPower; // Renamed for clarity
    private double desiredStowPower; // Renamed for clarity
    

    /**
     * Constructs an IntakeRollersCommand.
     * @param intakeRollersSubsystem The intake rollers subsystem to be controlled
     * @param desiredRollerPower The power to set for the rollers
     * @param desiredStowPower The power to set for the stow motor
     */
    public IntakeRollersCommand(IntakeRollers intakeRollersSubsystem, double desiredRollerPower, double desiredStowPower){
        this.intakeRollersSubsystem = intakeRollersSubsystem;
        this.desiredRollerPower = desiredRollerPower;
        this.desiredStowPower = desiredStowPower;
        addRequirements(intakeRollersSubsystem);
    }

    /**
     * Initializes the command by setting the roller and stow motor power.
     */
    @Override
    public void initialize(){
        intakeRollersSubsystem.setRollerPower(-1*desiredRollerPower);
        intakeRollersSubsystem.setStowPower(desiredStowPower);
    }

    /**
     * Executes the command. This method is called repeatedly until the command finishes.
     */
    @Override
    public void execute(){

    }

    /**
     * Determines if the command has finished. This command is instantaneous and finishes immediately after initialization.
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

    }
}
