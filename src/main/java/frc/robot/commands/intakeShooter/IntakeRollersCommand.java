package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeRollers;

/**
 * Command for controlling the intake rollers.
 * This command sets the power for the intake rollers and stow motors.
 */
public class IntakeRollersCommand extends Command{
    Timer timer = new Timer();
    private IntakeRollers intakeRollersSubsystem;
    private double intakeRollerPower;
    private double intakeStowPower;
    
    /**
     * Constructs an IntakeRollersCommand.
     * 
     * @param intakeRollers The intake rollers subsystem.
     * @param rollerPower The power to set for the rollers.
     * @param stowPower The power to set for the stow motor.
     */
    public IntakeRollersCommand(IntakeRollers intakeRollers, double rollerPower, double stowPower){
        intakeRollersSubsystem = intakeRollers;
        this.intakeRollerPower = rollerPower;
        this.intakeStowPower = stowPower;
        addRequirements(intakeRollers);
    }

    /**
     * Initializes the command.
     * Sets the power for the intake rollers and stow motor.
     */
    @Override
    public void initialize(){
        intakeRollersSubsystem.setRollerPower(-1*intakeRollerPower);
        intakeRollersSubsystem.setStowPower(intakeStowPower);
    }

    /**
     * Executes the command.
     * Currently, this method does not perform any actions.
     */
    @Override
    public void execute(){

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
     * Ends the command.
     * Currently, this method does not perform any actions.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){

    }
}
