package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakeRollers;

/**
 * Command for controlling the intake rollers.
 * This command sets the power for the intake rollers and stow motors.
 */
public class IntakeRollersCommand extends Command{
    Timer t = new Timer();
    private IntakeRollers m_intakeRollers;
    private double rollerPower;
    private double stowPower;
    
    /**
     * Constructs an IntakeRollersCommand.
     * 
     * @param intakeRollers The intake rollers subsystem.
     * @param rollerPower The power to set for the rollers.
     * @param stowPower The power to set for the stow motor.
     */
    public IntakeRollersCommand(IntakeRollers intakeRollers, double rollerPower, double stowPower){
        m_intakeRollers = intakeRollers;
        this.rollerPower = rollerPower;
        this.stowPower = stowPower;
        addRequirements(intakeRollers);
    }

    /**
     * Initializes the command.
     * Sets the power for the intake rollers and stow motor.
     */
    @Override
    public void initialize(){
        m_intakeRollers.setRollerPower(-1*rollerPower);
        m_intakeRollers.setStowPower(stowPower);
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
