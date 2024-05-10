package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

/**
 * Command to control the intake shooter subsystem.
 * This command sets the power for the roller and stow motors of the intake shooter.
 */
public class IntakeCommand extends Command{
    private IntakeShooter m_intakeShooter;
    private double rollerPower;
    private double stowPower;


    private boolean isFinished;

    /**
     * Constructs an IntakeCommand.
     * 
     * @param intakeShooter The IntakeShooter subsystem this command will run on.
     * @param rollerPower The power to set for the roller motor.
     * @param stowPower The power to set for the stow motor.
     */
    public IntakeCommand(IntakeShooter intakeShooter, double rollerPower, double stowPower){
        m_intakeShooter = intakeShooter;
        this.rollerPower = rollerPower;
        this.stowPower = stowPower;

        addRequirements(intakeShooter);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){

    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Sets the power for the roller and stow motors.
     */
    @Override
    public void execute(){
        m_intakeShooter.setRollerPower(rollerPower);
        m_intakeShooter.setStowPower(stowPower);
    }

    
}
