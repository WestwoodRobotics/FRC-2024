package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

/**
 * Command to set the IntakeShooter subsystem to a specified position.
 */
public class IntakeShooterPosition extends Command{
    Timer timer = new Timer();
    private IntakeShooter intakeShooter;
    private IntakeShooterPositions targetPosition;

    private boolean isFinished;

    /**
     * Constructs an IntakeShooterPosition command.
     * 
     * @param intakeShooter The IntakeShooter subsystem this command will run on.
     * @param position The target position for the IntakeShooter.
     */
    public IntakeShooterPosition(IntakeShooter intakeShooter, IntakeShooterPositions position){
        this.intakeShooter = intakeShooter;
        this.targetPosition = position;
        addRequirements(intakeShooter);
    }

    /**
     * Called when the command is initially scheduled.
     * Resets and starts the timer.
     */
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Sets the IntakeShooter to the target position.
     */
    @Override
    public void execute(){
        isFinished = intakeShooter.setToPosition(targetPosition);
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return (timer.get() > 3);
    }
}
