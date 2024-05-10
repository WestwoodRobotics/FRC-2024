package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;

/**
 * Command to move the elevator to a specified position without using PID control.
 */
public class elevatorPositionNoPID extends Command{
    Timer timerOne = new Timer();
    Timer timerTwo = new Timer();
    private Elevator m_elevator;
    private ElevatorPositions targetPosition;
    private boolean isFinished;

    /**
     * Constructs an instance of the elevatorPositionNoPID command.
     * 
     * @param elevator The Elevator subsystem this command will run on.
     * @param position The target position for the elevator.
     */
    public elevatorPositionNoPID(Elevator elevator, ElevatorPositions position){
        m_elevator = elevator;
        this.targetPosition = position;
        addRequirements(m_elevator);
    }

    /**
     * Called when the command is initially scheduled.
     * Resets and starts the timers.
     */
    @Override
    public void initialize(){
        timerOne.reset();
        timerOne.start();
        timerTwo.reset();
        timerTwo.start();
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Moves the elevator to the target position without PID control.
     */
    @Override
    public void execute(){
        m_elevator.setPivotPositionNOPID(targetPosition);
        isFinished = m_elevator.setElevatorPosition(targetPosition);
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return (timerOne.get() > 0.7) && ((timerTwo.get() > 2) || isFinished);
    }
}
