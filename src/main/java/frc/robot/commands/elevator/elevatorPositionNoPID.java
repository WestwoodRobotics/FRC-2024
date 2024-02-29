package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class elevatorPositionNoPID extends Command{
    private Elevator m_elevator;
    private ElevatorPositions targetPosition;
    private boolean isFinished;

    public elevatorPositionNoPID(Elevator elevator, ElevatorPositions position){
        m_elevator = elevator;
        this.targetPosition = position;
        addRequirements(m_elevator);
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
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
       m_elevator.setPivotPositionNOPID(targetPosition);

    }

    /**
     * Returns whether the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
        return (m_elevator.getElevatorPosition() == targetPosition);
    }
}