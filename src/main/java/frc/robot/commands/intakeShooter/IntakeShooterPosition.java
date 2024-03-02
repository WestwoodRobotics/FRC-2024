package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class IntakeShooterPosition extends Command{
    Timer t = new Timer();
    private IntakeShooter i;
    private IntakeShooterPositions ii;

    private boolean isFinished;


    public IntakeShooterPosition(IntakeShooter intakeShooter, IntakeShooterPositions position){
        i = intakeShooter;
        this.ii = position;
        addRequirements(i);
    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        t.reset();
        t.start();
        
    }

    /**
     * Called every time the scheduler runs while the command is scheduled.
     * Checks if the motors are at their target RPM and either finishes the command or resets the timer.
     */
    @Override
    public void execute(){
       //m_elevator.setElevatorPositionNOPID(targetPosition);
        //    m_elevator.setPivotPosition(targetPosition);
        isFinished = i.setToPosition(ii);
    }

    /**
     * Returns whether the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return (t.get() > 3);
    }
}