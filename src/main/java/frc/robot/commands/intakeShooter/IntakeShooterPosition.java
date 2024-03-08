package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intakeShooter.IntakeShooter;
import frc.robot.subsystems.utils.Position_Enums.ElevatorPositions;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

public class IntakeShooterPosition extends Command{
    Timer t = new Timer();
    private IntakeShooter m_intakeShooter;
    private IntakeShooterPositions targetPosition;

    private boolean isFinished;


    public IntakeShooterPosition(IntakeShooter intakeShooter, IntakeShooterPositions position){
        this.m_intakeShooter = intakeShooter;
        this.targetPosition = position;

    }

    /**
     * Called when the command is initially scheduled.
     * Sets the RPM of the motors if the IntakeShooter is in the correct state.
     */
    @Override
    public void initialize(){
        m_intakeShooter.setRollerPower(0);
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
        isFinished = m_intakeShooter.setToPosition(targetPosition);
        if (targetPosition == IntakeShooterPositions.SHOOT_FAR_SPEAKER || targetPosition == IntakeShooterPositions.SHOOT_NEAR_SPEAKER){
            m_intakeShooter.setRollerPower(-1);
        }
        else{
            m_intakeShooter.setRollerPower(0);
        }
        

    }

    /**
     * Returns whether the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
        
       return (t.get() > 3) || isFinished;
    }

    @Override
    public void end(boolean interrupted){
        m_intakeShooter.setRollerPower(0);
        m_intakeShooter.setStowPower(0);
        m_intakeShooter.setPositionState( interrupted ? IntakeShooterPositions.MANUAL : targetPosition);
    }
    
}