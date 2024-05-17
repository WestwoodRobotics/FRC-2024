package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * Command for setting the position of the intake shooter pivot.
 * This command uses a timer and a limit switch to determine when the pivot has reached the desired position.
 */
public class IntakeShooterPosition extends Command{
    Timer t = new Timer();
    private IntakePivot m_intakePivot;
    private IntakeShooterPositions targetPosition;
    
    private LimitSwitch l;

    private boolean isFinished;

    /**
     * Constructs an IntakeShooterPosition command.
     * 
     * @param intakePivot The intake pivot subsystem.
     * @param position The target position for the intake shooter.
     * @param limitSwitch The limit switch used to detect the home position.
     */
    public IntakeShooterPosition(IntakePivot intakePivot, IntakeShooterPositions position, LimitSwitch limitSwitch
    ){
        this.m_intakePivot = intakePivot;
        this.targetPosition = position;
        addRequirements(m_intakePivot);
        this.l = limitSwitch;
    }

    /**
     * Initializes the command by resetting and starting the timer.
     */
    @Override
    public void initialize(){
        t.reset();
        t.start();
    }

    /**
     * Executes the command by setting the pivot power based on the target position and current limit switch status.
     */
    @Override
    public void execute(){
        if(targetPosition == IntakeShooterPositions.HOME){
            if(!m_intakePivot.getPivotLimitReached()){
                m_intakePivot.setPivotPower(-0.75);
                isFinished = false;
            }
            else{
                m_intakePivot.setPivotPower(0);
                m_intakePivot.resetEncoder();
                isFinished = true;
            }
        }
        else{    
            isFinished = m_intakePivot.setToPosition(targetPosition);
        }
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return (t.get() > 2.0) || (l.getStatus() && (targetPosition == (IntakeShooterPositions.HOME))) || isFinished;
    }

    /**
     * Ends the command by stopping the pivot motor and setting the position state.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            m_intakePivot.setPivotPower(0);
        }
        m_intakePivot.setPositionState(interrupted ? IntakeShooterPositions.MANUAL : targetPosition);
    }
}
