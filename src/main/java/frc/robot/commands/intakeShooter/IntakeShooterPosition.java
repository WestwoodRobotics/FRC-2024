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
    Timer timer = new Timer();
    private IntakePivot intakePivotSubsystem;
    private IntakeShooterPositions intakeTargetPosition;
    
    private LimitSwitch limitSwitchSensor;

    private boolean isCommandFinished;

    /**
     * Constructs an IntakeShooterPosition command.
     * 
     * @param intakePivot The intake pivot subsystem.
     * @param position The target position for the intake shooter.
     * @param limitSwitch The limit switch used to detect the home position.
     */
    public IntakeShooterPosition(IntakePivot intakePivot, IntakeShooterPositions position, LimitSwitch limitSwitch
    ){
        this.intakePivotSubsystem = intakePivot;
        this.intakeTargetPosition = position;
        addRequirements(intakePivotSubsystem);
        this.limitSwitchSensor = limitSwitch;
    }

    /**
     * Initializes the command by resetting and starting the timer.
     */
    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    /**
     * Executes the command by setting the pivot power based on the target position and current limit switch status.
     */
    @Override
    public void execute(){
        isCommandFinished = intakePivotSubsystem.setToPosition(intakeTargetPosition);
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return isCommandFinished;
    }

    /**
     * Ends the command by stopping the pivot motor and setting the position state.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){
        intakePivotSubsystem.setPivotPower(0);
        intakePivotSubsystem.setPositionState(interrupted ? IntakeShooterPositions.MANUAL : intakeTargetPosition);
    }
}
