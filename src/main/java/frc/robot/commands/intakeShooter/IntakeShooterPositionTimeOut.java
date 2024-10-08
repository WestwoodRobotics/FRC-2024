package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * Command for setting the position of the intake shooter pivot with a timeout.
 * This command uses a timer to limit the duration of the command execution.
 */
public class IntakeShooterPositionTimeOut extends Command{
    Timer timer = new Timer();
    private IntakePivot intakePivotSubsystem;
    private IntakeShooterPositions intakeTargetPosition;
    private double commandTimeOutPeriod;
    private LimitSwitch limitSwitchSensor;

    /**
     * Constructor for the IntakeShooterPositionTimeOut command.
     * 
     * @param intakePivot The intake pivot subsystem.
     * @param position The target position for the intake shooter.
     * @param limitSwitch The limit switch used to detect the home position.
     * @param timeOutPeriod The maximum duration for the command.
     */
    public IntakeShooterPositionTimeOut(IntakePivot intakePivot, IntakeShooterPositions position, LimitSwitch limitSwitch, double timeOutPeriod){
        this.intakePivotSubsystem = intakePivot;
        this.intakeTargetPosition = position;
        this.commandTimeOutPeriod = timeOutPeriod;
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
        if(intakeTargetPosition == IntakeShooterPositions.HOME){
            if(!intakePivotSubsystem.getPivotLimitReached()){
                intakePivotSubsystem.setPivotPower(-0.75);
            }
            else{
                intakePivotSubsystem.setPivotPower(0);
                intakePivotSubsystem.resetEncoder();
            }
        }
        else{    
            intakePivotSubsystem.setToPosition(intakeTargetPosition);
        }
    }

    /**
     * Determines if the command is finished based on the timer or limit switch status.
     * 
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return (timer.get() > commandTimeOutPeriod) || (limitSwitchSensor.getStatus() && (intakeTargetPosition == (IntakeShooterPositions.HOME)));
    }

    /**
     * Ends the command by stopping the pivot motor and setting the position state.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            intakePivotSubsystem.setPivotPower(0);
        }
        intakePivotSubsystem.setPositionState(interrupted ? IntakeShooterPositions.MANUAL : intakeTargetPosition);
    }
}
