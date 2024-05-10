package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * Command to position the intake shooter with a timeout.
 * This command positions the intake pivot to a desired position with a specified timeout period.
 */
public class IntakeShooterPositionTimeOut extends Command{
    Timer t = new Timer();
    private IntakePivot intakePivotSubsystem; // Renamed for clarity
    private IntakeShooterPositions desiredPosition; // Renamed for clarity
    private double timeoutDuration; // Renamed for clarity
    private LimitSwitch limitSwitchSensor; // Renamed for clarity
    private boolean isFinished;

    /**
     * Constructor for the IntakeShooterPositionTimeOut command.
     * @param intakePivotSubsystem The intake pivot subsystem to be controlled
     * @param desiredPosition The target position for the intake shooter
     * @param limitSwitchSensor The limit switch sensor to detect position
     * @param timeoutDuration The timeout period for the command
     */
    public IntakeShooterPositionTimeOut(IntakePivot intakePivotSubsystem, IntakeShooterPositions desiredPosition, LimitSwitch limitSwitchSensor, double timeoutDuration){
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.desiredPosition = desiredPosition;
        this.timeoutDuration = timeoutDuration;
        addRequirements(intakePivotSubsystem);
        this.limitSwitchSensor = limitSwitchSensor;
    }

    @Override
    public void initialize(){
        t.reset();
        t.start();
    }

    @Override
    public void execute(){
        if(desiredPosition == IntakeShooterPositions.HOME){
            if(!intakePivotSubsystem.getPivotLimitReached()){
                intakePivotSubsystem.setPivotPower(-0.75);
                isFinished = false;
            }
            else{
                intakePivotSubsystem.setPivotPower(0);
                intakePivotSubsystem.resetEncoder();
                isFinished = true;
            }
        }
        else{    
            isFinished = intakePivotSubsystem.setToPosition(desiredPosition);
        }
    }

    @Override
    public boolean isFinished(){
       return (t.get() > timeoutDuration) || (limitSwitchSensor.getStatus() && (desiredPosition == (IntakeShooterPositions.HOME)));
    }

    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            intakePivotSubsystem.setPivotPower(0);
        }
        intakePivotSubsystem.setPositionState(interrupted ? IntakeShooterPositions.MANUAL : desiredPosition);
        System.out.println("finished!!");
    }
}
