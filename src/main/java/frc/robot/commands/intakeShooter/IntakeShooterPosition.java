package frc.robot.commands.intakeShooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * Command to position the intake shooter to a specified position.
 * This command adjusts the intake pivot to a desired position based on the IntakeShooterPositions enum.
 */
public class IntakeShooterPosition extends Command{
    Timer t = new Timer();
    private IntakePivot intakePivotSubsystem; // Renamed for clarity
    private IntakeShooterPositions desiredPosition; // Renamed for clarity
    
    private LimitSwitch limitSwitchSensor; // Renamed for clarity

    private boolean shouldChangePower = true; // Renamed for clarity
    private boolean isFinished;
    private static boolean isAlreadyPressed;

    /**
     * Constructs an IntakeShooterPosition command.
     * @param intakePivotSubsystem The intake pivot subsystem to be controlled
     * @param desiredPosition The target position for the intake shooter
     * @param limitSwitchSensor The limit switch sensor to detect position
     */
    public IntakeShooterPosition(IntakePivot intakePivotSubsystem, IntakeShooterPositions desiredPosition, LimitSwitch limitSwitchSensor
    ){
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.desiredPosition = desiredPosition;
        addRequirements(intakePivotSubsystem);
        this.limitSwitchSensor = limitSwitchSensor;
    }

    /**
     * Constructs an IntakeShooterPosition command with power change option.
     * @param intakePivotSubsystem The intake pivot subsystem to be controlled
     * @param desiredPosition The target position for the intake shooter
     * @param shouldChangePower Flag to indicate if power should be changed
     * @param limitSwitchSensor The limit switch sensor to detect position
     */
    public IntakeShooterPosition(IntakePivot intakePivotSubsystem, IntakeShooterPositions desiredPosition, boolean shouldChangePower, LimitSwitch limitSwitchSensor){
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.desiredPosition = desiredPosition;
        this.shouldChangePower = shouldChangePower;
        addRequirements(intakePivotSubsystem);
        this.limitSwitchSensor = limitSwitchSensor;
    }

    /**
     * Initializes the command with a timer reset.
     */
    @Override
    public void initialize(){
        isAlreadyPressed = intakePivotSubsystem.getPivotLimitReached();
        t.reset();
        t.start();
    }

    /**
     * Executes the command, adjusting the intake pivot to the desired position.
     */
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

    /**
     * Determines if the command has finished.
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       return (t.get() > 2.0) || (limitSwitchSensor.getStatus() && (desiredPosition == (IntakeShooterPositions.HOME))) || isFinished;
    }

    /**
     * Ends the command, stopping the intake pivot.
     * @param interrupted Whether the command was interrupted or not
     */
    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            intakePivotSubsystem.setPivotPower(0);
        }
        intakePivotSubsystem.setPositionState(interrupted ? IntakeShooterPositions.MANUAL : desiredPosition);
        System.out.println("finished!!");
    }
}
