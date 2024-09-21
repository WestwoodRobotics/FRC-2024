package frc.robot.commands.intakeShooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeShooter.IntakePivot;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;

/**
 * Command for setting the position of the intake shooter pivot.
 * This command uses a timer and a limit switch to determine when the pivot has reached the desired position.
 */
public class IntakeShooterPosition extends Command{
    Timer timer = new Timer();
    private IntakePivot intakePivotSubsystem;
    private CANSparkMax intakePivotMotorController;
    private IntakeShooterPositions intakeTargetPosition;
    private boolean isCommandFinished;

    /**
     * Constructs an IntakeShooterPosition command.
     * 
     * @param intakePivot The intake pivot subsystem.
     * @param position The target position for the intake shooter.
     */
    public IntakeShooterPosition(IntakePivot intakePivot, IntakeShooterPositions position){
        this.intakePivotSubsystem = intakePivot;
        intakePivotSubsystem.setHoldPoseState(false);
        this.intakeTargetPosition = position;
        this.isCommandFinished = false;
        addRequirements(intakePivotSubsystem);
    }

    /**
     * Initializes the command by resetting and starting the timer.
     */
    @Override
    public void initialize(){

        intakePivotSubsystem.setToPosition(intakeTargetPosition);

    }

    /**
     * Executes the command by setting the pivot power based on the target position and current limit switch status.
     */
    @Override
    public void execute(){
        intakePivotSubsystem.setPivotPower(intakePivotSubsystem.getCalculatedIntakePivotPIDValue());
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return true if the command has finished, false otherwise.
     */
    @Override
    public boolean isFinished(){
       if (intakePivotSubsystem.isPivotAtTargetPIDPosition()){
        System.out.println("Reached Pivot MOve Juggernaut lol");
       }
       return intakePivotSubsystem.isPivotAtTargetPIDPosition();
    }

    /**
     * Ends the command by stopping the pivot motor and setting the position state.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted){
        intakePivotSubsystem.stopAllMotors();
    }
}
