package frc.robot.subsystems.intakeShooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * The IntakePivot subsystem controls the pivoting mechanism of the intake shooter.
 * It allows for the adjustment of the intake shooter's angle for optimal ball intake and shooting.
 */
public class IntakePivot extends SubsystemBase {
    // Motor controller for the pivot mechanism
    public CANSparkMax pivotMotor;
    // PID controller for precise control of the pivot position
    private PIDController pivotPIDController;

    // Current position state of the intake shooter
    private IntakeShooterPositions intakeShooterPosition;

    // Limit switch to detect the home position
    private LimitSwitch limitSwitch;

    /**
     * Constructs an IntakePivot subsystem with a limit switch for home position detection.
     * 
     * @param limitSwitch The limit switch used to detect the home position of the pivot.
     */
    public IntakePivot(LimitSwitch limitSwitch){
        this.limitSwitch = limitSwitch;
        pivotMotor = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        
        pivotPIDController = new PIDController(IntakeShooterConstants.kPivotP, 
        IntakeShooterConstants.kPivotI, IntakeShooterConstants.kPivotD);

        intakeShooterPosition = IntakeShooterPositions.HOME;
    }

    /**
     * Sets the power for the pivot motor.
     * 
     * @param power The power to set for the pivot motor, ranging from -1.0 to 1.0.
     */
    public void setPivotPower(double power){
        if (this.getPivotLimitReached() && power < 0){
            pivotMotor.set(0);
            this.resetEncoder();
        }
        else{
            pivotMotor.set(power);
            intakeShooterPosition = IntakeShooterPositions.MANUAL;
        }
    }

    /**
     * Checks if the pivot has reached its limit switch indicating the home position.
     * 
     * @return True if the limit switch is activated, false otherwise.
     */
    public boolean getPivotLimitReached() {
        return limitSwitch.getStatus();
    }

    /**
     * Resets the encoder for the pivot motor.
     */
    public void resetEncoder(){
        pivotMotor.getEncoder().setPosition(0);
    }

    /**
     * Sets the desired position state for the intake shooter pivot.
     * 
     * @param position The desired position state from the IntakeShooterPositions enum.
     */
    public void setPositionState(IntakeShooterPositions position){
        intakeShooterPosition = position;
    }

    @Override
    public void periodic(){
        // Update SmartDashboard with the current state and pivot position
        SmartDashboard.putString("Intake Shooter State", intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Limit Switch", this.getPivotLimitReached());
    }
}
