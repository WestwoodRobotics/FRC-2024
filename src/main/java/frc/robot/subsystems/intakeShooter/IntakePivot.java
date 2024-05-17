package frc.robot.subsystems.intakeShooter;
import com.revrobotics.CANSparkMax;
import java.util.HashMap;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.utils.Position_Enums.IntakeShooterPositions;
import frc.robot.subsystems.vision.LimitSwitch;

/**
 * The IntakePivot subsystem controls the pivot mechanism of the intake shooter.
 * It uses a CANSparkMax motor for movement and a PIDController for precise positioning.
 * A LimitSwitch is used to detect the home position.
 */
public class IntakePivot extends SubsystemBase {
    public CANSparkMax pivotMotor;
    private PIDController pivotPIDController;

    private IntakeShooterPositions intakeShooterPosition;
    private HashMap<IntakeShooterPositions, Double> pivotPositionValues = new HashMap<>();

    private boolean isPivotPIDControl;
    
    private double calculatedPivotPIDValue;

    private LimitSwitch limitSwitch;

    /**
     * Constructor for the IntakePivot subsystem.
     * 
     * @param limitSwitch The limit switch used to determine if the pivot is at the home position.
     */
    public IntakePivot(LimitSwitch limitSwitch){
        this.limitSwitch = limitSwitch;
        pivotMotor = new CANSparkMax(IntakeShooterConstants.kPivotMotorPort, MotorType.kBrushless);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        

        pivotPIDController = new PIDController(IntakeShooterConstants.kPivotP, 
        IntakeShooterConstants.kPivotI, IntakeShooterConstants.kPivotD);

        intakeShooterPosition = IntakeShooterPositions.HOME;

        
        pivotPositionValues.put(IntakeShooterPositions.INTAKE, IntakeShooterConstants.kIntakePivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.HOME, IntakeShooterConstants.kHomePivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.SHOOT_NEAR_SPEAKER, IntakeShooterConstants.kShootNearSpeakerPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.SHOOT_FAR_SPEAKER, IntakeShooterConstants.kShootFarSpeakerPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.AUTON_SHOOT, IntakeShooterConstants.kShootNearSpeakerAutonPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.AUTON_INTAKE, IntakeShooterConstants.kAutoIntakePivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.AMP, IntakeShooterConstants.kShootPivotAmp);
        pivotPositionValues.put(IntakeShooterPositions.SHOOT_NEAR_SPEAKER_FACING_FORWARDS, IntakeShooterConstants.kShootNearSpeakerFacingForwardsPivotPosition);
        pivotPositionValues.put(IntakeShooterPositions.PODIUM_SHOT, IntakeShooterConstants.kShootPodiumShot);
    }

    /**
     * Checks if the pivot has reached the limit switch.
     * 
     * @return True if the limit switch is activated, indicating the pivot is at the home position.
     */
    public boolean getPivotLimitReached()
    {
        return limitSwitch.getStatus();
    }

    /**
     * Sets the power for the pivot motor.
     * 
     * @param power The power level to set for the pivot motor.
     */
    public void setPivotPower(double power){
        isPivotPIDControl = false;
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
     * Moves the pivot to a specified position using PID control.
     * 
     * @param position The target position for the pivot.
     * @return True if the pivot has reached the target position, false otherwise.
     */
    public boolean setToPosition(IntakeShooterPositions position) {
        isPivotPIDControl = true;
        if (limitSwitch.getStatus() && ((position == IntakeShooterPositions.HOME))){
            return true;
        }
        double setPoint = pivotPositionValues.get(position);     
        pivotPIDController.setSetpoint(setPoint);

        intakeShooterPosition = position;
        return (Math.abs(pivotMotor.getEncoder().getPosition() - setPoint) <= 1);
    }   

    /**
     * Stops all motors in the IntakePivot subsystem.
     */
    public void stopAllMotors(){
        stopPivotMotor();
    }

    /**
     * Stops the pivot motor.
     */
    public void stopPivotMotor(){
        pivotMotor.set(0);
    }

    /**
     * Gets the current state of the IntakePivot subsystem.
     * 
     * @return The current state of the IntakePivot subsystem.
     */
    public IntakeShooterPositions getState() {
        return intakeShooterPosition;
    }

    /**
     * Resets the encoder for the pivot motor.
     */
    public void resetEncoder(){
        pivotMotor.getEncoder().setPosition(0);
    }

    /**
     * Sets the state of the IntakePivot subsystem.
     * 
     * @param position The new state for the IntakePivot subsystem.
     */
    public void setPositionState(IntakeShooterPositions position){
        intakeShooterPosition = position;
    }

    /**
     * Periodically updates the subsystem's state and handles PID control for the pivot.
     */
    @Override
    public void periodic(){
        SmartDashboard.putString ("Intake Shooter State" , intakeShooterPosition.toString()); 
        SmartDashboard.putNumber("Pivot Position", pivotMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Limit", this.getPivotLimitReached());

        if (isPivotPIDControl = true){
            calculatedPivotPIDValue = pivotPIDController.calculate(pivotMotor.getEncoder().getPosition());
            pivotMotor.set(calculatedPivotPIDValue);
        }
    }
}
